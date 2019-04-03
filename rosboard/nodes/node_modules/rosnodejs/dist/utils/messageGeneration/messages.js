'use strict';

var fs = require('fs');
var path = require('path');
var md5 = require('md5');
var async = require('async');

var packages = require('./packages'),
    fieldsUtil = require('./fields'),
    MessageSpec = require('./MessageSpec.js'),
    MessageLoader = require('./MessageLoader.js');

var messages = exports;

// ---------------------------------------------------------
// exported functions

/** get message handler class from registry */
messages.getFromRegistry = function (messageType, type) {
  return getMessageFromRegistry(messageType, type);
};

messages.getPackageFromRegistry = function (packagename) {
  return registry[packagename];
};

/** ensure the handler for this message type is in the registry,
 * create it if it doesn't exist */
messages.getMessage = function (messageType, callback) {
  var fromRegistry = getMessageFromRegistry(messageType, ["msg"]);
  if (fromRegistry) {
    callback(null, fromRegistry);
  } else {
    getMessageFromPackage(messageType, "msg", callback);
  }
};

/** ensure the handler for requests for this service type is in the
 * registry, create it if it doesn't exist */
messages.getService = function (messageType, callback) {
  getMessageFromPackage(messageType, "srv", callback);
};

/** get all message and service definitions, from all packages */
messages.getAll = function () {
  const msgLoader = new MessageLoader(false);
  return msgLoader.buildPackageTree(null, false).then(() => {
    const pkgCache = msgLoader.getCache();

    return new Promise((resolve, reject) => {
      async.eachSeries(Object.keys(pkgCache), (pkgName, pkgCallback) => {
        async.eachSeries(['messages', 'services'], (type, typeCallback) => {
          let x = type;
          const msgs = pkgCache[pkgName][type];
          async.eachSeries(Object.keys(msgs), (msgName, msgCallback) => {
            // console.log('build %s', msgName)
            try {
              buildMessageFromSpec(msgs[msgName].msgSpec);
            } catch (err) {
              console.error('Error building %s: %s\n%s', msgName, err, err.stack);
              throw err;
            }
            msgCallback();
          }, typeCallback);
        }, pkgCallback);
      }, resolve);
    });
  });
};

// ---------------------------------------------------------
// Registry

var registry = {};
/*
   registry looks like:
  { 'packagename':
    {
      msg: {
        'String': classdef,
        'Pose': classdef,
        ...
      },
      srv: { Request:
             {
               'SetBool': classdef,
               ...
             },
             Response:
             {
               'SetBool': classdef,
               ...
             }
           }
    },
    'packagename2': {..}
  };
*/

/**
   @param messageType is the ROS message or service type, e.g.
   'std_msgs/String'
   @param type is from the set
   ["msg", "srv"]
*/
function getMessageFromRegistry(messageType, type) {
  var packageName = getPackageNameFromMessageType(messageType);
  var messageName = getMessageNameFromMessageType(messageType);
  var packageSection = registry[packageName];
  if (!packageSection) {
    return undefined;
  }
  var section = registry[packageName][type]; // msg or srv sub-object
  if (!section) {
    return undefined;
  }
  return section[messageName];
}

/**
    @param messageType is the ROS message or service type, e.g.
    'std_msgs/String'
    @param message is the message class definition
    @param type is from the set "msg", "srv"
    @param (optional) subtype \in { "Request", "Response" }
*/
function setMessageInRegistry(messageType, message, type) {

  var packageName = getPackageNameFromMessageType(messageType);
  var messageName = getMessageNameFromMessageType(messageType);

  if (!registry[packageName]) {
    registry[packageName] = { msg: {}, srv: {} };
  }

  if (type != MessageSpec.SRV_TYPE) {
    // message
    registry[packageName]['msg'][messageName] = message;
  } else {
    // service
    registry[packageName]['srv'][messageName] = message;
  }
}

// ---------------------------------------------------------
// private functions

/* get message or service definition class */
function getMessageFromPackage(messageType, type, callback) {
  var packageName = getPackageNameFromMessageType(messageType);
  var messageName = getMessageNameFromMessageType(messageType);
  packages.findPackage(packageName, function (error, directory) {
    var filePath;
    filePath = path.join(directory, type, messageName + '.' + type);
    getMessageFromFile(messageType, filePath, type, callback);
  });
};

function buildMessageFromSpec(msgSpec) {
  const type = msgSpec.type;

  const fullMsg = msgSpec.getFullMessageName();
  switch (type) {
    case MessageSpec.MSG_TYPE:
    case MessageSpec.ACTION_GOAL_TYPE:
    case MessageSpec.ACTION_FEEDBACK_TYPE:
    case MessageSpec.ACTION_RESULT_TYPE:
    case MessageSpec.ACTION_ACTION_GOAL_TYPE:
    case MessageSpec.ACTION_ACTION_FEEDBACK_TYPE:
    case MessageSpec.ACTION_ACTION_RESULT_TYPE:
    case MessageSpec.ACTION_ACTION_TYPE:
      setMessageInRegistry(fullMsg, buildMessageClass(msgSpec), type);
      break;
    case MessageSpec.SRV_TYPE:
      {
        const Request = buildMessageClass(msgSpec.request);
        const Response = buildMessageClass(msgSpec.response);
        const md5Sum = msgSpec.getMd5sum();
        const service = {
          Request,
          Response,
          md5sum: () => {
            return md5Sum;
          },
          datatype: () => {
            return fullMsg;
          }
        };
        setMessageInRegistry(fullMsg, service, type);
        break;
      }
    default:
      console.warn("Unknown msgspec type:", type);
  }
};

function parseMessageFile(fileName, details, type, callback) {
  details = details || {};
  fs.readFile(fileName, 'utf8', function (error, content) {
    if (error) {
      return callback(error);
    } else {
      extractFields(content, details, function (error, aggregate) {
        if (error) {
          callback(error);
        } else {
          if (type == "msg") {
            details.constants = aggregate[0].constants;
            details.fields = aggregate[0].fields;
            details.md5 = calculateMD5(details, "msg");
            callback(null, details);
          } else if (type == "srv") {
            // services combine the two message types to compute the
            // md5sum
            var rtv = {
              // we need to clone what's already there in details
              // into the sub-objects
              request: JSON.parse(JSON.stringify(details)),
              response: JSON.parse(JSON.stringify(details))
            };
            rtv.request.constants = aggregate[0].constants;
            rtv.request.fields = aggregate[0].fields;
            if (aggregate.length > 1) {
              // if there is a response:
              rtv.response.constants = aggregate[1].constants;
              rtv.response.fields = aggregate[1].fields;
            } else {
              rtv.response.constants = [];
              rtv.response.fields = [];
            }
            rtv.request.md5 = rtv.response.md5 = calculateMD5(rtv, "srv");
            callback(null, rtv);
          } else {
            console.log("parseMessageFile:", "Unknown type: ", type);
            callback("unknown type", null);
          }
        }
      });
    }
  });
};

// -------------------------------
// functions relating to handler class

function calculateMD5(details, type) {

  /* get the text for one part of the type definition to compute the
     md5sum over */
  function getMD5text(part) {
    var message = '';
    var constants = part.constants.map(function (field) {
      return field.type + ' ' + field.name + '=' + field.raw;
    }).join('\n');

    var fields = part.fields.map(function (field) {
      if (field.messageType) {
        return field.messageType.md5 + ' ' + field.name;
      } else {
        return field.type + ' ' + field.name;
      }
    }).join('\n');

    message += constants;
    if (message.length > 0 && fields.length > 0) {
      message += "\n";
    }
    message += fields;
    return message;
  }

  // depending on type, compose the right md5text to compute md5sum
  // over: Services just concatenate the individual message text (with
  // *no* new line in between)
  var text;
  if (type == "msg") {
    text = getMD5text(details);
  } else if (type == "srv") {
    text = getMD5text(details.request);
    text += getMD5text(details.response);
  } else {
    console.log("calculateMD5: Unknown type", type);
    return null;
  }

  return md5(text);
}

function extractFields(content, details, callback) {
  function parsePart(lines, callback) {
    var constants = [],
        fields = [];

    var parseLine = function parseLine(line, callback) {
      line = line.trim();

      var lineEqualIndex = line.indexOf('='),
          lineCommentIndex = line.indexOf('#');
      if (lineEqualIndex === -1 || lineCommentIndex === -1 || lineEqualIndex >= lineCommentIndex) {
        line = line.replace(/#.*/, '');
      }

      if (line === '') {
        callback();
      } else {
        var firstSpace = line.indexOf(' '),
            fieldType = line.substring(0, firstSpace),
            field = line.substring(firstSpace + 1),
            equalIndex = field.indexOf('='),
            fieldName = field.trim();

        if (equalIndex !== -1) {
          fieldName = field.substring(0, equalIndex).trim();
          // truncate comments and whitespace
          var constant = field.substring(equalIndex + 1, field.length).trim();
          var constantEnd = constant.length;
          var constantComment = constant.indexOf("#");
          if (constantComment >= 0) {
            if (fieldType != "string") {
              var firstWhitespace = constant.match(/\s/);
              if (firstWhitespace) {
                constantEnd = firstWhitespace.index;
              }
            } else {
              constantEnd = constantComment;
            }
          }
          var parsedConstant = fieldsUtil.parsePrimitive(fieldType, constant);

          constants.push({
            name: fieldName,
            type: fieldType,
            value: parsedConstant,
            raw: constant.slice(0, constantEnd),
            index: fields.length,
            messageType: null
          });
          callback();
        } else {
          if (fieldsUtil.isPrimitive(fieldType)) {
            fields.push({
              name: fieldName.trim(),
              type: fieldType,
              index: fields.length,
              messageType: null
            });
            callback();
          } else if (fieldsUtil.isArray(fieldType)) {
            var arrayType = fieldsUtil.getTypeOfArray(fieldType);
            if (fieldsUtil.isMessage(arrayType)) {
              fieldType = normalizeMessageType(fieldType, details.packageName);
              arrayType = normalizeMessageType(arrayType, details.packageName);
              messages.getMessage(arrayType, function (error, messageType) {
                fields.push({
                  name: fieldName.trim(),
                  type: fieldType,
                  index: fields.length,
                  messageType: messageType
                });
                callback();
              });
            } else {
              fields.push({
                name: fieldName.trim(),
                type: fieldType,
                index: fields.length,
                messageType: null
              });
              callback();
            }
          } else {
            fieldType = normalizeMessageType(fieldType, details.packageName);
            messages.getMessage(fieldType, function (error, messageType) {
              fields.push({
                name: fieldName.trim(),
                type: fieldType,
                index: fields.length,
                messageType: messageType
              });
              callback();
            });
          }
        }
      }
    };

    async.eachSeries(lines, parseLine, function (error) {
      if (error) {
        callback(error);
      } else {
        callback(null, { constants: constants, fields: fields });
      }
    });
  }

  var lines = content.split('\n');

  // break into parts:
  var parts = lines.reduce(function (memo, line) {
    if (line == "---") {
      // new part starts
      memo.push([]);
    } else if (line != "") {
      memo[memo.length - 1].push(line);
    }
    return memo;
  }, [[]]);

  async.map(parts, parsePart, function (err, aggregate) {
    callback(err, aggregate);
  });
};

function camelCase(underscoreWord, lowerCaseFirstLetter) {
  var camelCaseWord = underscoreWord.split('_').map(function (word) {
    return word[0].toUpperCase() + word.slice(1);
  }).join('');

  if (lowerCaseFirstLetter) {
    camelCaseWord = camelCaseWord[0].toLowerCase() + camelCaseWord.slice(1);
  }

  return camelCaseWord;
}

/** Construct the class definition for the given message type. The
 * resulting class holds the data and has the methods required for
 * use with ROS, incl. serialization, deserialization, and md5sum. */
function buildMessageClass(msgSpec) {

  function Message(values) {
    if (!(this instanceof Message)) {
      return new Message(values);
    }

    var that = this;

    if (msgSpec.fields) {
      msgSpec.fields.forEach(function (field) {
        if (!field.isBuiltin) {
          // sub-message class
          // is it an array?
          if (values && typeof values[field.name] != "undefined") {
            // values provided
            if (field.isArray) {
              that[field.name] = values[field.name].map(function (value) {
                return new (getMessageFromRegistry(field.baseType, 'msg'))(value);
              });
            } else {
              that[field.name] = new (getMessageFromRegistry(field.baseType, 'msg'))(values[field.name]);
            }
          } else {
            // use defaults
            if (field.isArray) {
              // it's an array
              const length = field.arrayLen || 0;
              that[field.name] = new Array(length).fill(new (getMessageFromRegistry(field.baseType, 'msg'))());
            } else {
              that[field.name] = new (getMessageFromRegistry(field.baseType, 'msg'))();
            }
          }
        } else {
          // simple type
          that[field.name] = values && typeof values[field.name] != "undefined" ? values[field.name] : field.value || fieldsUtil.getDefaultValue(field.type);
        }
      });
    }
  };

  Message.messageType = msgSpec.getFullMessageName();
  // TODO: bring these back?
  // Message.packageName = details.packageName;
  // Message.messageName = Message.prototype.messageName = details.messageName;
  // Message.md5         = Message.prototype.md5         = details.md5;
  const md5Sum = msgSpec.getMd5sum();
  Message.md5sum = function () {
    return md5Sum;
  };
  Message.Constants = (() => {
    const ret = {};
    msgSpec.constants.forEach(constant => {
      ret[constant.name.toUpperCase()] = constant.value;
    });
    return ret;
  })();
  Message.fields = msgSpec.fields;
  Message.serialize = function (obj, buffer, offset) {
    serializeInnerMessage(msgSpec, obj, buffer, offset);
  };
  Message.deserialize = function (buffer) {
    var message = new Message();

    message = deserializeInnerMessage(msgSpec, message, buffer, [0]);

    return message;
  };
  Message.getMessageSize = function (msg) {
    return fieldsUtil.getMessageSize(msg, msgSpec);
  };

  const fullMsgDefinition = msgSpec.computeFullText();
  Message.messageDefinition = function () {
    return fullMsgDefinition;
  };
  Message.datatype = function () {
    return msgSpec.getFullMessageName();
  };

  return Message;
}

// ---------------------------------------------------------

function getMessageType(packageName, messageName) {
  return packageName ? packageName + '/' + messageName : messageName;
}

function getPackageNameFromMessageType(messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[0] : '';
}

function normalizeMessageType(messageType, packageName) {
  var normalizedMessageType = messageType;
  if (messageType == "Header") {
    normalizedMessageType = getMessageType("std_msgs", messageType);
  } else if (messageType.indexOf("/") < 0) {
    normalizedMessageType = getMessageType(packageName, messageType);
  }
  return normalizedMessageType;
}

function getMessageNameFromMessageType(messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[1] : messageType;
}

// ---------------------------------------------------------
// Serialize

function serializeInnerMessage(spec, message, buffer, bufferOffset) {
  spec.fields.forEach(function (field) {
    var fieldValue = message[field.name];

    if (field.isArray) {
      if (field.arrayLen === null) {
        buffer.writeUInt32LE(fieldValue.length, bufferOffset);
        bufferOffset += 4; // only for variable length arrays
      }

      var arrayType = field.baseType;
      fieldValue.forEach(function (value) {
        if (field.isBuiltin) {
          bufferOffset = fieldsUtil.serializePrimitive(arrayType, value, buffer, bufferOffset);
        } else if (fieldsUtil.isMessage(arrayType)) {
          bufferOffset = serializeInnerMessage(spec.getMsgSpecForType(arrayType), value, buffer, bufferOffset);
        }
      });
    } else if (field.isBuiltin) {
      bufferOffset = fieldsUtil.serializePrimitive(field.type, fieldValue, buffer, bufferOffset);
    } else {
      // is message
      bufferOffset = serializeInnerMessage(spec.getMsgSpecForType(field.baseType), fieldValue, buffer, bufferOffset);
    }
  });

  return bufferOffset;
}

// ---------------------------------------------------------
// Deserialize

function deserializeInnerMessage(spec, message, buffer, bufferOffset) {
  spec.fields.forEach(function (field) {
    var fieldValue = message[field.name];

    if (field.isArray) {
      var array = [];
      var arrayType = field.baseType;

      var arraySize;
      if (field.arrayLen !== null) {
        arraySize = field.arrayLen;
      } else {
        arraySize = buffer.readUInt32LE(bufferOffset[0]);
        bufferOffset[0] += 4; // only for variable length arrays
      }

      let ArrayMsgClass;
      let isPrimitive = field.isBuiltin;
      if (!isPrimitive) {
        ArrayMsgClass = getMessageFromRegistry(arrayType, 'msg');
      }

      for (var i = 0; i < arraySize; i++) {
        if (isPrimitive) {
          var value = fieldsUtil.deserializePrimitive(arrayType, buffer, bufferOffset);
          array.push(value);
        } else {
          // is message
          var arrayMessage = new ArrayMsgClass();

          arrayMessage = deserializeInnerMessage(spec.getMsgSpecForType(arrayType), arrayMessage, buffer, bufferOffset);

          array.push(arrayMessage);
        }
      }
      fieldValue = array;
    } else if (field.isBuiltin) {
      fieldValue = fieldsUtil.deserializePrimitive(field.type, buffer, bufferOffset);
    } else {
      // is message
      var innerMessage = new getMessageFromRegistry(field.baseType, 'msg')();
      fieldValue = deserializeInnerMessage(spec.getMsgSpecForType(field.baseType), innerMessage, buffer, bufferOffset);
    }

    message[field.name] = fieldValue;
  });

  return message;
};

// ---------------------------------------------------------

module.exports = messages;