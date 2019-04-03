'use strict';

const fs = require('fs');
const util = require('util');
const path = require('path');
const md5 = require('md5');
const fieldsUtil = require('./fields.js');
const IndentedWriter = require('./IndentedWriter.js');
const MessageWriter = require('./MessageWriter.js');

const MSG_DIVIDER = '---';

const MSG_TYPE = 'msg';
const SRV_TYPE = 'srv';
const SRV_REQUEST_TYPE = 'srvRequest';
const SRV_RESPONSE_TYPE = 'srvResponse';
const ACTION_TYPE = 'action';
const ACTION_GOAL_TYPE = 'actionGoal';
const ACTION_FEEDBACK_TYPE = 'actionFeedback';
const ACTION_RESULT_TYPE = 'actionResult';
const ACTION_ACTION_GOAL_TYPE = 'actionGoal';
const ACTION_ACTION_FEEDBACK_TYPE = 'actionActionFeedback';
const ACTION_ACTION_RESULT_TYPE = 'actionActionResult';
const ACTION_ACTION_TYPE = 'actionAction';

function getFullMessageName(packageName, messageName) {
  return packageName + '/' + messageName;
}

function getPackageNameFromMessageType(messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[0] : '';
}

const isArrayRegex = /.*\[*\]$/;
function isArray(fieldType) {
  return fieldType.match(isArrayRegex) !== null;
}

function getLengthOfArray(arrayType) {
  var match = arrayType.match(/.*\[(\d*)\]$/);
  if (match[1] === '') {
    return null;
  }
  return parseInt(match[1]);
}

function parseType(msgType) {
  if (!msgType) {
    throw new Error(`Invalid empty type ${JSON.stringify(field)}`);
  }
  // else
  const field = {};
  if (isArray(msgType)) {
    field.isArray = true;
    const variableLength = msgType.endsWith('[]');
    const splits = msgType.split('[');
    if (splits.length > 2) {
      throw new Error(`Only support 1-dimensional array types: ${msgType}`);
    }
    field.baseType = splits[0];
    if (!variableLength) {
      field.arrayLen = getLengthOfArray(msgType);
    } else {
      field.arrayLen = null;
    }
  } else {
    field.baseType = msgType;
    field.isArray = false;
    field.arrayLen = null;
  }
  return field;
}

function isHeader(type) {
  return ['Header', 'std_msgs/Header', 'roslib/Header'].indexOf(type) >= 0;
}

class Field {
  constructor(name, type) {
    this.name = name;
    this.type = type;
    Object.assign(this, parseType(type));
    this.isHeader = isHeader(type);
    this.isBuiltin = fieldsUtil.isPrimitive(this.baseType);
  }

  getPackage() {
    if (this.isBuiltin) {
      return null;
    }
    return this.baseType.split('/')[0];
  }

  getMessage() {
    if (this.isBuiltin) {
      return null;
    }
    return this.baseType.split('/')[1];
  }
}

/**
 * @class RosMsgSpec
 * Base class for message spec. Provides useful functionality on its own that is extended
 * by subclasses.
 */
class RosMsgSpec {
  /**
   * Constructor for base class
   * @param msgCache {MessageManager}
   * @param packageName {string} name of package
   * @param messageName {string} name of message
   * @param type {string} type of message (see MSG_TYPE, SRV_TYPE, ... above)
   * @param filePath {string|null} path to message file
   * @returns {RosMsgSpec}
   */
  constructor(msgCache, packageName, messageName, type) {
    let filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    this.msgCache = msgCache;
    this.messageName = messageName;
    this.packageName = packageName;
    this.type = type;
    this.fileContents = null;
  }

  /**
   * Given a type of message, returns the correct subclass of RosMsgSpec
   * @param msgCache {MessageManager}
   * @param packageName {string} name of package
   * @param messageName {string} name of message
   * @param type {string} type of message (see MSG_TYPE, SRV_TYPE, ... above)
   * @param filePath {string|null} path to message file
   * @returns {SrvSpec|MsgSpec|ActionSpec}
   */
  static create(msgCache, packageName, messageName, type) {
    let filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    switch (type) {
      case SRV_TYPE:
        return new SrvSpec(msgCache, packageName, messageName, type, filePath);
      case MSG_TYPE:
        return new MsgSpec(msgCache, packageName, messageName, type, filePath);
      case ACTION_TYPE:
        return new ActionSpec(msgCache, packageName, messageName, type, filePath);
      default:
        throw new Error(`Unable to create message spec for type [${type}]`);
    }
  }

  /**
   * Query the cache for another message spec
   * @param type {string} full type of message to search for (e.g. sensor_msgs/Image)
   * @returns {RosMsgSpec}
   */
  getMsgSpecForType(type) {
    return this.msgCache.getMessageSpec(type);
  }

  /**
   * Tries to load and parse message file
   * @param [filePath] {string} path to file - will load file from here if provided
   * @param [fileContents] {string} file contents - will parse into desired fields
   */
  loadFile() {
    let filePath = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;
    let fileContents = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;


    if (filePath !== null) {
      fileContents = this._loadMessageFile(filePath);
    }
    if (fileContents !== null) {
      this.fileContents = fileContents;

      this._parseMessage(fileContents);
    }
  }

  _parseMessage() {
    throw new Error('Unable to parse message file for base class RosMsgSpec');
  }

  /**
   * Generates the file data for this class
   */
  writeMessageClassFile() {
    throw new Error('Unable to write message class file for base class RosMsgSpec');
  }

  /**
   * Get full message name for this spec (e.g. sensor_msgs/String)
   * @returns {string}
   */
  getFullMessageName() {
    return getFullMessageName(this.packageName, this.messageName);
  }

  /**
   * Get a unique list of other packages this spec depends on
   * @param [deps] {Set} dependencies will be added to this set if provided
   * @returns {Set} list of dependencies
   */
  getMessageDependencies() {
    let deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();
    const packageName = this.packageName;

    this.fields.forEach(field => {
      const fieldPackage = getPackageNameFromMessageType(field.baseType);
      if (!field.isBuiltin && fieldPackage !== packageName) {
        if (field.isHeader) {
          deps.add('std_msgs');
        } else {
          deps.add(fieldPackage);
        }
      }
    });
    return deps;
  }

  /**
   * Reads file at specified location and returns its contents
   * @param fileName {string}
   * @returns fileContents {string}
   * @private
   */
  _loadMessageFile(fileName) {
    return fs.readFileSync(fileName, 'utf8');
  }

  /**
   * For this message spec, generates the text used to calculate the message's md5 sum
   * @returns {string}
   */
  getMd5text() {
    return '';
  }

  /**
   * Get the md5 sum of this message
   * @returns {string}
   */
  getMd5sum() {
    return md5(this.getMd5text());
  }

  /**
   * Generates a depth-first list of all dependencies of this message in field order.
   * @param [deps] {Array}
   * @returns {Array}
   */
  getFullDependencies() {
    let deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : [];

    return [];
  }

  /**
   * Computes the full text of a message/service.
   * Necessary for rosbags.
   * Mirrors gentools.
   * See compute_full_text() in
   *   https://github.com/ros/ros/blob/kinetic-devel/core/roslib/src/roslib/gentools.py
   */
  computeFullText() {
    const w = new IndentedWriter();

    const deps = this.getFullDependencies();
    const sep = '='.repeat(80);
    w.write(this.fileContents.trim()).newline();

    deps.forEach(dep => {
      w.write(sep).write(`MSG: ${dep.getFullMessageName()}`).write(dep.fileContents.trim()).newline();
    });

    return w.get().trim();
  }
}

/**
 * Subclass of RosMsgSpec
 * Implements logic for individual ros messages as well as separated parts of services and actions
 * (e.g. Request, Response, Goal, ActionResult, ...)
 * @class MsgSpec
 */
class MsgSpec extends RosMsgSpec {
  constructor(msgCache, packageName, messageName, type) {
    let filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;
    let fileContents = arguments.length > 5 && arguments[5] !== undefined ? arguments[5] : null;

    super(msgCache, packageName, messageName, type, filePath);
    this.constants = [];
    this.fields = [];

    this.loadFile(filePath, fileContents);
  }

  /**
   * Parses through message definition for fields and constants
   * @param content {string} relevant portion of message definition
   * @private
   */
  _parseMessage(content) {
    let lines = content.split('\n').map(line => line.trim());

    try {
      lines.forEach(this._parseLine.bind(this));
    } catch (err) {
      console.error('Error while parsing message %s: %s', this.getFullMessageName(), err);
      throw err;
    }
  }

  /**
   * Given a line from the message file, parse it for useful contents
   * @param line {string}
   * @private
   */
  _parseLine(line) {
    line = line.trim();

    const lineEqualIndex = line.indexOf('=');
    const lineCommentIndex = line.indexOf('#');

    // clear out comments if this line is not a constant
    // string constants include EVERYTHING after the equals
    if (lineEqualIndex === -1 && lineCommentIndex !== -1 || lineEqualIndex > lineCommentIndex) {
      line = line.replace(/#.*/, '');
    }

    if (line !== '') {

      var firstSpace = line.indexOf(' '),
          fieldType = line.substring(0, firstSpace).trim(),
          field = line.substring(firstSpace + 1).trim(),
          equalIndex = field.indexOf('='),
          fieldName = field.trim();

      if (equalIndex !== -1) {
        fieldName = field.substring(0, equalIndex).trim();
        if (fieldType !== 'string') {
          const commentIndex = field.indexOf('#');
          if (commentIndex !== -1) {
            field = field.substring(0, commentIndex).trim();
          }
        }
        const constant = field.substring(equalIndex + 1, field.length).trim();
        const parsedConstant = fieldsUtil.parsePrimitive(fieldType, constant);

        this.constants.push({
          name: fieldName,
          type: fieldType,
          value: parsedConstant,
          stringValue: constant // include the string value for md5 text
          , index: this.constants.length,
          messageType: null
        });
      } else {
        // ROS lets you not include the package name if it's in the same package
        // e.g. in tutorial_msgs/MyMsg
        //    ComplexType fieldName  # this is assumed to be in tutorial_msgs
        // TODO: would ROS automatically search for fields in other packages if possible??
        //       we may need to support this...
        var _parseType = parseType(fieldType);

        const baseType = _parseType.baseType;
        // if it's a header and isn't explicit, be explicit

        if (isHeader(baseType) && !getPackageNameFromMessageType(baseType)) {
          fieldType = 'std_msgs/' + fieldType;
        } else if (!fieldsUtil.isPrimitive(baseType) && !getPackageNameFromMessageType(baseType)) {
          fieldType = this.packageName + '/' + fieldType;
        }
        let f = new Field(fieldName, fieldType);
        this.fields.push(f);
      }
    }
  }

  /**
   * Check if this message will have a fixed size regardless of its contents
   * @returns {boolean}
   */
  isMessageFixedSize() {
    // Check if a particular message specification has a constant size in bytes
    const fields = this.fields;
    const types = fields.map(field => {
      return field.baseType;
    });
    const variableLengthArrays = fields.map(field => {
      return field.isArray && field.arrayLen === null;
    });
    const isBuiltin = fields.map(field => {
      return field.isBuiltin;
    });
    if (types.indexOf('string') !== -1) {
      return false;
    } else if (variableLengthArrays.indexOf(true) !== -1) {
      return false;
    } else if (isBuiltin.indexOf(false) === -1) {
      return true;
    } else {
      const nonBuiltins = fields.filter(field => {
        return !field.isBuiltin;
      });
      return nonBuiltins.every(field => {
        const msgSpec = this.getMsgSpecForType(field.baseType);
        if (!msgSpec) {
          throw new Error(`Unable to load spec for field [${field.baseType}]`);
        }
        return msgSpec.isMessageFixedSize();
      });
    }
  }

  /**
   * Calculates the fixed size of this message if it has a fixed size
   * @returns {number|null} size if message has fixed size else null
   */
  getMessageFixedSize() {
    // Return the size of the message.
    // If the message does not have a fixed size, returns null
    if (!this.isMessageFixedSize()) {
      return null;
    }
    // else
    let length = 0;
    this.fields.forEach(field => {
      if (field.isBuiltin) {
        const typeSize = fieldsUtil.getPrimitiveSize(field.baseType);
        if (typeSize === 0) {
          throw new Error(`Field ${field.baseType} in message ${this.getFullMessageName()} has a non-constant size`);
        }
        if (!field.isArray) {
          length += typeSize;
        } else if (field.arrayLen === null) {
          throw new Error(`Array field ${field.baseType} in message ${this.getFullMessageName()} has a variable length`);
        } else {
          length += field.arrayLen * typeSize;
        }
      } else {
        const msgSpec = this.getMsgSpecForType(field.baseType);
        if (!msgSpec) {
          throw new Error(`Unable to load spec for field [${field.baseType}] in message ${this.getFullMessageName()}`);
        }
        const fieldSize = msgSpec.getMessageFixedSize();
        if (fieldSize === null) {
          throw new Error(`Field ${field.baseType} in message ${this.getFullMessageName()} has a non-constant size`);
        }
        length += fieldSize;
      }
    });
    return length;
  }

  /**
   * Generates the text used to calculate this message's md5 sum
   * @returns {string}
   */
  getMd5text() {
    let text = '';
    var constants = this.constants.map(function (constant) {
      // NOTE: use the string value of the constant from when we parsed it so that JS doesn't drop decimal precision
      // e.g. message has constant "float32 A_CONSTANT=1.0"
      //  here would turn into "float32 A_CONSTANT=1" if we used its parsed value
      return constant.type + ' ' + constant.name + '=' + constant.stringValue;
    }).join('\n');

    var fields = this.fields.map(field => {
      if (field.isBuiltin) {
        return field.type + ' ' + field.name;
      } else {
        const spec = this.getMsgSpecForType(field.baseType);
        return spec.getMd5sum() + ' ' + field.name;
      }
    }).join('\n');

    text += constants;
    if (text.length > 0 && fields.length > 0) {
      text += "\n";
    }
    text += fields;
    return text;
  }

  /**
   * Generates text for message class file
   * @returns {string}
   */
  generateMessageClassFile() {
    return MessageWriter.createMessageClass(this);
  }

  /**
   * Generates a depth-first list of all dependencies of this message in field order.
   * @param [deps] {Array}
   * @returns {Array}
   */
  getFullDependencies() {
    let deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : [];

    this.fields.forEach(field => {
      if (!field.isBuiltin) {
        const fieldSpec = this.getMsgSpecForType(field.baseType);
        if (deps.indexOf(fieldSpec) === -1) {
          deps.push(fieldSpec);
        }
        fieldSpec.getFullDependencies(deps);
      }
    });

    return deps;
  }
}

/**
 * Subclass of RosMsgSpec
 * Implements logic for ros services. Creates MsgSpecs for request and response
 * @class SrvSpec
 */
class SrvSpec extends RosMsgSpec {
  constructor(msgCache, packageName, messageName, type) {
    let filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    super(msgCache, packageName, messageName, type, filePath);

    this.fileContents = this._loadMessageFile(filePath);

    var _extractMessageSectio = this._extractMessageSections(this.fileContents);

    const req = _extractMessageSectio.req,
          resp = _extractMessageSectio.resp;


    this.request = new MsgSpec(msgCache, packageName, messageName + 'Request', SRV_REQUEST_TYPE, null, req);
    this.response = new MsgSpec(msgCache, packageName, messageName + 'Response', SRV_RESPONSE_TYPE, null, resp);
  }

  /**
   * Takes a full service definition and pulls out the request and response sections
   * @param fileContents {string}
   * @returns {object}
   * @private
   */
  _extractMessageSections(fileContents) {
    let lines = fileContents.split('\n').map(line => line.trim());

    const sections = {
      req: '',
      resp: ''
    };

    let currentSection = 'req';

    lines.forEach(line => {
      if (line.startsWith(MSG_DIVIDER)) {
        currentSection = 'resp';
      } else {
        sections[currentSection] += `\n${line}`;
      }
    });

    return sections;
  }

  getMd5text() {
    return this.request.getMd5text() + this.response.getMd5text();
  }

  getMessageDependencies() {
    let deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();

    this.request.getMessageDependencies(deps);
    this.response.getMessageDependencies(deps);
    return deps;
  }

  /**
   * Generates text for service class file
   * @returns {string}
   */
  generateMessageClassFile() {
    return MessageWriter.createServiceClass(this);
  }
}

/**
 * Subclass of RosMsgSpec
 * Implements logic for ROS actions which generate 7 messages from their definition.
 * Creates MsgSpecs for goal, result, feedback, action goal, action result, action feedback, and action
 * @class ActionSpec
 */
class ActionSpec extends RosMsgSpec {
  constructor(msgCache, packageName, messageName, type) {
    let filePath = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    super(msgCache, packageName, messageName, type, filePath);

    this.fileContents = this._loadMessageFile(filePath);

    var _extractMessageSectio2 = this._extractMessageSections(this.fileContents);

    const goal = _extractMessageSectio2.goal,
          result = _extractMessageSectio2.result,
          feedback = _extractMessageSectio2.feedback;

    // Parse the action definition into its 3 respective parts

    this.goal = new MsgSpec(msgCache, packageName, messageName + 'Goal', ACTION_GOAL_TYPE, null, goal);
    this.result = new MsgSpec(msgCache, packageName, messageName + 'Result', ACTION_RESULT_TYPE, null, result);
    this.feedback = new MsgSpec(msgCache, packageName, messageName + 'Feedback', ACTION_FEEDBACK_TYPE, null, feedback);
    this.generateActionMessages();
  }

  /**
   * Takes a full service definition and pulls out the request and response sections
   * @param fileContents {string}
   * @returns {object}
   * @private
   */
  _extractMessageSections(fileContents) {
    let lines = fileContents.split('\n').map(line => line.trim());

    const sections = {
      goal: '',
      result: '',
      feedback: ''
    };

    let currentSection = 'goal';

    lines.forEach(line => {
      if (line.startsWith(MSG_DIVIDER)) {
        currentSection = {
          goal: 'result',
          result: 'feedback'
        }[currentSection];
      } else {
        sections[currentSection] += `\n${line}`;
      }
    });

    return sections;
  }

  /**
   * Get a list of all the message specs created by this ros action
   * @returns {MsgSpec[]}
   */
  getMessages() {
    return [this.goal, this.result, this.feedback, this.actionGoal, this.actionResult, this.actionFeedback, this.action];
  }

  /**
   * Creates the remaining 4 action messages
   */
  generateActionMessages() {
    this.generateActionGoalMessage();
    this.generateActionResultMessage();
    this.generateActionFeedbackMessage();
    this.generateActionMessage();
  }

  generateActionGoalMessage() {
    const goalMessage = MessageWriter.generateActionGoalMessage(this.getFullMessageName());

    this.actionGoal = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionGoal', ACTION_ACTION_GOAL_TYPE, null, goalMessage);
  }

  generateActionResultMessage() {
    const resultMessage = MessageWriter.generateActionResultMessage(this.getFullMessageName());

    this.actionResult = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionResult', ACTION_ACTION_RESULT_TYPE, null, resultMessage);
  }

  generateActionFeedbackMessage() {
    const feedbackMessage = MessageWriter.generateActionFeedbackMessage(this.getFullMessageName());

    this.actionFeedback = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'ActionFeedback', ACTION_ACTION_FEEDBACK_TYPE, null, feedbackMessage);
  }

  generateActionMessage() {
    const actionMessage = MessageWriter.generateActionMessage(this.getFullMessageName());

    this.action = new MsgSpec(this.msgCache, this.packageName, this.messageName + 'Action', ACTION_ACTION_TYPE, null, actionMessage);
  }

  getMessageDependencies() {
    let deps = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : new Set();

    this.goal.getMessageDependencies(deps);
    this.result.getMessageDependencies(deps);
    this.feedback.getMessageDependencies(deps);
    this.actionGoal.getMessageDependencies(deps);
    this.actionResult.getMessageDependencies(deps);
    this.actionFeedback.getMessageDependencies(deps);
    this.action.getMessageDependencies(deps);
    return deps;
  }
}

RosMsgSpec.MSG_TYPE = MSG_TYPE;
RosMsgSpec.SRV_TYPE = SRV_TYPE;
RosMsgSpec.SRV_REQUEST_TYPE = SRV_REQUEST_TYPE;
RosMsgSpec.SRV_RESPONSE_TYPE = SRV_RESPONSE_TYPE;
RosMsgSpec.ACTION_TYPE = ACTION_TYPE;
RosMsgSpec.ACTION_GOAL_TYPE = ACTION_GOAL_TYPE;
RosMsgSpec.ACTION_FEEDBACK_TYPE = ACTION_FEEDBACK_TYPE;
RosMsgSpec.ACTION_RESULT_TYPE = ACTION_RESULT_TYPE;
RosMsgSpec.ACTION_ACTION_GOAL_TYPE = ACTION_ACTION_GOAL_TYPE;
RosMsgSpec.ACTION_ACTION_FEEDBACK_TYPE = ACTION_ACTION_FEEDBACK_TYPE;
RosMsgSpec.ACTION_ACTION_RESULT_TYPE = ACTION_ACTION_RESULT_TYPE;
RosMsgSpec.ACTION_ACTION_TYPE = ACTION_ACTION_TYPE;

module.exports = RosMsgSpec;