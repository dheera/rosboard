'use strict';

var fields = exports;

const ros_msg_utils = require('../../ros_msg_utils');
const BN = require('bn.js');

/* map of all primitive types and their default values */
var map = {
  'char': 0,
  'byte': 0,
  'bool': false,
  'int8': 0,
  'uint8': 0,
  'int16': 0,
  'uint16': 0,
  'int32': 0,
  'uint32': 0,
  'int64': 0,
  'uint64': 0,
  'float32': 0,
  'float64': 0,
  'string': '',
  'time': { secs: 0, nsecs: 0 },
  'duration': { secs: 0, nsecs: 0 }
};

fields.primitiveTypes = Object.keys(map);

fields.getDefaultValue = function (type) {
  let match = type.match(/(.*)\[(\d*)\]/);
  if (match) {
    // it's an array
    const basetype = match[1];
    const length = match[2].length > 0 ? parseInt(match[2]) : 0;
    return new Array(length).fill(fields.getDefaultValue(basetype));
  } else {
    return map[type];
  }
};

fields.isString = function (type) {
  return type === 'string';
};

fields.isTime = function (type) {
  return type === 'time' || type === 'duration';
};

fields.isBool = function (type) {
  return type === 'bool';
};

fields.isFloat = function (type) {
  return type === 'float32' || type === 'float64';
};

fields.isInteger = function (type) {
  return ['byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'].indexOf('type') >= 0;
};

fields.isPrimitive = function (fieldType) {
  return fields.primitiveTypes.indexOf(fieldType) >= 0;
};

var isArrayRegex = /\[(\d*)\]$/;
fields.isArray = function (fieldType, details) {
  var match = fieldType.match(isArrayRegex);
  if (match) {
    if (match[1] && details) {
      details.length = match[1];
    }
    return true;
  } else {
    return false;
  }
};

fields.isMessage = function (fieldType) {
  return !this.isPrimitive(fieldType) && !this.isArray(fieldType);
};

fields.getTypeOfArray = function (arrayType) {
  return this.isArray(arrayType) ? arrayType.split('[')[0] : false;
};

fields.getLengthOfArray = function (arrayType) {
  var match = arrayType.match(/.*\[(\d*)\]$/);
  if (match[1] === '') {
    return null;
  }
  return parseInt(match[1]);
};

function parseType(msgType, field) {
  if (!msgType) {
    throw new Error(`Invalid empty type ${JSON.stringify(field)}`);
  }
  // else
  if (fields.isArray(msgType)) {
    field.isArray = true;
    var constantLength = msgType.endsWith('[]');
    var splits = msgType.split('[');
    if (splits.length > 2) {
      throw new Error(`Only support 1-dimensional array types: ${msgType}`);
    }
    field.baseType = splits[0];
    if (constantLength) {
      field.arrayLen = fields.getLengthOfArray(msgType);
    } else {
      field.arrayLen = null;
    }
  } else {
    field.baseType = msgType;
    field.isArray = false;
    field.arrayLen = null;
  }
}

function isHeader(type) {
  return ['Header', 'std_msgs/Header', 'roslib/Header'].indexOf(type) >= 0;
}

fields.Constant = function (name, type, val, valText) {
  this.name = name;
  this.type = type;
  this.val = val;
  this.valText = valText;
};

fields.Constant.prototype.equals = function (other) {
  return other instanceof fields.Constant && other.name === this.name && other.type === this.type && other.val === this.val;
};

fields.Field = function (name, type) {
  this.name = name;
  this.type = type;
  parseType(type, this);
  this.isHeader = isHeader(type);
  this.isBuiltin = fields.isPrimitive(this.baseType);
};

fields.Field.isHeader = function (type) {
  return isHeader(type);
};

fields.Field.isBuiltin = function (type) {
  return fields.isPrimitive(type);
};

fields.parsePrimitive = function (fieldType, fieldValue) {
  var parsedValue = fieldValue;

  if (fieldType === 'bool') {
    parsedValue = fieldValue === '1';
  } else if (fieldType === 'int8' || fieldType === 'byte') {
    parsedValue = parseInt(fieldValue);
  } else if (fieldType === 'uint8' || fieldType === 'char') {
    parsedValue = parseInt(fieldValue);
    parsedValue = Math.abs(parsedValue);
  } else if (fieldType === 'int16') {
    parsedValue = parseInt(fieldValue);
  } else if (fieldType === 'uint16') {
    parsedValue = parseInt(fieldValue);
    parsedValue = Math.abs(parsedValue);
  } else if (fieldType === 'int32') {
    parsedValue = parseInt(fieldValue);
  } else if (fieldType === 'uint32') {
    parsedValue = parseInt(fieldValue);
    parsedValue = Math.abs(parsedValue);
  } else if (fieldType === 'int64') {
    parsedValue = new BN(fieldValue);
  } else if (fieldType === 'uint64') {
    parsedValue = new BN(fieldValue);
  } else if (fieldType === 'float32') {
    parsedValue = parseFloat(fieldValue);
  } else if (fieldType === 'float64') {
    parsedValue = parseFloat(fieldValue);
  } else if (fieldType === 'time') {
    var now;
    if (fieldValue.secs && fieldValue.nsecs) {
      parsedValue.secs = fieldValue.secs;
      parsedValue.nsecs = fieldValue.nsecs;
    } else {
      if (fieldValue instanceof Date) {
        now = fieldValue.getTime();
      } else if (typeof fieldValue == "number") {
        now = fieldValue;
      } else {
        now = Date.now();
      }
      var secs = parseInt(now / 1000);
      var nsecs = now % 1000 * 1000;

      parsedValue.secs = secs;
      parsedValue.nsecs = nsecs;
    }
  }

  return parsedValue;
};

fields.serializePrimitive = function (fieldType, fieldValue, buffer, bufferOffset) {

  const serializer = ros_msg_utils.Serialize[fieldType];
  if (!serializer) {
    throw new Error(`Unable to get primitive serializer for field type ${fieldType}`);
  }
  // else

  return serializer(fieldValue, buffer, bufferOffset);
};

fields.deserializePrimitive = function (fieldType, buffer, bufferOffset) {
  const deserializer = ros_msg_utils.Deserialize[fieldType];
  if (!deserializer) {
    throw new Error(`Unable to get primitive deserializer for field type ${fieldType}`);
  }
  // else

  return deserializer(buffer, bufferOffset);
};

fields.getPrimitiveSize = function (fieldType, fieldValue) {
  var fieldSize = 0;

  if (fieldType === 'char') {
    fieldSize = 1;
  } else if (fieldType === 'byte') {
    fieldSize = 1;
  } else if (fieldType === 'bool') {
    fieldSize = 1;
  } else if (fieldType === 'int8') {
    fieldSize = 1;
  } else if (fieldType === 'uint8') {
    fieldSize = 1;
  } else if (fieldType === 'int16') {
    fieldSize = 2;
  } else if (fieldType === 'uint16') {
    fieldSize = 2;
  } else if (fieldType === 'int32') {
    fieldSize = 4;
  } else if (fieldType === 'uint32') {
    fieldSize = 4;
  } else if (fieldType === 'int64') {
    fieldSize = 8;
  } else if (fieldType === 'uint64') {
    fieldSize = 8;
  } else if (fieldType === 'float32') {
    fieldSize = 4;
  } else if (fieldType === 'float64') {
    fieldSize = 8;
  } else if (fieldType === 'string') {
    if (fieldValue !== undefined) {
      fieldSize = fieldValue.length + 4;
    }
  } else if (fieldType === 'time') {
    fieldSize = 8;
  } else if (fieldType === 'duration') {
    fieldSize = 8;
  }

  return fieldSize;
};

fields.getArraySize = function (field, array, msgSpec) {
  var that = this,
      arraySize = 0;

  //  if this is a variable length array it has a 4 byte length field at the beginning
  if (field.arrayLen === null) {
    arraySize = 4;
  }

  array.forEach(function (value) {
    if (field.isBuiltin) {
      arraySize += that.getPrimitiveSize(field.baseType, value);
    } else {
      arraySize += that.getMessageSize(value, msgSpec.getMsgSpecForType(field.baseType));
    }
  });

  return arraySize;
};

fields.getMessageSize = function (message, msgSpec) {
  var that = this,
      messageSize = 0,
      innerfields = msgSpec.fields;

  innerfields.forEach(function (field) {
    var fieldValue = message[field.name];
    if (field.isArray) {
      messageSize += that.getArraySize(field, fieldValue, msgSpec);
    } else if (field.isBuiltin) {
      messageSize += that.getPrimitiveSize(field.type, fieldValue);
    } else {
      // it's a message
      messageSize += that.getMessageSize(fieldValue, msgSpec.getMsgSpecForType(field.baseType));
    }
  });

  return messageSize;
};

fields.getMessageNameFromMessageType = function (messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[1] : messageType;
};

fields.getPackageNameFromMessageType = function (messageType) {
  return messageType.indexOf('/') !== -1 ? messageType.split('/')[0] : '';
};