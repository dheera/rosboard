/*
 *    Copyright 2016 Rethink Robotics
 *
 *    Copyright 2016 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

'use strict';

const BN = require('bn.js');
const getByteLength = require('./encoding_utils.js').getByteLength;

/*-----------------------------------------------------------------------------
 * Primitive Serialization Functions
 *
 * Each primitive type serialization function has an identical signature
 *
 * @param value {*} value to serialize as determined by function name
 * @param buffer {Buffer} buffer to serialize value into
 * @param bufferOffset {Number} offset from buffer start to store value
 * @returns {Number} new buffer offset after serializing value
 * SerializeFunc(value, buffer, bufferOffset)
 *-----------------------------------------------------------------------------*/

function StringSerializer(value, buffer, bufferOffset) {
  let len = getByteLength(value);
  bufferOffset = buffer.writeUInt32LE(len, bufferOffset);
  return bufferOffset + buffer.write(value, bufferOffset, len, 'utf8');
}

function UInt8Serializer(value, buffer, bufferOffset) {
  return buffer.writeUInt8(value, bufferOffset, true);
}

function UInt16Serializer(value, buffer, bufferOffset) {
  return buffer.writeUInt16LE(value, bufferOffset, true);
}

function UInt32Serializer(value, buffer, bufferOffset) {
  return buffer.writeUInt32LE(value, bufferOffset, true);
}

function UInt64Serializer(value, buffer, bufferOffset) {
  if (!BN.isBN(value)) {
    value = new BN(value);
  }

  const buf = value.toBuffer('le', 8);
  buffer.set(buf, bufferOffset);

  return bufferOffset + 8;
}

function Int8Serializer(value, buffer, bufferOffset) {
  return buffer.writeInt8(value, bufferOffset, true);
}

function Int16Serializer(value, buffer, bufferOffset) {
  return buffer.writeInt16LE(value, bufferOffset, true);
}

function Int32Serializer(value, buffer, bufferOffset) {
  return buffer.writeInt32LE(value, bufferOffset, true);
}

function Int64Serializer(value, buffer, bufferOffset) {
  if (!BN.isBN(value)) {
    value = new BN(value, 'le');
  }

  value = value.toTwos(64);

  const buf = value.toBuffer('le', 8);
  buffer.set(buf, bufferOffset);

  return bufferOffset + 8;
}

function Float32Serializer(value, buffer, bufferOffset) {
  return buffer.writeFloatLE(value, bufferOffset, true);
}

function Float64Serializer(value, buffer, bufferOffset) {
  return buffer.writeDoubleLE(value, bufferOffset, true);
}

function TimeSerializer(value, buffer, bufferOffset) {
  bufferOffset = buffer.writeInt32LE(value.secs, bufferOffset, true);
  return buffer.writeInt32LE(value.nsecs, bufferOffset, true);
}

function BoolSerializer(value, buffer, bufferOffset) {
  return buffer.writeInt8(value ? 1 : 0, bufferOffset, true);
}

/*-----------------------------------------------------------------------------
 * Primitive Array Serialization Functions
 *
 * Each primitive type array serialization function has an identical signature
 *
 * @param value {*} value to serialize as determined by function name
 * @param buffer {Buffer} buffer to serialize value into
 * @param bufferOffset {Number} offset from buffer start to store value
 * @param [specArrayLen] {Number|null} array length desired by message specification
 *        a negative number or null means to serialize a variable length array into the buffer
 *        a positive number means to serialize a constant length array from the buffer
 * @returns {Number} new buffer offset after serializing value
 * SerializeFunc(value, buffer, bufferOffset, specArrayLen)
 *-----------------------------------------------------------------------------*/

/**
 * Template for most primitive array serializers which are bound to this function and provide
 * the serializeFunc param
 * @param serializeFunc {function} function to serialize a single instance of the type. Typically hidden
 *   from users by binding.
 * @param array {Array} array of values of the desired type
 * @param buffer {Buffer} buffer to serialize data into
 * @param bufferOffset {Array.number}
 * @param specArrayLen {null|number}
 * @returns {Number} buffer offset
 * @constructor
 */
function DefaultArraySerializer(serializeFunc, array, buffer, bufferOffset) {
  let specArrayLen = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

  const arrLen = array.length;

  if (specArrayLen === null || specArrayLen < 0) {
    bufferOffset = buffer.writeUInt32LE(arrLen, bufferOffset, true);
  }

  for (let i = 0; i < arrLen; ++i) {
    bufferOffset = serializeFunc(array[i], buffer, bufferOffset);
  }

  return bufferOffset;
}

/**
 * Specialized array serialization for UInt8 Arrays
 */
function UInt8ArraySerializer(array, buffer, bufferOffset) {
  let specArrayLen = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : null;

  const arrLen = array.length;

  if (specArrayLen === null || specArrayLen < 0) {
    bufferOffset = buffer.writeUInt32LE(arrLen, bufferOffset, true);
  }

  buffer.set(array, bufferOffset);
  return bufferOffset + arrLen;
}

//-----------------------------------------------------------------------------

const PrimitiveSerializers = {
  string: StringSerializer,
  float32: Float32Serializer,
  float64: Float64Serializer,
  bool: BoolSerializer,
  int8: Int8Serializer,
  int16: Int16Serializer,
  int32: Int32Serializer,
  int64: Int64Serializer,
  uint8: UInt8Serializer,
  uint16: UInt16Serializer,
  uint32: UInt32Serializer,
  uint64: UInt64Serializer,
  char: UInt8Serializer,
  byte: Int8Serializer,
  time: TimeSerializer,
  duration: TimeSerializer
};

const ArraySerializers = {
  string: DefaultArraySerializer.bind(null, StringSerializer),
  float32: DefaultArraySerializer.bind(null, Float32Serializer),
  float64: DefaultArraySerializer.bind(null, Float64Serializer),
  bool: DefaultArraySerializer.bind(null, BoolSerializer),
  int8: DefaultArraySerializer.bind(null, Int8Serializer),
  int16: DefaultArraySerializer.bind(null, Int16Serializer),
  int32: DefaultArraySerializer.bind(null, Int32Serializer),
  int64: DefaultArraySerializer.bind(null, Int64Serializer),
  uint8: UInt8ArraySerializer,
  uint16: DefaultArraySerializer.bind(null, UInt16Serializer),
  uint32: DefaultArraySerializer.bind(null, UInt32Serializer),
  uint64: DefaultArraySerializer.bind(null, UInt64Serializer),
  char: UInt8ArraySerializer,
  byte: DefaultArraySerializer.bind(null, Int8Serializer),
  time: DefaultArraySerializer.bind(null, TimeSerializer),
  duration: DefaultArraySerializer.bind(null, TimeSerializer)
};

//-----------------------------------------------------------------------------

module.exports = Object.assign({}, PrimitiveSerializers, { Array: ArraySerializers });