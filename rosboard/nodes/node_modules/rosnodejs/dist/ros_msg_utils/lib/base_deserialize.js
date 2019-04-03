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

/*-----------------------------------------------------------------------------
 * Primitive Deserialization Functions
 *
 * Each primitive type deserialization function has an identical signature
 *
 * @param buffer {Buffer} buffer to deserialize value from
 * @param bufferOffset {Array.Number} array of length 1 with the current bufferOffset
 *        stored at index 0. This value should be incremented according to the size of
 *        the value pulled from the buffer.
 * @returns {*} the specified value from the buffer
 * DeserializeFunc(buffer, bufferOffset)
 *-----------------------------------------------------------------------------*/

function StringDeserializer(buffer, bufferOffset) {
  const len = UInt32Deserializer(buffer, bufferOffset);
  const str = buffer.slice(bufferOffset[0], bufferOffset[0] + len).toString('utf8');
  bufferOffset[0] += len;
  return str;
}

function UInt8Deserializer(buffer, bufferOffset) {
  const val = buffer.readUInt8(bufferOffset[0], true);
  bufferOffset[0] += 1;
  return val;
}

function UInt16Deserializer(buffer, bufferOffset) {
  let val = buffer.readUInt16LE(bufferOffset[0], true);
  bufferOffset[0] += 2;
  return val;
}

function UInt32Deserializer(buffer, bufferOffset) {
  let val = buffer.readUInt32LE(bufferOffset[0], true);
  bufferOffset[0] += 4;
  return val;
}

function UInt64Deserializer(buffer, bufferOffset) {
  let slice = buffer.slice(bufferOffset[0], bufferOffset[0] + 8);
  let val = new BN(slice, 'le');
  bufferOffset[0] += 8;
  return val;
}

function Int8Deserializer(buffer, bufferOffset) {
  let val = buffer.readInt8(bufferOffset[0], true);
  bufferOffset[0] += 1;
  return val;
}

function Int16Deserializer(buffer, bufferOffset) {
  let val = buffer.readInt16LE(bufferOffset[0], true);
  bufferOffset[0] += 2;
  return val;
}

function Int32Deserializer(buffer, bufferOffset) {
  let val = buffer.readInt32LE(bufferOffset[0], true);
  bufferOffset[0] += 4;
  return val;
}

function Int64Deserializer(buffer, bufferOffset) {
  let slice = buffer.slice(bufferOffset[0], bufferOffset[0] + 8);
  let val = new BN(slice, 'le').fromTwos(64);
  bufferOffset[0] += 8;
  return val;
}

function Float32Deserializer(buffer, bufferOffset) {
  let val = buffer.readFloatLE(bufferOffset[0], true);
  bufferOffset[0] += 4;
  return val;
}

function Float64Deserializer(buffer, bufferOffset) {
  let val = buffer.readDoubleLE(bufferOffset[0], true);
  bufferOffset[0] += 8;
  return val;
}

function TimeDeserializer(buffer, bufferOffset) {
  const secs = Int32Deserializer(buffer, bufferOffset);
  const nsecs = Int32Deserializer(buffer, bufferOffset);
  return { secs: secs, nsecs: nsecs };
}

function BoolDeserializer(buffer, bufferOffset) {
  const val = !!buffer.readInt8(bufferOffset[0], true);
  bufferOffset[0] += 1;
  return val;
}

/*-----------------------------------------------------------------------------
 * Primitive Array Deserialization Functions
 *
 * Each primitive type array deserialization function has an identical signature
 *
 * @param buffer {Buffer} buffer to deserialize value from
 * @param bufferOffset {Array.Number} array of length 1 with the current bufferOffset
 *        stored at index 0. This value should be incremented according to the size of
 *        the value pulled from the buffer.
 * @param [arrayLen] {Number|null}
 *        a negative number or null means to deserialize a variable length array from the buffer
 *        a positive number means to deserialize a constant length array from the buffer
 * @returns {*} the specified value from the buffer
 * DeserializeArrayFunc(buffer, bufferOffset, arrayLen)
 *-----------------------------------------------------------------------------*/

/**
 * @function getArrayLen helper function to deserialize the length of an array
 *      when necessary. Array length is just encoded as a uint32.
 * @returns {Number}
 */
const getArrayLen = UInt32Deserializer;

/**
 * Template for most primitive array deserializers which are bound to this function and provide
 * the deserializeFunc param
 * @param deserializeFunc {function} function to deserialize a single instance of the type. Typically hidden
 *   from users by binding.
 * @param buffer {Buffer} buffer to deserialize data from
 * @param bufferOffset {Array.number}
 * @param arrayLen {null|number}
 * @returns {Array}
 * @constructor
 */
function DefaultArrayDeserializer(deserializeFunc, buffer, bufferOffset) {
  let arrayLen = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : null;

  // interpret a negative array len as a variable length array
  // so we need to parse its length ourselves
  if (arrayLen === null || arrayLen < 0) {
    arrayLen = getArrayLen(buffer, bufferOffset);
  }
  const array = new Array(arrayLen);
  for (let i = 0; i < arrayLen; ++i) {
    array[i] = deserializeFunc(buffer, bufferOffset, null);
  }
  return array;
}

/**
 * Specialized array deserialization for UInt8 Arrays
 * We return the raw buffer when deserializing uint8 arrays because it's much faster
  */
function UInt8ArrayDeserializer(buffer, bufferOffset) {
  let arrayLen = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : null;

  if (arrayLen === null || arrayLen < 0) {
    arrayLen = getArrayLen(buffer, bufferOffset);
  }

  const array = buffer.slice(bufferOffset[0], bufferOffset[0] + arrayLen);
  bufferOffset[0] += arrayLen;
  return array;
}

//-----------------------------------------------------------------------------

const PrimitiveDeserializers = {
  string: StringDeserializer,
  float32: Float32Deserializer,
  float64: Float64Deserializer,
  bool: BoolDeserializer,
  int8: Int8Deserializer,
  int16: Int16Deserializer,
  int32: Int32Deserializer,
  int64: Int64Deserializer,
  uint8: UInt8Deserializer,
  uint16: UInt16Deserializer,
  uint32: UInt32Deserializer,
  uint64: UInt64Deserializer,
  char: UInt8Deserializer,
  byte: Int8Deserializer,
  time: TimeDeserializer,
  duration: TimeDeserializer
};

const ArrayDeserializers = {
  string: DefaultArrayDeserializer.bind(null, StringDeserializer),
  float32: DefaultArrayDeserializer.bind(null, Float32Deserializer),
  float64: DefaultArrayDeserializer.bind(null, Float64Deserializer),
  bool: DefaultArrayDeserializer.bind(null, BoolDeserializer),
  int8: DefaultArrayDeserializer.bind(null, Int8Deserializer),
  int16: DefaultArrayDeserializer.bind(null, Int16Deserializer),
  int32: DefaultArrayDeserializer.bind(null, Int32Deserializer),
  int64: DefaultArrayDeserializer.bind(null, Int64Deserializer),
  uint8: UInt8ArrayDeserializer,
  uint16: DefaultArrayDeserializer.bind(null, UInt16Deserializer),
  uint32: DefaultArrayDeserializer.bind(null, UInt32Deserializer),
  uint64: DefaultArrayDeserializer.bind(null, UInt64Deserializer),
  char: UInt8ArrayDeserializer,
  byte: DefaultArrayDeserializer.bind(null, Int8Deserializer),
  time: DefaultArrayDeserializer.bind(null, TimeDeserializer),
  duration: DefaultArrayDeserializer.bind(null, TimeDeserializer)
};

//-----------------------------------------------------------------------------

module.exports = Object.assign({}, PrimitiveDeserializers, { Array: ArrayDeserializers });