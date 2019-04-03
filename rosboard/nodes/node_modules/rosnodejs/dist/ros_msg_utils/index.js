'use strict';

const MessageCache = require('./lib/message_cache.js');

module.exports = {
  Serialize: require('./lib/base_serialize.js'),
  Deserialize: require('./lib/base_deserialize.js'),
  getByteLength: require('./lib/encoding_utils.js').getByteLength,
  Find: MessageCache.Find,
  CMAKE_PREFIX_PATH: MessageCache.CMAKE_PREFIX_PATH,
  CMAKE_PATHS: MessageCache.CMAKE_PATHS,
  MESSAGE_PATH: MessageCache.MESSAGE_PATH,
  packageMap: MessageCache.packageMap
};