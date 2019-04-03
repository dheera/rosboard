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

var _slicedToArray = function () { function sliceIterator(arr, i) { var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"]) _i["return"](); } finally { if (_d) throw _e; } } return _arr; } return function (arr, i) { if (Array.isArray(arr)) { return arr; } else if (Symbol.iterator in Object(arr)) { return sliceIterator(arr, i); } else { throw new TypeError("Invalid attempt to destructure non-iterable instance"); } }; }();

const fs = require('fs');
const path = require('path');
const utils = require('util');
const loggingManager = require('../lib/Logging.js');
const messages = require('./messageGeneration/messages.js');
const ros_msg_utils = require('../ros_msg_utils');

// *grumble grumble* this is unfortunate
// Our ros messages are going to be loaded from all over the place
// They all need access to ros_msg_utils but we can't guarantee that
// they'll be able to find ros_msg_utils without forcing people to
// add ros_msg_utils to their node_path or installing it globally
// or installing it separately for every message package
global._ros_msg_utils = ros_msg_utils;

let messagePackageMap = {};

//-----------------------------------------------------------------------
// Utilities for loading, finding handlers for
// message serialization/deserialization
//
//  When rosnodejs starts, it searches through your cmakepath for generated
//  javascript messages. It caches paths for any of the packages it finds.
//  Then, in rosnodejs when you ask to use a message package we check for it
//  in the cache and require it if found.
//-----------------------------------------------------------------------

function createDirectory(directory) {
  let curPath = '/';
  const paths = directory.split(path.sep);
  paths.forEach((part, index) => {
    if (!part) {
      return;
    }
    curPath = path.join(curPath, part);
    const thisPath = curPath;

    try {
      fs.mkdirSync(thisPath);
    } catch (err) {
      if (err.code !== 'EEXIST') {
        throw err;
      }
    }
  });
}

function copyFile(from, to, replaceCallback) {
  return new Promise((resolve, reject) => {
    const readStream = fs.createReadStream(from);
    let fileData = '';
    readStream.on('data', data => {
      fileData += data;
    });

    readStream.on('end', () => {
      if (typeof replaceCallback === 'function') {
        fileData = replaceCallback(fileData);
      }

      // open the output file for writing
      const writeStream = fs.createWriteStream(to);
      writeStream.on('open', () => {
        writeStream.write(fileData);
        writeStream.end();
        resolve();
      });
    });
  });
}

let MessageUtils = {
  getTopLevelMessageDirectory() {
    return path.join(ros_msg_utils.CMAKE_PATHS[0], ros_msg_utils.MESSAGE_PATH);
  },

  flatten(outputDir) {
    const finderDeclRegex = /^let _finder = require\('\.\.\/\.\.\/\.\.\/find\.js'\);/m;
    const finderCallRegex = /^let (\w+) = _finder\(\'\1\'\);/gm;

    const flatten_local = (packageName, startPath, localPath, outputDir) => {
      const flattenPath = path.join(startPath, localPath);
      fs.readdir(flattenPath, (err, files) => {
        if (err) {
          // if the path doesn't exist, it just means the package currently
          // being flattened doesn't have either msgs or srvs
          if (err.code !== 'ENOENT') {
            throw new Error('Error while flattening generated messages ' + err);
          }
          return;
        }
        // else
        const outputPath = path.join(outputDir, packageName, localPath);
        createDirectory(outputPath);

        files.forEach(file => {
          const filePath = path.join(flattenPath, file);
          const outputFilePath = path.join(outputDir, packageName, localPath, file);
          let callback;
          if (file !== '_index.js') {
            callback = fileData => {
              fileData = fileData.replace(finderDeclRegex, '');
              let matchData;
              while ((matchData = finderCallRegex.exec(fileData)) !== null) {
                const matchStr = matchData[0];
                const msgPackage = matchData[1];
                const replaceStr = utils.format('let %s = require(\'../../%s/_index.js\');', msgPackage, msgPackage);
                fileData = fileData.replace(matchStr, replaceStr);
              }
              return fileData;
            };
          }
          copyFile(filePath, outputFilePath, callback);
        });
      });
    };

    outputDir = path.resolve(outputDir);
    const messageDirectory = path.join(outputDir, 'ros');
    createDirectory(messageDirectory);

    // find relevant genjs base files and copy to output directory
    const filesToCopy = ['base_deserialize.js', 'base_serialize.js'];
    cmakePaths.some(cmakePath => {
      const checkPath = path.join(cmakePath, 'share', 'node_js');
      let files = fs.readdirSync(checkPath);
      if (!files) {
        return false;
      }
      files.forEach(fileName => {
        if (filesToCopy.indexOf(fileName) !== -1) {
          copyFile(path.join(checkPath, fileName), path.join(outputDir, fileName));
        }
      });
      return true;
    });

    Object.keys(messagePackagePathMap).forEach(packageName => {
      const messagePackagePath = messagePackagePathMap[packageName];
      const dir = path.dirname(messagePackagePath);

      flatten_local(packageName, dir, 'msg', messageDirectory);
      flatten_local(packageName, dir, 'srv', messageDirectory);
      // copy the index
      copyFile(messagePackagePath, path.join(messageDirectory, packageName, '_index.js'));
    });
  },

  loadMessagePackage(msgPackage) {
    messagePackageMap[msgPackage] = ros_msg_utils.Find(msgPackage);
  },

  getPackage(msgPackage) {
    return messagePackageMap[msgPackage];
  },

  requireMsgPackage(msgPackage) {
    // check our registry of on-demand generate message definition
    var fromRegistry = messages.getPackageFromRegistry(msgPackage);
    if (fromRegistry) {
      return fromRegistry;
    }

    // if we can't find it in registry, check for gennodejs
    // pre-compiled versions
    let pack = this.getPackage(msgPackage);
    if (!pack) {
      this.loadMessagePackage(msgPackage);
      return this.getPackage(msgPackage);
    }
    // else
    return pack;
  },

  getAvailableMessagePackages() {
    return ros_msg_utils.packageMap;
  },

  getHandlerForMsgType(rosDataType) {
    let loadIfMissing = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

    let type = messages.getFromRegistry(rosDataType, "msg");
    if (type) {
      return type;
    } else {
      var _rosDataType$split = rosDataType.split('/'),
          _rosDataType$split2 = _slicedToArray(_rosDataType$split, 2);

      const msgPackage = _rosDataType$split2[0],
            type = _rosDataType$split2[1];

      let messagePackage = this.getPackage(msgPackage);
      if (!messagePackage && loadIfMissing) {
        this.loadMessagePackage(msgPackage);
        messagePackage = this.getPackage(msgPackage);
      }

      if (!messagePackage) {
        throw new Error('Unable to find message package ' + msgPackage);
      }
      // else
      return messagePackage.msg[type];
    }
  },

  getHandlerForSrvType(rosDataType) {
    let loadIfMissing = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

    let srv = messages.getFromRegistry(rosDataType, "srv");
    if (srv) {
      return srv;
    } else {
      var _rosDataType$split3 = rosDataType.split('/'),
          _rosDataType$split4 = _slicedToArray(_rosDataType$split3, 2);

      const msgPackage = _rosDataType$split4[0],
            type = _rosDataType$split4[1];

      let messagePackage = this.getPackage(msgPackage);

      if (!messagePackage && loadIfMissing) {
        this.loadMessagePackage(msgPackage);
        messagePackage = this.getPackage(msgPackage);
      }

      if (!messagePackage) {
        throw new Error('Unable to find service package ' + msgPackage + '. Request: ' + !!request + ', Response: ' + !!response);
      }
      // else
      return messagePackage.srv[type];
    }
  }
};

//-----------------------------------------------------------------------

module.exports = MessageUtils;