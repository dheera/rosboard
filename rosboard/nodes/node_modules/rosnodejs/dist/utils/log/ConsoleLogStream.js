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

const bunyan = require('bunyan');

class ConsoleLogStream {
  constructor(options) {
    if (options.hasOwnProperty('formatter')) {
      this._formatter = options.formatter;
    } else {
      this._formatter = rec => {
        return rec.msg;
      };
    }
  }

  write(rec) {
    let msg;
    if (typeof rec === 'string' || rec instanceof String) {
      console.log(rec);
      return;
    } else if (typeof rec === 'object') {
      const formattedMsg = this._formatter(rec);
      if (typeof formattedMsg === 'string' || formattedMsg instanceof String) {
        msg = formattedMsg;
      } else {
        console.error('Unable to format message %j', rec);
        return;
      }

      const logLevel = rec.level;
      if (logLevel <= bunyan.INFO) {
        console.info(msg);
      } else if (logLevel <= bunyan.WARN) {
        console.warn(msg);
      } else {
        // logLevel === bunyan.ERROR || bunyan.FATAL
        console.error(msg);
      }
    }
  }
};

module.exports = ConsoleLogStream;