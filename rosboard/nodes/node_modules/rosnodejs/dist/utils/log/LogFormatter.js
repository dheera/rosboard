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

const moment = require('moment');
const bunyan = require('bunyan');
const timeUtils = require('../time_utils.js');

const DEFAULT_FORMAT = '[${severity}] [${time}] (${logger}): ${message}';
const CONSOLE_FORMAT = process.env.ROSCONSOLE_JS_FORMAT || DEFAULT_FORMAT;
const CONSOLE_TOKEN_REGEX = /\${([a-z|A-Z]+)}/g;

class LogFormatter {
  constructor() {
    this._tokens = [];

    this._parseFormat();
    this._numTokens = this._tokens.length;
  }

  _parseFormat() {
    let match;
    let lastMatchIndex = 0;
    while ((match = CONSOLE_TOKEN_REGEX.exec(CONSOLE_FORMAT)) !== null) {
      const preToken = CONSOLE_FORMAT.substr(lastMatchIndex, match.index - lastMatchIndex);
      if (preToken.length > 0) {
        this._tokens.push(new DefaultToken(preToken));
      }
      this._tokens.push(this._getTokenizer(match[1]));
      lastMatchIndex = match.index + match[0].length;
    }
    const postToken = CONSOLE_FORMAT.substr(lastMatchIndex);
    if (postToken.length > 0) {
      this._tokens.push(new DefaultToken(postToken));
    }
  }

  _getTokenizer(token) {
    switch (token) {
      case 'severity':
        return new SeverityToken();
      case 'message':
        return new MessageToken();
      case 'time':
        return new TimeToken();
      case 'logger':
        return new LoggerToken();
      case 'isodate':
        return new IsoDateToken();
      default:
        return new DefaultToken(token);
    }
  }

  format(rec) {
    const fields = this._tokens.map(token => {
      return token.format(rec);
    });
    return fields.join('');
  }
}

// ----------------------------------------------------------------------------------------
// Tokens used for log formatting

class DefaultToken {
  constructor(val) {
    this.val = val;
  }

  format() {
    return this.val;
  }
}

class SeverityToken {
  constructor() {}

  format(rec) {
    return bunyan.nameFromLevel[rec.level].toUpperCase();
  }
}

class MessageToken {
  constructor() {}

  format(rec) {
    return rec.msg;
  }
}

class TimeToken {
  constructor() {}

  format(rec) {
    const recTime = rec.time;
    return `${(recTime / 1000).toFixed(3)}`;
  }
}

class LoggerToken {
  constructor() {}

  format(rec) {
    return rec.scope || rec.name;
  }
}

class IsoDateToken {
  constructor() {}

  format(rec) {
    return moment(rec.time).format('YYYY-MM-DDTHH:mm:ss.SSSZZ');
  }
}

const logFormatter = new LogFormatter();
module.exports = logFormatter.format.bind(logFormatter);