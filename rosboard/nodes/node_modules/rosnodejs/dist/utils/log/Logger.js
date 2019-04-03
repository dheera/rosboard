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
const crypto = require('crypto');

//------------------------------------------------------------------------

/**
 * Logger is a minimal wrapper around a bunyan logger. It adds useful methods
 * to throttle/limit logging.
 * @class Logger
 */
class Logger {
  constructor(options) {
    options = options || {};

    this._name = options.name;

    if (options.$parent) {
      this._logger = options.$parent.child(options.childOptions);
    } else {
      this._logger = bunyan.createLogger({
        name: this._name,
        level: options.level || bunyan.INFO,
        streams: options.streams
      });
    }

    this._throttledLogs = new Map();
    this._onceLogs = new Set();

    const logMethods = new Set(['trace', 'debug', 'info', 'warn', 'error', 'fatal']);
    this._createLogMethods(logMethods);
    this._createThrottleLogMethods(logMethods);
    this._createOnceLogMethods(logMethods);
  }

  getStreams() {
    return this._logger.streams;
  }

  child(childOptions) {
    // setup options
    const name = childOptions.name;
    delete childOptions.name;
    const options = {
      childOptions: childOptions,
      $parent: this._logger,
      name
    };

    // create logger
    return new Logger(options);
  }

  level() {
    let level = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;

    this._logger.level(level);
  }

  setLevel(level) {
    this._logger.level(level);
  }

  getLevel() {
    return this._logger.level();
  }

  getName() {
    return this._name;
  }

  addStream(stream) {
    this._logger.addStream(stream, this.getLevel());
  }

  clearStreams() {
    this._logger.streams = [];
  }

  /**
   * Binds to bunyan logger's method for each method (info, debug, etc)
   * @param methods {Set.<String>}
   */
  _createLogMethods(methods) {
    methods.forEach(method => {
      if (this.hasOwnProperty(method)) {
        throw new Error('Unable to create method %s', method);
      }
      this[method] = this._logger[method].bind(this._logger);
    });
  }

  /**
   * Attaches throttled logging functions to this object for each level method
   * (info, debug, etc)
   * e.g.
   *  logger.infoThrottle(1000, 'Hi');
   *  logger.debugThrottle(1000, 'Hi');
   * Logs are throttled by a String key taken from the second argument (first
   * should always be throttle time in ms). So if you're logging something with
   * variable values, using format strings should be preferred since it will
   * throttle appropriately while composition will not.
   * e.g.
   *   let i = 0;
   *   setInterval(() => {
   *     logger.infoThrottle(1000, 'Counter: %d', i); // prints once second
   *     logger.infoThrottle(1000, 'Counter: ' + i);  // prints twice a second
   *     ++i;
   *   }, 500);
   *
   * @param methods {Set.<String>}
   */
  _createThrottleLogMethods(methods) {
    methods.forEach(method => {
      let throttleMethod = method + 'Throttle';
      if (this.hasOwnProperty(throttleMethod)) {
        throw new Error('Unable to create method %s', throttleMethod);
      }

      // there's currently a bug using arguments in a () => {} function
      this[throttleMethod] = function (throttleTimeMs, args) {
        // If the desired log level is enabled and the message
        // isn't being throttled, then log the message.
        if (this[method]() && !this._throttle.apply(this, arguments)) {
          return this[method].apply(this, Array.from(arguments).slice(1));
        }
        return false;
      }.bind(this);
    });
  }

  _createOnceLogMethods(methods) {
    methods.forEach(method => {
      let onceMethod = method + 'Once';
      if (this.hasOwnProperty(onceMethod)) {
        throw new Error('Unable to create method %s', onceMethod);
      }

      // there's currently a bug using arguments in a () => {} function
      this[onceMethod] = function (args) {
        if (this[method]() && this._once(arguments)) {
          return this[method].apply(this, arguments);
        }
        return false;
      }.bind(this);
    });
  }

  //--------------------------------------------------------------
  // Throttled loggers
  //  These will generally be slower. Performance will also degrade the more
  //  places where you throttle your logs. Keep this in mind. Make child loggers.
  //--------------------------------------------------------------

  /**
   * Handles throttling logic for each log statement. Throttles logs by attempting
   * to create a string log 'key' from the arguments.
   * @param throttleTimeMs {number}
   * @param args {Array} arguments provided to calling function
   * @return {boolean} should this log be throttled (if true, the log should not be written)
   */
  _throttle(throttleTimeMs) {
    const now = Date.now();

    for (var _len = arguments.length, args = Array(_len > 1 ? _len - 1 : 0), _key = 1; _key < _len; _key++) {
      args[_key - 1] = arguments[_key];
    }

    const throttlingMsg = this._getThrottleMsg(args);
    if (throttlingMsg === null) {
      // we couldn't get a msg to hash - fall through and log the message
      return false;
    }
    // else
    const msgHash = hashMessage(throttlingMsg);

    const throttledLog = this._throttledLogs.get(msgHash);

    if (throttledLog === undefined || now + 1 - throttledLog.getStartTime() >= throttledLog.getThrottleTime()) {
      const newThrottledLog = new ThrottledLog(now, throttleTimeMs);
      this._throttledLogs.set(msgHash, newThrottledLog);
      return false;
    }
    return true;
  }

  //--------------------------------------------------------------
  // Throttled loggers
  //  These will generally be slower. Performance will also degrade the more
  //  places where you throttle your logs. Keep this in mind. Make child loggers.
  //--------------------------------------------------------------

  /**
   * Handles once logic for each log statement. Throttles logs by attempting
   * to create a string log 'key' from the arguments.
   * @param args {Array} arguments provided to calling function
   * @return {boolean} should this be written
   */
  _once(args) {
    const throttleMsg = this._getThrottleMsg(args);
    if (throttleMsg === null) {
      // we couldn't get a msg to hash - fall through and log the message
      return true;
    }

    const logKey = hashMessage(throttleMsg);

    if (!this._onceLogs.has(logKey)) {
      this._onceLogs.add(logKey);
      return true;
    }
    return false;
  }

  _getThrottleMsg(args) {
    const firstArg = args[0];
    if (typeof firstArg === 'string' || firstArg instanceof String) {
      return firstArg;
    } else if (typeof firstArg === 'object') {
      // bunyan supports passing an object as the first argument with
      // optional fields to add to the log record - the second argument
      // is the actual string 'log message' in this case, so just return that
      return args[1];
    }
    // fall through *womp womp*
    return null;
  }

  /**
   * Remove old throttled logs (logs that were throttled whose throttling time has passed) from the throttling map
   * @returns {Number} number of logs that were cleaned out
   */
  clearExpiredThrottledLogs() {
    const logsToRemove = [];
    const now = Date.now();
    this._throttledLogs.forEach((log, key) => {
      if (now - log.getStartTime() >= log.getThrottleTime()) {
        logsToRemove.push(key);
      }
    });

    logsToRemove.forEach(logKey => {
      this._throttledLogs.delete(logKey);
    });

    return logsToRemove.length;
  }

  getThrottledLogSize() {
    return this._throttledLogs.size;
  }
}

//-----------------------------------------------------------------------

/**
 * @class ThrottledLog
 * Small utility class implementation for ThrottledLogger
 */
class ThrottledLog {
  constructor(timeThrottleStarted, throttlingTime) {
    this.logThrottleStartTime = timeThrottleStarted;
    this.logthrottleTimeMs = throttlingTime;
  }

  getStartTime() {
    return this.logThrottleStartTime;
  }

  getThrottleTime() {
    return this.logthrottleTimeMs;
  }
}

// Utility function to help hash messages when we throttle them.
function hashMessage(msg) {
  const sha1 = crypto.createHash('sha1');
  sha1.update(msg);
  return sha1.digest('hex');
}

//-----------------------------------------------------------------------

module.exports = Logger;