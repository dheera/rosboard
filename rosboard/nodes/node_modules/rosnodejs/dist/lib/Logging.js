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
const Logger = require('../utils/log/Logger.js');
const RosLogStream = require('../utils/log/RosLogStream.js');
const ConsoleLogStream = require('../utils/log/ConsoleLogStream.js');
const LogFormatter = require('../utils/log/LogFormatter.js');

//-----------------------------------------------------------------------

const DEFAULT_LOGGER_NAME = 'ros';
const LOG_CLEANUP_INTERVAL_MS = 30000; // 30 seconds

// TODO: put this in a config file somewhere
const KNOWN_LOGS = [{
  name: `${DEFAULT_LOGGER_NAME}.superdebug`,
  level: 'fatal'
}, {
  name: `${DEFAULT_LOGGER_NAME}.rosnodejs`,
  level: 'warn'
}, {
  name: `${DEFAULT_LOGGER_NAME}.masterapi`,
  level: 'warn'
}, {
  name: `${DEFAULT_LOGGER_NAME}.params`,
  level: 'warn'
}, {
  name: `${DEFAULT_LOGGER_NAME}.spinner`,
  level: 'error'
}];

//-----------------------------------------------------------------------

class LoggingManager {
  constructor() {
    this.loggerMap = {};

    // initialize the root logger with a console stream
    const rootLoggerOptions = {
      name: DEFAULT_LOGGER_NAME,
      streams: [{
        type: 'raw',
        name: 'ConsoleLogStream',
        stream: new ConsoleLogStream({ formatter: LogFormatter }),
        level: 'info'
      }],
      level: 'info'
    };
    this.rootLogger = new Logger(rootLoggerOptions);

    this._bindNodeLoggerMethods(this.rootLogger);
    this._cleanLoggersInterval = null;

    this.nameFromLevel = bunyan.nameFromLevel;
    this.levelFromName = bunyan.levelFromName;
    this.DEFAULT_LOGGER_NAME = DEFAULT_LOGGER_NAME;

    // in case the node we're running has it's own logging system, we'll
    // allow users to pass in callbacks for getting and setting loggers
    // through the logging services (_handleGetLoggers, _handleSetLoggerLevel)
    this._externalLog = {
      getLoggers: null,
      setLoggerLevel: null
    };

    KNOWN_LOGS.forEach(log => {
      this.generateLogger(log);
    });
  }

  initializeNodeLogger(nodeName) {
    let options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};


    // setup desired streams
    if (options.hasOwnProperty('streams')) {
      options.streams.forEach(stream => {
        this.addStream(stream);
      });
    }
    // set desired log level
    if (options.hasOwnProperty('level')) {
      this.setLevel(options.level);
    }

    // automatically clear out expired throttled logs every so often unless specified otherwise
    if (!options.hasOwnProperty('overrideLoggerCleanup')) {
      this._cleanLoggersInterval = setInterval(this.clearThrottledLogs.bind(this), LOG_CLEANUP_INTERVAL_MS);
    }

    if (typeof options.getLoggers === 'function') {
      this._externalLog.getLoggers = options.getLoggers;
    }

    if (typeof options.setLoggerLevel === 'function') {
      this._externalLog.setLoggerLevel = options.setLoggerLevel;
    }
  }

  initializeRosOptions(rosnodejs) {
    let options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};

    if (options.skipRosLogging) {
      return Promise.resolve();
    }

    const nh = rosnodejs.nh;
    let rosLogStream;
    try {
      const rosgraphMsgs = rosnodejs.require('rosgraph_msgs');
      rosLogStream = new RosLogStream(nh, rosgraphMsgs.msg.Log);
      this.addStream({
        type: 'raw',
        name: 'RosLogStream',
        stream: rosLogStream
      });
    } catch (err) {
      this.rootLogger.warn('Unable to setup ros logging stream');
    }

    // try to set up logging services
    try {
      const roscpp = rosnodejs.require('roscpp');
      const getLoggerSrv = nh.getNodeName() + '/get_loggers';
      const setLoggerSrv = nh.getNodeName() + '/set_logger_level';
      nh.advertiseService(getLoggerSrv, roscpp.srv.GetLoggers, this._handleGetLoggers.bind(this));
      nh.advertiseService(setLoggerSrv, roscpp.srv.SetLoggerLevel, this._handleSetLoggerLevel.bind(this));
    } catch (err) {
      this.rootLogger.warn('Unable to setup ros logging services');
    }

    if (rosLogStream && options.waitOnRosOut !== undefined && options.waitOnRosOut) {
      this.rootLogger.debug('Waiting for /rosout connection before resolving node initialization...');
      return new Promise((resolve, reject) => {
        rosLogStream.getPub().on('connection', () => {
          this.rootLogger.debug('Got connection to /rosout !');
          resolve();
        });
      });
    }
    return Promise.resolve();
  }

  generateLogger(options) {
    if (!options.hasOwnProperty('name')) {
      throw new Error('Unable to generate logger without name');
    }
    const loggerName = options.name;

    // don't regenerate the logger if it exists
    if (this.loggerMap.hasOwnProperty(loggerName)) {
      return this.loggerMap[loggerName];
    }
    // else
    // generate a child logger from root
    let newLogger = this._createChildLogger(loggerName, this.rootLogger, options);

    // stash the logger and return it
    this.loggerMap[loggerName] = newLogger;
    return newLogger;
  }

  getLogger(loggerName, options) {
    if (!loggerName || loggerName === this.rootLogger.getName()) {
      return this.rootLogger;
    } else if (!this.hasLogger(loggerName)) {
      options = options || {};
      options.name = loggerName;
      return this.generateLogger(options);
    }
    // else
    return this.loggerMap[loggerName];
  }

  hasLogger(loggerName) {
    return this.loggerMap.hasOwnProperty(loggerName);
  }

  removeLogger(loggerName) {
    if (loggerName !== DEFAULT_LOGGER_NAME) {
      delete this.loggerMap[loggerName];
    }
  }

  getLoggers() {
    const loggerNames = Object.keys(this.loggerMap);
    loggerNames.push(this.rootLogger.getName());
    return loggerNames;
  }

  getStreams() {
    return this.rootLogger.getStreams();
  }

  getStream(streamName) {
    const streams = this.getStreams();
    for (let i = 0; i < streams.length; ++i) {
      const stream = streams[i];
      if (stream.name === streamName) {
        return stream;
      }
    }
  }

  setLevel(level) {
    this._forEachLogger(logger => logger.setLevel(level), true);
  }

  addStream(stream) {
    this._forEachLogger(logger => logger.addStream(stream), true);
  }

  clearStreams() {
    this._forEachLogger(logger => logger.clearStreams(), true);
  }

  clearThrottledLogs() {
    this._forEachLogger(logger => logger.clearExpiredThrottledLogs(), true);
  }

  stopLogCleanup() {
    clearInterval(this._cleanLoggersInterval);
  }

  _handleGetLoggers(req, resp) {
    if (this._externalLog.getLoggers !== null) {
      this._externalLog.getLoggers(req, resp);
    }

    this._forEachLogger(logger => {
      resp.loggers.push({
        name: logger.getName(),
        level: bunyan.nameFromLevel[logger.getLevel()]
      });
    }, true);

    return true;
  }

  _handleSetLoggerLevel(req, resp) {
    let handled = false;
    if (this._externalLog.setLoggerLevel !== null) {
      handled = this._externalLog.setLoggerLevel(req, resp);
    }

    if (!handled) {
      const logger = this.getLogger(req.logger);
      if (!logger) {
        return false;
      }
      // else
      logger.setLevel(req.level);
    }

    return true;
  }

  _bindNodeLoggerMethods(logger) {
    const rawMethods = ['trace', 'debug', 'info', 'warn', 'error', 'fatal'];
    let methods = [];
    rawMethods.forEach(method => methods.push(method));
    rawMethods.forEach(method => methods.push(method + 'Throttle'));
    rawMethods.forEach(method => methods.push(method + 'Once'));
    methods.forEach(method => {
      this[method] = logger[method].bind(logger);
    });
  }

  _forEachLogger(perLoggerCallback, includeRoot) {
    if (includeRoot) {
      perLoggerCallback(this.rootLogger);
    }
    Object.keys(this.loggerMap).forEach(loggerName => {
      perLoggerCallback(this.loggerMap[loggerName]);
    });
  }

  _createChildLogger(childLoggerName, parentLogger, options) {
    // setup options
    options = options || {};
    options.scope = childLoggerName;

    // create logger
    const childLogger = parentLogger.child(options);

    // cache in map
    this.loggerMap[childLoggerName] = childLogger;
    return childLogger;
  }
}

module.exports = new LoggingManager();