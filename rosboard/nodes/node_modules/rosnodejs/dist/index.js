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

"use strict";

//------------------------------------------------------------------

var _slicedToArray = function () { function sliceIterator(arr, i) { var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"]) _i["return"](); } finally { if (_d) throw _e; } } return _arr; } return function (arr, i) { if (Array.isArray(arr)) { return arr; } else if (Symbol.iterator in Object(arr)) { return sliceIterator(arr, i); } else { throw new TypeError("Invalid attempt to destructure non-iterable instance"); } }; }();

const netUtils = require('./utils/network_utils.js');
const msgUtils = require('./utils/message_utils.js');
const messages = require('./utils/messageGeneration/messages.js');
const util = require('util');
const RosLogStream = require('./utils/log/RosLogStream.js');
const ConsoleLogStream = require('./utils/log/ConsoleLogStream.js');
const LogFormatter = require('./utils/log/LogFormatter.js');
const RosNode = require('./lib/RosNode.js');
const NodeHandle = require('./lib/NodeHandle.js');
const Logging = require('./lib/Logging.js');
const ActionClientInterface = require('./lib/ActionClientInterface.js');
const Time = require('./lib/Time.js');
const packages = require('./utils/messageGeneration/packages.js');

const ActionServer = require('./actions/ActionServer.js');
const ActionClient = require('./actions/ActionClient.js');
const ClientStates = require('./actions/ClientStates.js');
const SimpleActionClient = require('./actions/SimpleActionClient.js');
const SimpleActionServer = require('./actions/SimpleActionServer.js');

const MsgLoader = require('./utils/messageGeneration/MessageLoader.js');
const RemapUtils = require('./utils/remapping_utils.js');
const names = require('./lib/Names.js');
const ThisNode = require('./lib/ThisNode.js');

// will be initialized through call to initNode
let log = Logging.getLogger();
let pingMasterTimeout = null;

//------------------------------------------------------------------

let Rosnodejs = {
  /**
   * Initializes a ros node for this process. Only one ros node can exist per process.
   * If called a second time with the same nodeName, returns a handle to that node.
   * @param {string} nodeName name of the node to initialize
   * @param {object} options  overrides for this node
   * @param {boolean}   options.anonymous Set node to be anonymous
   * @param {object}    options.logging logger options for this node
   * @param {function}  options.logging.getLoggers  the function for setting which loggers
   *                                                to be used for this node
   * @param {function}  options.logging.setLoggerLevel  the function for setting the logger
   *                                                    level
   * @param {string}    options.rosMasterUri the Master URI to use for this node
   * @param {number}    options.timeout time in ms to wait for node to be initialized
   *                                    before timing out. A negative value will retry forever.
   *                                    A value of '0' will try once before stopping. @default -1
   * @return {Promise} resolved when connection to master is established
   */
  initNode(nodeName, options) {
    if (typeof nodeName !== 'string') {
      throw new Error('The node name must be a string');
    } else if (nodeName.length === 0) {
      throw new Error('The node name must not be empty!');
    }

    options = options || {};

    // process remappings from command line arguments.
    // First two are $ node <file> so we skip them
    const remappings = RemapUtils.processRemapping(process.argv.slice(2));

    // initialize netUtils from possible command line remappings
    netUtils.init(remappings);

    var _resolveNodeName2 = _resolveNodeName(nodeName, remappings, options),
        _resolveNodeName3 = _slicedToArray(_resolveNodeName2, 2);

    const resolvedName = _resolveNodeName3[0],
          namespace = _resolveNodeName3[1];


    names.init(remappings, namespace);

    if (ThisNode.node !== null) {
      if (nodeName === ThisNode.getNodeName()) {
        return Promise.resolve(this.getNodeHandle());
      }
      // else
      return Promise.reject(Error('Unable to initialize node [' + nodeName + '] - node [' + ThisNode.getNodeName() + '] already exists'));
    }

    Logging.initializeNodeLogger(resolvedName, options.logging);

    // create the ros node. Return a promise that will
    // resolve when connection to master is established
    const nodeOpts = options.node || {};
    const rosMasterUri = options.rosMasterUri || remappings['__master'] || process.env.ROS_MASTER_URI;;

    ThisNode.node = new RosNode(resolvedName, rosMasterUri, nodeOpts);

    return new Promise((resolve, reject) => {
      this._loadOnTheFlyMessages(options).then(() => {
        return _checkMasterHelper(100, options.timeout);
      }).then(Logging.initializeRosOptions.bind(Logging, this, options.logging)).then(Time._initializeRosTime.bind(Time, this, options.notime)).then(() => {
        resolve(this.getNodeHandle());
      }).catch(err => {
        log.error('Error during initialization: ' + err);
        this.shutdown();
        reject(err);
      });
    });
  },

  reset() {
    ThisNode.node = null;
  },

  shutdown() {
    clearTimeout(pingMasterTimeout);
    return ThisNode.shutdown();
  },

  ok() {
    return ThisNode.ok();
  },

  on(evt, handler) {
    if (ThisNode.node) {
      ThisNode.node.on(evt, handler);
    }
  },

  once(evt, handler) {
    if (ThisNode.node) {
      ThisNode.node.once(evt, handler);
    }
  },

  removeListener(evt, handler) {
    if (ThisNode.node) {
      ThisNode.node.removeListener(evt, handler);
    }
  },

  _loadOnTheFlyMessages(_ref) {
    let onTheFly = _ref.onTheFly;

    if (onTheFly) {
      return messages.getAll();
    }
    // else
    return Promise.resolve();
  },

  loadPackage(packageName) {
    let outputDir = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;
    let verbose = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : false;

    const msgLoader = new MsgLoader(verbose);
    if (!outputDir) {
      outputDir = msgUtils.getTopLevelMessageDirectory();
    }
    return msgLoader.buildPackage(packageName, outputDir).then(() => {
      console.log('Finished building messages!');
    }).catch(err => {
      console.error(err);
    });
  },

  loadAllPackages() {
    let outputDir = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;
    let verbose = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

    const msgLoader = new MsgLoader(verbose);
    if (!outputDir) {
      outputDir = msgUtils.getTopLevelMessageDirectory();
    }
    return msgLoader.buildPackageTree(outputDir).then(() => {
      console.log('Finished building messages!');
    });
  },

  findPackage(packageName) {
    return new Promise((resolve, reject) => {
      packages.findPackage(packageName, (err, dir) => {
        if (err) {
          reject(err);
        }
        // else
        resolve(dir);
      });
    });
  },

  require(msgPackage) {
    return msgUtils.requireMsgPackage(msgPackage);
  },

  getAvailableMessagePackages() {
    return msgUtils.getAvailableMessagePackages();
  },

  /** check that a message definition is loaded for a ros message
      type, e.g., geometry_msgs/Twist */
  checkMessage(type) {
    const parts = type.split('/');
    let rtv;
    try {
      rtv = this.require(parts[0]).msg[parts[1]];
    } catch (e) {}
    return rtv;
  },

  /** check that a service definition is loaded for a ros service
      type, e.g., turtlesim/TeleportRelative */
  checkService(type) {
    const parts = type.split('/');
    let rtv;
    try {
      rtv = this.require(parts[0]).srv[parts[1]];
    } catch (e) {}
    return rtv;
  },

  /**
   * @return {NodeHandle} for initialized node
   */
  getNodeHandle(namespace) {
    return new NodeHandle(ThisNode.node, namespace);
  },

  get nodeHandle() {
    return new NodeHandle(ThisNode.node);
  },

  get nh() {
    return new NodeHandle(ThisNode.node);
  },

  get log() {
    return Logging;
  },

  get logStreams() {
    return {
      console: ConsoleLogStream,
      ros: RosLogStream
    };
  },

  get Time() {
    return Time;
  },

  //------------------------------------------------------------------
  // ActionLib
  //------------------------------------------------------------------

  /**
    Get an action client for a given type and action server.
     **Deprecated**: Use rosNode.nh.actionClientInterface instead.
     Example:
      let ac = rosNode.nh.getActionClient(
        "/turtle_shape", "turtle_actionlib/ShapeAction");
      let shapeActionGoal =
        rosnodejs.require('turtle_actionlib').msg.ShapeActionGoal;
      ac.sendGoal(new shapeActionGoal({ goal: { edges: 3,  radius: 1 } }));
   */
  getActionClient(options) {
    return this.nh.actionClientInterface(options.actionServer, options.type, options);
  }
};

Rosnodejs.ActionServer = ActionServer;
Rosnodejs.ActionClient = ActionClient;
Rosnodejs.SimpleActionServer = SimpleActionServer;
Rosnodejs.SimpleActionClient = SimpleActionClient;
Rosnodejs.SimpleClientGoalState = ClientStates.SimpleClientGoalState;

module.exports = Rosnodejs;

//------------------------------------------------------------------
// Local Helper Functions
//------------------------------------------------------------------

/**
 * @private
 * Helper function to see if the master is available and able to accept
 * connections.
 * @param {number} timeout time in ms between connection attempts
 * @param {number} maxTimeout maximum time in ms to retry before timing out.
 * A negative number will make it retry forever. 0 will only make one attempt
 * before timing out.
 */
function _checkMasterHelper() {
  let timeout = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 100;
  let maxTimeout = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : -1;

  const startTime = Date.now();

  const localHelper = (resolve, reject) => {
    pingMasterTimeout = setTimeout(() => {
      // also check that the slave api server is set up
      if (!ThisNode.node.slaveApiSetupComplete()) {
        if (Date.now() - startTime >= maxTimeout && maxTimeout >= 0) {
          log.error(`Unable to register with master node [${ThisNode.node.getRosMasterUri()}]: unable to set up slave API Server. Stopping...`);
          reject(new Error('Unable to setup slave API server.'));
          return;
        }
        localHelper(resolve, reject);
        return;
      }
      ThisNode.node.getMasterUri({ maxAttempts: 1 }).then(() => {
        log.infoOnce(`Connected to master at ${ThisNode.node.getRosMasterUri()}!`);
        pingMasterTimeout = null;
        resolve();
      }).catch((err, resp) => {
        if (Date.now() - startTime >= maxTimeout && !(maxTimeout < 0)) {
          log.error(`Timed out before registering with master node [${ThisNode.node.getRosMasterUri()}]: master may not be running yet.`);
          reject(new Error('Registration with master timed out.'));
          return;
        } else {
          log.warnThrottle(60000, `Unable to register with master node [${ThisNode.node.getRosMasterUri()}]: master may not be running yet. Will keep trying.`);
          localHelper(resolve, reject);
        }
      });
    }, timeout);
  };

  return new Promise((resolve, reject) => {
    localHelper(resolve, reject);
  });
}

function _resolveNodeName(nodeName, remappings, options) {
  let namespace = remappings['__ns'] || process.env.ROS_NAMESPACE || '';
  namespace = names.clean(namespace);
  if (namespace.length === 0 || !namespace.startsWith('/')) {
    namespace = `/${namespace}`;
  }

  names.validate(namespace, true);

  nodeName = remappings['__name'] || nodeName;
  nodeName = names.resolve(namespace, nodeName);

  // only anonymize node name if they didn't remap from the command line
  if (options.anonymous && !remappings['__name']) {
    nodeName = _anonymizeNodeName(nodeName);
  }

  return [nodeName, namespace];
}

/**
 * Appends a random string of numeric characters to the end
 * of the node name. Follows rospy logic.
 * @param nodeName {string} string to anonymize
 * @return {string} anonymized nodeName
 */
function _anonymizeNodeName(nodeName) {
  return util.format('%s_%s_%s', nodeName, process.pid, Date.now());
}