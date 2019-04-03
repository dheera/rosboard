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

let RosNode = require('./RosNode.js');
const messageUtils = require('../utils/message_utils.js');
const names = require('./Names.js');
const ActionClientInterface = require('./ActionClientInterface.js');
const ActionServerInterface = require('./ActionServerInterface.js');

/**
 * Handle class for nodes created with rosnodejs
 * @param node {RosNode} node that handle is attached to.
 * @param namespace {string} namespace of node. @default null
 */
class NodeHandle {
  constructor(node) {
    let namespace = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;

    this._node = node;
    this._namespace = '';

    this.setNamespace(namespace);
  }

  setNamespace(namespace) {
    if (typeof namespace !== 'string') {
      namespace = '';
    }

    if (namespace.startsWith('~')) {
      namespace = names.resolve(namespace);
    }

    this._namespace = this.resolveName(namespace, true);
  }

  getNodeName() {
    return this._node.getNodeName();
  }

  isShutdown() {
    return this._node && this._node.isShutdown();
  }

  //------------------------------------------------------------------
  // Pubs, Subs, Services
  //------------------------------------------------------------------
  /**
   * Creates a ros publisher with the provided options
   * @param topic {string}
   * @param type {string|Object} string representing message type or instance
   * @param [options] {object}
   * @param [options.latching] {boolean} latch messages
   * @param [options.tpcNoDelay] {boolean} set TCP no delay option on Socket
   * @param [options.queueSize] {number} number of messages to queue when publishing
   * @param [options.throttleMs] {number} milliseconds to throttle when publishing
   * @return {Publisher}
   */
  advertise(topic, type) {
    let options = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};

    if (!topic) {
      throw new Error(`Unable to advertise unnamed topic - got ${topic}`);
    }
    if (!type) {
      throw new Error(`Unable to advertise topic ${topic} without type - got ${type}`);
    }

    try {
      options.topic = this.resolveName(topic);
      if (typeof type === 'string' || type instanceof String) {
        options.type = type;
        options.typeClass = messageUtils.getHandlerForMsgType(type, true);
      } else {
        options.typeClass = type;
        options.type = type.datatype();
      }
      return this._node.advertise(options);
    } catch (err) {
      this._node._log.error(`Exception trying to advertise topic ${topic}`);
      throw err;
    }
  }

  /**
   * Creates a ros subscriber with the provided options
   * @param topic {string}
   * @param type {string|Object} string representing message type or instance
   * @param callback {function} function to call when message is received
   * @param [options] {object}
   * @param [options.queueSize] {number} number of messages to queue when subscribing
   * @param [options.throttleMs] {number} milliseconds to throttle when subscribing
   * @return {Subscriber}
   */
  subscribe(topic, type, callback) {
    let options = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : {};

    if (!topic) {
      throw new Error(`Unable to subscribe to unnamed topic - got ${topic}`);
    }
    if (!type) {
      throw new Error(`Unable to subscribe to topic ${topic} without type - got ${type}`);
    }

    try {
      options.topic = this.resolveName(topic);
      if (typeof type === 'string' || type instanceof String) {
        options.type = type;
        options.typeClass = messageUtils.getHandlerForMsgType(type, true);
      } else {
        options.typeClass = type;
        options.type = type.datatype();
      }
      return this._node.subscribe(options, callback);
    } catch (err) {
      this._node._log.error(`Exception trying to subscribe to topic ${topic}`);
      throw err;
    }
  }

  /**
   * Creates a ros Service server with the provided options
   * @param service {string}
   * @param type {string|Object} string representing service type or instance
   * @param callback {function} function to call when this service is called
   *   e.g.
   *     (request, response) => {
   *       response.data = !request.data;
   *       return true;
   *     }
   * @return {ServiceServer}
   */
  advertiseService(service, type, callback) {
    if (!service) {
      throw new Error(`Unable to advertise unnamed service - got ${service}`);
    }
    if (!type) {
      throw new Error(`Unable to advertise service ${service} without type - got ${type}`);
    }

    try {
      let options = { service: this.resolveName(service) };
      if (typeof type === 'string' || type instanceof String) {
        options.type = type;
        options.typeClass = messageUtils.getHandlerForSrvType(type, true);
      } else {
        options.typeClass = type;
        options.type = type.datatype();
      }

      return this._node.advertiseService(options, callback);
    } catch (err) {
      this._node._log.error(`Exception trying to advertise service ${service}`);
      throw err;
    }
  }

  /**
   * Creates a ros Service client with the provided options
   * @param service {string}
   * @param type {string|Object} string representing service type or instance
   * @param options {Object} extra options to pass to service client
   * @return {ServiceClient}
   */
  serviceClient(service, type) {
    let options = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};

    if (!service) {
      throw new Error(`Unable to create unnamed service client - got ${service}`);
    }
    if (!type) {
      throw new Error(`Unable to create service client ${service} without type - got ${type}`);
    }
    options.service = this.resolveName(service);

    try {
      if (typeof type === 'string' || type instanceof String) {
        options.type = type;
        options.typeClass = messageUtils.getHandlerForSrvType(type, true);
      } else {
        options.typeClass = type;
        options.type = type.datatype();
      }
      return this._node.serviceClient(options);
    } catch (err) {
      this._node._log.error(`Exception trying to create service client ${service}`);
      throw err;
    }
  }

  /**
   * @deprecated - use actionClientInterface
   */
  actionClient() {
    return this.actionClientInterface.apply(this, arguments);
  }

  /**
   * Create an action client
   * @param  {String} actionServer name of the action server
   * (e.g., "/turtle_shape")
   * @param  {String} type action type
   * (e.g., "turtle_actionlib/Shape")
   * @return {[type]} an instance of ActionClientInterface
   */
  actionClientInterface(actionServer, type) {
    let options = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};

    if (!actionServer) {
      throw new Error(`Unable to create action client to unspecified server - [${actionServer}]`);
    } else if (!type) {
      throw new Error(`Unable to create action client ${actionServer} without type - got ${type}`);
    }

    // don't namespace action client - topics will be resolved by
    // advertising through this NodeHandle
    return new ActionClientInterface(Object.assign({}, options, {
      actionServer,
      type,
      nh: this
    }));
  }

  actionServerInterface(actionServer, type) {
    let options = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : {};

    if (!actionServer) {
      throw new Error(`Unable to create unspecified action server  [${actionServer}]`);
    } else if (!type) {
      throw new Error(`Unable to create action server ${actionServer} without type - got ${type}`);
    }

    // don't namespace action server - topics will be resolved by
    // advertising through this NodeHandle
    return new ActionServerInterface(Object.assign({}, options, {
      actionServer,
      type,
      nh: this
    }));
  }

  /**
   * Stop receiving callbacks for this topic
   * Unregisters subscriber from master
   * @param topic {string} topic to unsubscribe from
   */
  unsubscribe(topic) {
    return this._node.unsubscribe(this.resolveName(topic));
  }

  /**
   * Stops publishing on this topic
   * Unregisters publisher from master
   * @param topic {string} topic to unadvertise
   */
  unadvertise(topic) {
    return this._node.unadvertise(this.resolveName(topic));
  }

  /**
   * Unregister service from master
   * @param service {string} service to unadvertise
   */
  unadvertiseService(service) {
    return this._node.unadvertiseService(this.resolveName(service));
  }

  /**
   * Polls master for service
   * @param service {string} name of service
   * @param [timeout] {number} give up after some time
   * @return {Promise} resolved when service exists or timeout occurs. Returns true/false for service existence
   */
  waitForService(service, timeout) {
    service = this.resolveName(service);

    let _waitForService = (callback, timeout) => {
      setTimeout(() => {
        this._node.lookupService(service).then(resp => {
          callback(true);
        }).catch((err, resp) => {
          _waitForService(callback, 500);
        });
      }, timeout);
    };

    let waitPromise = new Promise((resolve, reject) => {
      _waitForService(resolve, 0);
    });

    if (typeof timeout === 'number') {
      let timeoutPromise = new Promise((resolve, reject) => {
        setTimeout(resolve.bind(null, false), timeout);
      });

      return Promise.race([waitPromise, timeoutPromise]);
    }
    // else
    return waitPromise;
  }

  getMasterUri() {
    return this._node.getMasterUri();
  }

  /**
   * @typedef {Object} TopicList
   * @property {{name: string, type: string}[]} topics Array of topics
   */

  /**
   * Get list of topics that can be subscribed to. This does not return
   * topics that have no publishers.
   *
   * @param {string} subgraph Restrict topic names to match within the
   *                          specified subgraph. Subgraph namespace is
   *                          resolved relative to this node's namespace.
   *                          Will return all names if no subgraph is given.
   * @return {Promise.<TopicList>}
   */
  getPublishedTopics() {
    let subgraph = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : "";

    return this._node.getPublishedTopics(subgraph);
  }

  /**
   * Retrieve list topic names and their types.
   *
   * @return {Promise.<TopicList>}
   */
  getTopicTypes() {
    return this._node.getTopicTypes();
  }

  /**
   * @typedef {Object} SystemState
   * @property {{...string:Array.<string>}} publishers An object with topic names as keys and
   * an array of publishers as values
   * @property {{...string:Array.<string>}} subscribers An object with topic names as keys and
   * an array of subscribers as values
   * @property {{...string:Array.<string>}} services An object with service names as keys and
   * an array of providers as values
   */

  /**
   * Retrieve list representation of system state (i.e. publishers,
   * subscribers, and services).
   *
   * @return {Promise.<SystemState>}
   */
  getSystemState() {
    return this._node.getSystemState();
  }

  //------------------------------------------------------------------
  // Param Interface
  //------------------------------------------------------------------
  deleteParam(key) {
    return this._node.deleteParam(this.resolveName(key));
  }

  setParam(key, value) {
    return this._node.setParam(this.resolveName(key), value);
  }

  getParam(key) {
    return this._node.getParam(this.resolveName(key));
  }

  hasParam(key) {
    return this._node.hasParam(this.resolveName(key));
  }

  //------------------------------------------------------------------
  // Namespacing
  //------------------------------------------------------------------

  resolveName(name) {
    let remap = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : true;
    let noValidate = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : false;

    if (!noValidate) {
      names.validate(name, true);
    }

    if (name.length === 0) {
      return this._namespace;
    }

    if (name.startsWith('~')) {
      throw new Error('Using ~ names with NodeHandle methods is not allowed');
    } else if (!name.startsWith('/') && this._namespace.length > 0) {
      name = names.append(this._namespace, name);
    } else {
      name = names.clean(name);
    }

    if (remap) {
      return this._remapName(name);
    } else {
      return names.resolve(name, false);
    }
  }

  remapName(name) {
    name = this.resolveName(name, false);

    return this._remapName(name);
  }

  _remapName(name) {
    return names.remap(name);
  }
}

//------------------------------------------------------------------

module.exports = NodeHandle;