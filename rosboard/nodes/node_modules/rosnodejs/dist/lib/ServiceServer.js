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

const net = require('net');
const NetworkUtils = require('../utils/network_utils.js');
const ros_msg_utils = require('../ros_msg_utils');
const base_serializers = ros_msg_utils.Serialize;
const SerializationUtils = require('../utils/serialization_utils.js');
const DeserializeStream = SerializationUtils.DeserializeStream;
const Deserialize = SerializationUtils.Deserialize;
const Serialize = SerializationUtils.Serialize;
const TcprosUtils = require('../utils/tcpros_utils.js');
const EventEmitter = require('events');
const Logging = require('./Logging.js');

var _require = require('../utils/ClientStates.js');

const REGISTERING = _require.REGISTERING,
      REGISTERED = _require.REGISTERED,
      SHUTDOWN = _require.SHUTDOWN;


class ServiceServer extends EventEmitter {
  constructor(options, callback, nodeHandle) {
    super();
    this._service = options.service;

    this._type = options.type;

    this._port = null;

    this._nodeHandle = nodeHandle;

    this._log = Logging.getLogger('ros.rosnodejs');

    this._requestCallback = callback;

    if (!options.typeClass) {
      throw new Error(`Unable to load service for service ${this.getService()} with type ${this.getType()}`);
    }
    this._messageHandler = options.typeClass;

    this._clients = {};

    this._state = REGISTERING;

    this._register();
  }

  getService() {
    return this._service;
  }

  getType() {
    return this._type;
  }

  getPersist() {
    return this._persist;
  }

  isCallInProgress() {
    return this._calling;
  }

  getServiceUri() {
    return NetworkUtils.formatServiceUri(this._port);
  }

  getClientUris() {
    return Object.keys(this._clients);
  }

  /**
   * The ROS client shutdown code is a little noodly. Users can close a client through
   * the ROS node or the client itself and both are correct. Either through a node.unadvertise()
   * call or a client.shutdown() call - in both instances a call needs to be made to the ROS master
   * and the client needs to tear itself down.
   */
  shutdown() {
    this._nodeHandle.unadvertiseService(this.getService());
  }

  isShutdown() {
    return this._state === SHUTDOWN;
  }

  disconnect() {
    this._state = SHUTDOWN;

    Object.keys(this._clients).forEach(clientId => {
      const client = this._clients[clientId];

      client.$deserializeStream.removeAllListeners();

      client.end();
      client.destroy();
    });

    this._clients = {};
  }

  handleClientConnection(client, header) {
    if (this.isShutdown()) {
      return;
    }
    // else
    // TODO: verify header data
    this._log.debug('Service %s handling new client connection ', this.getService());

    const error = TcprosUtils.validateServiceClientHeader(header, this.getService(), this._messageHandler.md5sum());
    if (error) {
      this._log.error('Error while validating service %s connection header: %s', this.getService(), error);
      client.end(Serialize(TcprosUtils.createTcpRosError(error)));
      return;
    }

    let respHeader = TcprosUtils.createServiceServerHeader(this._nodeHandle.getNodeName(), this._messageHandler.md5sum(), this.getType());
    client.write(respHeader);

    client.$persist = header['persistent'] === '1';

    // bind to message handler
    client.$messageHandler = this._handleMessage.bind(this, client);
    client.$deserializeStream.on('message', client.$messageHandler);

    client.on('close', () => {
      delete this._clients[client.name];
      this._log.debug('Service client %s disconnected!', client.name);
    });

    this._clients[client.name] = client;
    this.emit('connection', header, client.name);
  }

  _handleMessage(client, data) {
    this._log.trace('Service  ' + this.getService() + ' got message! ' + data.toString('hex'));
    // deserialize msg
    const req = this._messageHandler.Request.deserialize(data);

    // call service callback
    let resp = new this._messageHandler.Response();
    let result = this._requestCallback(req, resp);
    Promise.resolve(result).then(success => {
      // client should already have been closed, so if we got here just cut out early
      if (this.isShutdown()) {
        return;
      }

      const serializeResponse = TcprosUtils.serializeServiceResponse(this._messageHandler.Response, resp, success);

      // send service response
      client.write(serializeResponse);

      if (!client.$persist) {
        this._log.debug('Closing non-persistent client');
        client.end();
        delete this._clients[client.name];
      }
    });
  }

  _register() {
    this._nodeHandle.registerService(this.getService()).then(resp => {
      // if we were shutdown between the starting the registration and now, bail
      if (this.isShutdown()) {
        return;
      }

      this._state = REGISTERED;
      this.emit('registered');
    });
  }
}

module.exports = ServiceServer;