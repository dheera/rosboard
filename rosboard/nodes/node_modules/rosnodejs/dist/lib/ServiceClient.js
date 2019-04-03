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

let net = require('net');
let NetworkUtils = require('../utils/network_utils.js');
const ros_msg_utils = require('../ros_msg_utils');
const base_serializers = ros_msg_utils.Serialize;
let SerializationUtils = require('../utils/serialization_utils.js');
let DeserializeStream = SerializationUtils.DeserializeStream;
let Deserialize = SerializationUtils.Deserialize;
let Serialize = SerializationUtils.Serialize;
let TcprosUtils = require('../utils/tcpros_utils.js');
let EventEmitter = require('events');
let Logging = require('./Logging.js');

var _require = require('../utils/ClientStates.js');

const REGISTERED = _require.REGISTERED,
      SHUTDOWN = _require.SHUTDOWN;

/**
 * @class ServiceCall
 * A small utility class for ServiceClient...
 * basically just a struct.
 */

class ServiceCall {
  constructor(request, resolve, reject) {
    this.request = request;
    this.resolve = resolve;
    this.reject = reject;

    this.serviceClient = null;
  }
}

/**
 * @class ServiceClient
 * ServiceClient provides an interface to querying a service in ROS.
 * Typically ROS service calls are blocking. This isn't an option for JS though.
 * To accommodate multiple successive service calls, calls are queued along with
 * resolve/reject handlers created for that specific call. When a call completes, the
 * next call in the queue is handled
 */
class ServiceClient extends EventEmitter {
  constructor(options, nodeHandle) {
    super();
    this._service = options.service;

    this._type = options.type;

    this._persist = !!options.persist;

    this._maxQueueLength = options.queueLength || -1;

    this._resolve = !!options.resolve;

    this._calling = false;

    this._log = Logging.getLogger('ros.rosnodejs');

    this._nodeHandle = nodeHandle;

    if (!options.typeClass) {
      throw new Error(`Unable to load service for service client ${this.getService()} with type ${this.getType()}`);
    }
    this._messageHandler = options.typeClass;

    this._serviceClient = null;

    this._callQueue = [];

    this._currentCall = null;

    // ServiceClients aren't "registered" anywhere but it's not
    // waiting to get registered either so REGISTERING doesn't make sense...
    // Hence, we'll just call it REGISTERED.
    this._state = REGISTERED;
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

  close() {
    // don't remove service client if call is in progress
    if (!this.isCallInProgress()) {
      this._serviceClient = null;
    }
  }

  shutdown() {
    this._state = SHUTDOWN;
    if (this._currentCall) {
      this._currentCall.reject('SHUTDOWN');
    }
  }

  isShutdown() {
    return this._state === SHUTDOWN;
  }

  /**
   * Call the service - if a current call is in progress, nothing will be done
   * @return {Promise}
   */
  call(request) {
    return new Promise((resolve, reject) => {
      const newCall = new ServiceCall(request, resolve, reject);
      this._callQueue.push(newCall);

      // shift off old calls if user specified a max queue length
      if (this._maxQueueLength > 0 && this._callQueue.length > this._maxQueueLength) {
        const oldCall = this._callQueue.shift();
        const err = new Error('Unable to complete service call because of queue limitations');
        err.code = 'E_ROSSERVICEQUEUEFULL';
        oldCall.reject(err);
      }

      // if there weren't any other calls in the queue and there's no current call, execute this new call
      // otherwise new call will be handled in order when others complete
      if (this._callQueue.length === 1 && this._currentCall === null) {
        this._executeCall();
      }
    });
  }

  _executeCall() {
    if (this.isShutdown()) {
      return;
    } else if (this._callQueue.length === 0) {
      this._log.warn('Tried executing service call on empty queue');
      return;
    }
    // else
    const call = this._callQueue.shift();
    this._currentCall = call;
    this._calling = true;

    this._initiateServiceConnection(call).then(() => {
      this._throwIfShutdown();

      return this._sendRequest(call);
    }).then(msg => {
      this._throwIfShutdown();

      this._calling = false;
      this._currentCall = null;

      this._scheduleNextCall();

      call.resolve(msg);
    }).catch(err => {
      if (!this.isShutdown()) {
        // this probably just means the service didn't exist yet - don't complain about it
        // We should still reject the call
        if (err.code !== 'EROSAPIERROR') {
          this._log.error(`Error during service ${this.getService()} call ${err}`);
        }

        this._calling = false;
        this._currentCall = null;

        this._scheduleNextCall();

        call.reject(err);
      }
    });
  }

  _scheduleNextCall() {
    if (this._callQueue.length > 0 && !this.isShutdown()) {
      process.nextTick(this._executeCall.bind(this));
    }
  }

  _initiateServiceConnection(call) {
    // if we haven't connected to the service yet, create the connection
    // this will always be the case unless this is persistent service client
    // calling for a second time.
    if (!this.getPersist() || this._serviceClient === null) {
      return this._nodeHandle.lookupService(this.getService()).then(resp => {
        this._throwIfShutdown();

        const serviceUri = resp[2];
        const serviceHost = NetworkUtils.getAddressAndPortFromUri(serviceUri);

        // connect to the service's tcpros server
        return this._connectToService(serviceHost, call);
      });
    } else {
      // this is a persistent service that we've already set up
      call.serviceClient = this._serviceClient;
      return Promise.resolve();
    }
  }

  _sendRequest(call) {
    if (this._resolve) {
      call.request = this._messageHandler.Request.Resolve(call.request);
    }

    // serialize request
    const serializedRequest = TcprosUtils.serializeMessage(this._messageHandler.Request, call.request);

    call.serviceClient.write(serializedRequest);

    return new Promise((resolve, reject) => {
      const closeHandler = () => {
        this._log.debug('Service %s client disconnected during call!', this.getService());
        reject(new Error('Connection was closed'));
      };

      call.serviceClient.$deserializeStream.once('message', (msg, success) => {
        call.serviceClient.removeListener('close', closeHandler);

        if (success) {
          resolve(this._messageHandler.Response.deserialize(msg));
        } else {
          const error = new Error(msg);
          error.code = 'E_ROSSERVICEFAILED';
          reject(error);
        }
      });

      // if the connection closes while waiting for a response, reject the request
      call.serviceClient.on('close', closeHandler);
    });
  }

  _connectToService(serviceHost, call) {
    return new Promise((resolve, reject) => {
      this._log.debug('Service client %s connecting to %j', this.getService(), serviceHost);

      this._createCallSocketAndHandlers(serviceHost, call, reject);

      this._cacheSocketIfPersistent(call);

      let deserializer = new DeserializeStream();
      call.serviceClient.$deserializeStream = deserializer;
      call.serviceClient.pipe(deserializer);

      deserializer.once('message', msg => {
        if (!call.serviceClient.$initialized) {
          let header = TcprosUtils.parseTcpRosHeader(msg);
          if (header.error) {
            reject(new Error(header.error));
            return;
          }

          // stream deserialization for service response is different - set that up for next message
          deserializer.setServiceRespDeserialize();
          call.serviceClient.$initialized = true;
          resolve();
        }
      });
    });
  }

  _createCallSocketAndHandlers(serviceHost, call, reject) {
    // create a socket connection to the service provider
    call.serviceClient = net.connect(serviceHost, () => {

      // Connection to service's TCPROS server succeeded - generate and send a connection header
      this._log.debug('Sending service client %s connection header', this.getService());

      let serviceClientHeader = TcprosUtils.createServiceClientHeader(this._nodeHandle.getNodeName(), this.getService(), this._messageHandler.md5sum(), this.getType(), this.getPersist());

      call.serviceClient.write(serviceClientHeader);
    });

    // bind a close handling function
    call.serviceClient.on('close', () => {
      call.serviceClient = null;
      // we could probably just always reset this._serviceClient to null here but...
      if (this.getPersist()) {
        this._serviceClient = null;
      }
    });

    // bind an error function - any errors connecting to the service
    // will cause the call to be rejected (in this._executeCall)
    call.serviceClient.on('error', err => {
      this._log.info(`Service Client ${this.getService()} error: ${err}`);
      reject(err);
    });
  }

  _cacheSocketIfPersistent(call) {
    // If this is a persistent service client, we're here because we haven't connected to this service before.
    // Cache the service client for later use. Future calls won't need to lookup the service with the ROS master
    // or deal with the connection header.
    if (this.getPersist()) {
      this._serviceClient = call.serviceClient;
    }
  }

  _throwIfShutdown() {
    if (this.isShutdown()) {
      throw new Error('SHUTDOWN');
    }
  }
}

module.exports = ServiceClient;