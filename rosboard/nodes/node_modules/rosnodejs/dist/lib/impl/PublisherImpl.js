/*
 *    Copyright 2017 Rethink Robotics
 *
 *    Copyright 2017 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing,
 *    software distributed under the License is distributed on an "AS
 *    IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 *    express or implied. See the License for the specific language
 *    governing permissions and limitations under the License.
 */

"use strict";

const SerializationUtils = require('../../utils/serialization_utils.js');
const Serialize = SerializationUtils.Serialize;
const TcprosUtils = require('../../utils/tcpros_utils.js');
const EventEmitter = require('events');
const Logging = require('../Logging.js');

var _require = require('../../utils/ClientStates.js');

const REGISTERING = _require.REGISTERING,
      REGISTERED = _require.REGISTERED,
      SHUTDOWN = _require.SHUTDOWN;

/**
 * Implementation class for a Publisher. Handles registration, connecting to
 * subscribers, etc. Public-facing publisher classes will be given an instance
 * of this to use
 */

class PublisherImpl extends EventEmitter {
  constructor(options, nodeHandle) {
    super();

    this.count = 0;

    this._topic = options.topic;

    this._type = options.type;

    this._latching = !!options.latching;

    this._tcpNoDelay = !!options.tcpNoDelay;

    if (options.hasOwnProperty('queueSize')) {
      this._queueSize = options.queueSize;
    } else {
      this._queueSize = 1;
    }

    /**
     * throttleMs interacts with queueSize to determine when to send
     * messages.
     *  < 0  : send immediately - no interaction with queue
     * >= 0 : place event at end of event queue to publish message
         after minimum delay (MS)
     */
    if (options.hasOwnProperty('throttleMs')) {
      this._throttleMs = options.throttleMs;
    } else {
      this._throttleMs = 0;
    }

    // OPTIONS STILL NOT HANDLED:
    //  headers: extra headers to include
    //  subscriber_listener: callback for new subscribers connect/disconnect

    this._resolve = !!options.resolve;

    this._lastSentMsg = null;

    this._nodeHandle = nodeHandle;
    this._nodeHandle.getSpinner().addClient(this, this._getSpinnerId(), this._queueSize, this._throttleMs);

    this._log = Logging.getLogger('ros.rosnodejs');

    this._subClients = {};

    if (!options.typeClass) {
      throw new Error(`Unable to load message for publisher ${this.getTopic()} with type ${this.getType()}`);
    }
    this._messageHandler = options.typeClass;

    this._state = REGISTERING;
    this._register();
  }

  /**
   * Uniquely identifies this publisher to the node's Spinner
   * @returns {string}
   */
  _getSpinnerId() {
    return `Publisher://${this.getTopic()}`;
  }

  /**
   * Get the name of the topic this subscriber is listening on
   * @returns {string}
   */
  getTopic() {
    return this._topic;
  }

  /**
   * Get the type of message this subscriber is handling
   *            (e.g. std_msgs/String)
   * @returns {string}
   */
  getType() {
    return this._type;
  }

  /**
   * Check if this publisher is set to latch it's messages
   * @returns {boolean}
   */
  getLatching() {
    return this._latching;
  }

  /**
   * Get the count of subscribers connected to this publisher
   * @returns {number}
   */
  getNumSubscribers() {
    return Object.keys(this._subClients).length;
  }

  /**
   * Get the list of client addresses connect to this publisher.
   * Used for getBusInfo Slave API calls
   * @returns {Array}
   */
  getClientUris() {
    return Object.keys(this._subClients);
  }

  /**
   * Get the ros node this subscriber belongs to
   * @returns {RosNode}
   */
  getNode() {
    return this._nodeHandle;
  }

  /**
   * Clears and closes all client connections for this publisher.
   */
  shutdown() {
    this._state = SHUTDOWN;
    this._log.debug('Shutting down publisher %s', this.getTopic());

    Object.keys(this._subClients).forEach(clientId => {
      const client = this._subClients[clientId];
      client.end();
    });

    // disconnect from the spinner in case we have any pending callbacks
    this._nodeHandle.getSpinner().disconnect(this._getSpinnerId());
    this._subClients = {};
  }

  /**
   * Check if this publisher has been shutdown
   * @returns {boolean}
   */
  isShutdown() {
    return this._state === SHUTDOWN;
  }

  /**
   * Schedules the msg for publishing or publishes immediately if we're
   * supposed to (throttleTime < 0)
   * @param msg {object} object type matching this._type
   * @param [throttleMs] {number} optional override for publisher setting
   */
  publish(msg, throttleMs) {
    if (this.isShutdown()) {
      return;
    }

    if (typeof throttleMs !== 'number') {
      throttleMs = this._throttleMs;
    }

    if (throttleMs < 0) {
      // short circuit JS event queue, publish "synchronously"
      this._handleMsgQueue([msg]);
    } else {
      this._nodeHandle.getSpinner().ping(this._getSpinnerId(), msg);
    }
  }

  /**
   * Pulls all msgs off queue, serializes, and publishes them to all clients.
   * @param msgQueue {Array} Array of messages. Type of each message matches this._type
   */
  _handleMsgQueue(msgQueue) {

    // There's a small chance that we were shutdown while the spinner was locked
    // which could cause _handleMsgQueue to be called if this publisher was in there.
    if (this.isShutdown()) {
      return;
    }

    const numClients = this.getNumSubscribers();
    if (numClients === 0) {
      this._log.debugThrottle(2000, `Publishing message on ${this.getTopic()} with no subscribers`);
    }

    try {
      msgQueue.forEach(msg => {
        if (this._resolve) {
          msg = this._messageHandler.Resolve(msg);
        }

        const serializedMsg = TcprosUtils.serializeMessage(this._messageHandler, msg);

        Object.keys(this._subClients).forEach(client => {
          this._subClients[client].write(serializedMsg);
        });

        // if this publisher is supposed to latch,
        // save the last message. Any subscribers that connects
        // before another call to publish() will receive this message
        if (this.getLatching()) {
          this._lastSentMsg = serializedMsg;
        }
      });
    } catch (err) {
      this._log.error('Error when publishing message on topic %s: %s', this.getTopic(), err.stack);
      this.emit('error', err);
    }
  }

  /**
   * Handles a new connection from a subscriber to this publisher's node.
   * Validates the connection header and sends a response header
   * @param socket {Socket} client that sent the header
   * @param header {Object} deserialized connection header.
   */
  handleSubscriberConnection(socket, header) {
    let error = TcprosUtils.validateSubHeader(header, this.getTopic(), this.getType(), this._messageHandler.md5sum());

    if (error !== null) {
      this._log.error('Unable to validate subscriber connection header ' + JSON.stringify(header));
      socket.end(Serialize(error));
      return;
    }
    // else
    this._log.info('Pub %s got connection header %s', this.getTopic(), JSON.stringify(header));

    // create and send response
    let respHeader = TcprosUtils.createPubHeader(this._nodeHandle.getNodeName(), this._messageHandler.md5sum(), this.getType(), this.getLatching(), this._messageHandler.messageDefinition());
    socket.write(respHeader);

    // if this publisher had the tcpNoDelay option set
    // disable the nagle algorithm
    if (this._tcpNoDelay || header.tcp_nodelay === '1') {
      socket.setNoDelay(true);
    }

    socket.on('close', () => {
      this._log.info('Publisher client socket %s on topic %s disconnected', socket.name, this.getTopic());
      socket.removeAllListeners();
      delete this._subClients[socket.name];
      this.emit('disconnect');
    });

    socket.on('end', () => {
      this._log.info('Publisher client socket %s on topic %s ended the connection', socket.name, this.getTopic());
    });

    socket.on('error', err => {
      this._log.warn('Publisher client socket %s on topic %s had error: %s', socket.name, this.getTopic(), err);
    });

    // if we've cached a message from latching, send it now
    if (this._lastSentMsg !== null) {
      this._log.debug('Sending latched msg to new subscriber');
      socket.write(this._lastSentMsg);
    }

    // handshake was good - we'll start publishing to it
    this._subClients[socket.name] = socket;

    this.emit('connection', header, socket.name);
  }

  /**
   * Makes an XMLRPC call to registers this publisher with the ROS master
   */
  _register() {
    this._nodeHandle.registerPublisher(this._topic, this._type).then(resp => {
      // if we were shutdown between the starting the registration and now, bail
      if (this.isShutdown()) {
        return;
      }

      this._log.info('Registered %s as a publisher: %j', this._topic, resp);
      let code = resp[0];
      let msg = resp[1];
      let subs = resp[2];
      if (code === 1) {
        // registration worked
        this._state = REGISTERED;
        this.emit('registered');
      }
    }).catch(err => {
      this._log.error('Error while registering publisher %s: %s', this.getTopic(), err);
    });
  }
}

module.exports = PublisherImpl;