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

const EventEmitter = require('events');
const Ultron = require('ultron');

var _require = require('../utils/event_utils.js');

const rebroadcast = _require.rebroadcast;

//-----------------------------------------------------------------------

/**
 * @class Subscriber
 * Public facing subscriber class. Allows users to listen to messages from
 * publishers on a given topic.
 */

class Subscriber extends EventEmitter {
  constructor(impl) {
    super();

    ++impl.count;
    this._impl = impl;
    this._ultron = new Ultron(impl);

    this._topic = impl.getTopic();
    this._type = impl.getType();

    rebroadcast('registered', this._ultron, this);
    rebroadcast('connection', this._ultron, this);
    rebroadcast('disconnect', this._ultron, this);
    rebroadcast('error', this._ultron, this);
    rebroadcast('message', this._ultron, this);
  }

  /**
   * Get the topic this publisher is publishing on
   * @returns {string}
   */
  getTopic() {
    return this._topic;
  }

  /**
   * Get the type of message this publisher is sending
   *            (e.g. std_msgs/String)
   * @returns {string}
   */
  getType() {
    return this._type;
  }

  /**
   * Get the number of publishers currently connected to this subscriber
   * @returns {number}
   */
  getNumPublishers() {
    if (this._impl) {
      return this._impl.getNumPublishers();
    }
    // else
    return 0;
  }

  /**
   * Shuts down this subscriber. If this is the last subscriber on this topic
   * for this node, closes the subscriber and unregisters the topic from Master
   * @returns {Promise}
   */
  shutdown() {
    if (this._impl) {
      const impl = this._impl;
      this._impl = null;
      this._ultron.destroy();
      this._ultron = null;

      --impl.count;
      if (impl.count <= 0) {
        return impl.getNode().unsubscribe(impl.getTopic());
      }

      this.removeAllListeners();
    }
    // else
    return Promise.resolve();
  }

  /**
   * Check if this publisher has been shutdown
   * @returns {boolean}
   */
  isShutdown() {
    return !!this._impl;
  }
}

//-----------------------------------------------------------------------

module.exports = Subscriber;