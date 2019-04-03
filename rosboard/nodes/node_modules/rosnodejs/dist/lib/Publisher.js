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
 *    Unless required by applicable law or agreed to in writing,
 *    software distributed under the License is distributed on an "AS
 *    IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 *    express or implied. See the License for the specific language
 *    governing permissions and limitations under the License.
 */

"use strict";

const EventEmitter = require('events');
const Ultron = require('ultron');

var _require = require('../utils/event_utils.js');

const rebroadcast = _require.rebroadcast;

/**
 * @class Publisher
 * Public facing publishers class. Allows users to send messages to subscribers
 * on a given topic.
 */

class Publisher extends EventEmitter {
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
   * Check if this publisher is latching
   * @returns {boolean}
   */
  getLatching() {
    if (this._impl) {
      return this._impl.getLatching();
    }
    // else
    return false;
  }

  /**
   * Get the numbber of subscribers currently connected to this publisher
   * @returns {number}
   */
  getNumSubscribers() {
    if (this._impl) {
      return this._impl.getNumSubscribers();
    }
    // else
    return 0;
  }

  /**
   * Shuts down this publisher. If this is the last publisher on this topic
   * for this node, closes the publisher and unregisters the topic from Master
   * @returns {Promise}
   */
  shutdown() {
    const topic = this.getTopic();
    if (this._impl) {
      const impl = this._impl;
      this._impl = null;
      this._ultron.destroy();
      this._ultron = null;

      --impl.count;
      if (impl.count <= 0) {
        return impl.getNode().unadvertise(impl.getTopic());
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

  /**
   * Schedule the msg for publishing - or publish immediately if we're
   * supposed to
   * @param msg {object} object type matching this._type
   * @param [throttleMs] {number} optional override for publisher setting
   */
  publish(msg, throttleMs) {
    this._impl.publish(msg, throttleMs);
  }
}

module.exports = Publisher;