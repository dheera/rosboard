'use strict';

const EventEmitter = require('events');

/**
 * @class ClientQueue
 * Queue of messages to handle for an individual client (subscriber or publisher)
 */
class ClientQueue extends EventEmitter {
  constructor(client, queueSize, throttleMs) {
    super();

    if (queueSize < 1) {
      queueSize = Number.POSITIVE_INFINITY;
    }

    this._client = client;

    this._queue = [];
    this._queueSize = queueSize;

    this.throttleMs = throttleMs;
    this._handleTime = null;
  }

  destroy() {
    this._queue = [];
    this._client = null;
    this._handleTime = null;
  }

  push(item) {
    this._queue.push(item);
    if (this.length > this._queueSize) {
      this._queue.shift();
    }
  }

  get length() {
    return this._queue.length;
  }

  handleClientMessages(time) {
    if (this._handleTime === null || time - this._handleTime >= this.throttleMs) {
      this._handleTime = time;
      this._client._handleMsgQueue(this._queue);
      this._queue = [];
      return true;
    }
    // else
    return false;
  }
}

module.exports = ClientQueue;