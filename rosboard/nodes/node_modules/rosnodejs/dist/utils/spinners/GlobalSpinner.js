'use strict';

const DEFAULT_SPIN_RATE_HZ = 200;
const events = require('events');
const LoggingManager = require('../../lib/Logging.js');
const log = LoggingManager.getLogger('ros.spinner');

const ClientQueue = require('./ClientQueue.js');

const PING_OP = 'ping';
const DELETE_OP = 'delete';
const ADD_OP = 'add';

/**
 * @class GlobalSpinner
 * Clients (subscribers and publishers) will register themselves with the node's spinner
 * when they're created. Clients will disconnect from the spinner whenever they're shutdown.
 * Whenever they receive a new message to handle, those clients will "ping" the spinner,
 * which will push the new message onto that client's queue and add the client to a list
 * of clients to be handled on the next spin. While spinning, the spinner is locked and
 * ping and disconnect operations are cached in order to ensure that changes aren't
 * made to the spinner during its execution (e.g. subscriber callback publishes a message,
 * publisher pings the spinner which queues the new message and adds the client to its callback
 * list, the client list is cleared at the end of the spin and this client has a
 * message hanging in its queue that will never be handled). Once all of the messages
 * received since the last spin are handled the Spinner is unlocked and all cached
 * ping and disconnect operations are replayed in order.
 */
class GlobalSpinner extends events {
  constructor() {
    var _ref = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {},
        _ref$spinRate = _ref.spinRate;

    let spinRate = _ref$spinRate === undefined ? null : _ref$spinRate;
    var _ref$emit = _ref.emit;
    let emit = _ref$emit === undefined ? false : _ref$emit;

    super();

    if (typeof spinRate === 'number') {
      this._spinTime = 1 / spinRate;
    } else {
      this._spinTime = 0;
    }

    this._spinTimer = null;

    this._clientCallQueue = [];
    this._clientQueueMap = new Map();

    /**
     * Acts as a mutex while handling messages in _handleQueue
     * @type {boolean}
     * @private
     */
    this._queueLocked = false;
    this._lockedOpCache = [];

    // emit is just for testing purposes
    this._emit = emit;
  }

  clear() {
    clearTimeout(this._spinTimer);
    this._queueLocked = false;
    this._clientQueueMap.forEach(clientQueue => {
      clientQueue.destroy();
    });
    this._clientQueueMap.clear();
    this._clientCallQueue = [];
  }

  addClient(client, clientId, queueSize, throttleMs) {
    if (this._queueLocked) {
      this._lockedOpCache.push({ op: ADD_OP, client, clientId, queueSize, throttleMs });
    } else if (queueSize > 0) {
      this._clientQueueMap.set(clientId, new ClientQueue(client, queueSize, throttleMs));
    }
  }

  /**
   * When subscribers/publishers receive new messages to handle, they will
   * "ping" the spinner.
   * @param clientId
   * @param msg
   */
  ping() {
    let clientId = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;
    let msg = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;

    if (!clientId || !msg) {
      throw new Error('Trying to ping spinner without clientId');
    }

    if (this._queueLocked) {
      this._lockedOpCache.push({ op: PING_OP, clientId, msg });
    } else {
      this._queueMessage(clientId, msg);
      this._setTimer();
    }
  }

  disconnect(clientId) {
    if (this._queueLocked) {
      this._lockedOpCache.push({ op: DELETE_OP, clientId });
    } else {
      const index = this._clientCallQueue.indexOf(clientId);
      if (index !== -1) {
        this._clientCallQueue.splice(index, 1);
      }
      this._clientQueueMap.delete(clientId);
    }
  }

  _queueMessage(clientId, message) {
    const clientQueue = this._clientQueueMap.get(clientId);
    if (!clientQueue) {
      throw new Error(`Unable to queue message for unknown client ${clientId}`);
    }
    // else
    if (clientQueue.length === 0) {
      this._clientCallQueue.push(clientId);
    }

    clientQueue.push(message);
  }

  _handleLockedOpCache() {
    const len = this._lockedOpCache.length;
    for (let i = 0; i < len; ++i) {
      var _lockedOpCache$i = this._lockedOpCache[i];
      const op = _lockedOpCache$i.op,
            clientId = _lockedOpCache$i.clientId,
            msg = _lockedOpCache$i.msg,
            client = _lockedOpCache$i.client,
            queueSize = _lockedOpCache$i.queueSize,
            throttleMs = _lockedOpCache$i.throttleMs;

      if (op === PING_OP) {
        this.ping(clientId, msg);
      } else if (op === DELETE_OP) {
        this.disconnect(clientId);
      } else if (op === ADD_OP) {
        this.addClient(client, clientId, queueSize, throttleMs);
      }
    }
    this._lockedOpCache = [];
  }

  _setTimer() {
    if (this._spinTimer === null) {
      if (this._emit) {
        this._spinTimer = setTimeout(() => {
          this._handleQueue();
          this.emit('tick');
        }, this._spinTime);
      } else {
        this._spinTimer = setTimeout(this._handleQueue.bind(this), this._spinTime);
      }
    }
  }

  _handleQueue() {
    // lock the queue so that ping and disconnect operations are cached
    // while we're running through the call list instead of modifying
    // the list beneath us.
    this._queueLocked = true;
    const now = Date.now();
    const keepOnQueue = [];
    let len = this._clientCallQueue.length;
    for (let i = 0; i < len; ++i) {
      const clientId = this._clientCallQueue[i];
      const clientQueue = this._clientQueueMap.get(clientId);
      if (!clientQueue.handleClientMessages(now)) {
        keepOnQueue.push(clientId);
      }
    }

    if (keepOnQueue.length > 0) {
      this._clientCallQueue = keepOnQueue;
    } else {
      this._clientCallQueue = [];
    }

    // unlock the queue now that we've handled everything
    this._queueLocked = false;
    // handle any operations that occurred while the queue was locked
    this._handleLockedOpCache();

    // TODO: figure out if these clients that are throttling messages are
    // consistently keeping the timer running when it otherwise wouldn't be
    // and eating up CPU. Consider starting a slower timer if the least-throttled
    // client won't be handled for N cycles (e.g N === 5).
    this._spinTimer = null;
    if (this._clientCallQueue.length > 0) {
      this._setTimer();
    }
  }
}

module.exports = GlobalSpinner;