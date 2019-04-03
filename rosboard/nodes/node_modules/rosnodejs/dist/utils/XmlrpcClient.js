'use strict';

const EventEmitter = require('events');
const xmlrpc = require('xmlrpc');

const CONNECTION_REFUSED = 'ECONNREFUSED';
const TRY_AGAIN_LIST = [1, 2, 2, 4, 4, 4, 4, 8, 8, 8, 8, 16, 16, 32, 64, 128, 256, 512, 1024, 2048];

class XmlrpcCall {
  constructor(method, data, resolve, reject) {
    let options = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : {};

    this.method = method;
    this.data = data;
    this.resolve = resolve;
    this.reject = reject;

    this.maxAttempts = options.maxAttempts || Infinity;
  }

  call(client) {
    return new Promise((resolve, reject) => {
      client.methodCall(this.method, this.data, (err, resp) => {
        if (err) {
          reject(err);
        } else if (resp[0] !== 1) {
          const msg = resp[1];
          const error = new Error(`ROS XMLRPC Error: ${msg}`);
          error.code = 'EROSAPIERROR';
          error.statusCode = resp[0];
          error.statusMessage = msg;
          error.value = resp[2];
          reject(error);
        } else {
          resolve(resp);
        }
      });
    });
  }
}

class XmlrpcClient extends EventEmitter {
  constructor(clientAddressInfo, log) {
    super();

    this._xmlrpcClient = xmlrpc.createClient(clientAddressInfo);

    this._log = log;

    this._callQueue = [];

    this._timeout = 0;
    this._timeoutId = null;

    this._failedAttempts = 0;
  }

  getClient() {
    return this._xmlrpcClient;
  }

  call(method, data, resolve, reject, options) {
    const newCall = new XmlrpcCall(method, data, resolve, reject, options);
    const numCalls = this._callQueue.length;
    this._callQueue.push(newCall);
    // if nothing else was on the queue, try executing the call now
    if (numCalls === 0) {
      this._tryExecuteCall();
    }
  }

  clear() {
    this._log.info('Clearing xmlrpc client queue...');
    if (this._callQueue.length !== 0) {
      this._callQueue[0].reject(new Error('Clearing call queue - probably shutting down...'));
    }
    clearTimeout(this._timeoutId);
    this._callQueue = [];
  }

  _tryExecuteCall() {
    if (this._callQueue.length === 0) {
      this._log.warn('Tried executing xmlprc call on empty queue');
      return;
    }
    // else
    const call = this._callQueue[0];
    this._log.info('Try execute call %s: %j', call.method, call.data);
    call.call(this._xmlrpcClient).then(resp => {
      // call succeeded, clean up and call its handler
      this._log.info('Call %s %j succeeded! %j', call.method, call.data, resp);
      this._shiftQueue();
      this._resetTimeout();
      call.resolve(resp);
    }).catch(err => {
      ++this._failedAttempts;
      this._log.info('Call %s %j failed! %s', call.method, call.data, err);
      if (err instanceof Error && err.code === CONNECTION_REFUSED && this._failedAttempts < call.maxAttempts) {
        // Call failed to connect - try to connect again.
        // All future calls would have same error since they're
        // directed at the same xmlrpc server.
        this._log.info('Trying call again on attempt %d of %d', this._failedAttempts, call.maxAttempts);
        this._scheduleTryAgain();
        this.emit(CONNECTION_REFUSED, err, this._failedAttempts);
      } else {
        // call failed - move on.
        this._shiftQueue();
        this._resetTimeout();
        call.reject(err);
      }
    }).then(() => {
      if (this._timeoutId === null && this._callQueue.length > 0) {
        this._tryExecuteCall();
      }
    });
  }

  _shiftQueue() {
    this._callQueue.shift();
  }

  _resetTimeout() {
    this._timeout = 0;
    this._timeoutId = null;
    this._failedAttempts = 0;
  }

  _scheduleTryAgain() {
    const timeout = TRY_AGAIN_LIST[this._timeout];
    if (this._timeout + 1 < TRY_AGAIN_LIST.length) {
      ++this._timeout;
    }
    this._log.info('Scheduling call again in %dms', timeout);
    this._timeoutId = setTimeout(this._tryExecuteCall.bind(this), timeout);
  }
}

module.exports = XmlrpcClient;