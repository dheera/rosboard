const xmlrpc = require('xmlrpc');
const EventEmitter = require('events').EventEmitter;

class RosMasterStub extends EventEmitter {
  constructor(host, port) {
    super();

    this._host = host;
    this._port = port;

    this._apiMap = {
      getUri: this._onGetUri.bind(this),
      getParam: this._onGetParam.bind(this),
      registerService: this._onRegisterService.bind(this),
      unregisterService: this._onUnregisterService.bind(this),
      registerSubscriber: this._onRegisterSubscriber.bind(this),
      unregisterSubscriber: this._onUnregisterSubscriber.bind(this),
      registerPublisher: this._onRegisterPublisher.bind(this),
      unregisterPublisher: this._onUnregisterPublisher.bind(this),
      deleteParam: this._deleteParam.bind(this),
      setParam: this._setParam.bind(this),
      getParam: this._getParam.bind(this),
      hasParam: this._hasParam.bind(this),
      getParamNames: this._getParamNames.bind(this)
    };

    this._providedApis = new Set();

    this._clientCache = {
      service: null,
      sub: null,
      pub: null
    };

    this._params = {};

    this.verbose = true;

    this.listen();
  }

  listen() {
    this._server = xmlrpc.createServer({host: this._host, port: this._port}, () => {
      this.emit('ready');
    });

    this._server.on('NotFound', (method, params) => {
      if (this.verbose) {
        console.error('Method %s does not exist', method);
      }
    });
  }

  shutdown() {
    this._params = {};
    this._providedApis.clear();
    this.removeAllListeners();
    return new Promise((resolve, reject) => {
      this._server.close(resolve);
      this._server = null;
    });
  }

  provide(api) {
    const method = this._apiMap[api];
    if (method && !this._providedApis.has(api)) {
      this._server.on(api, (err, params, callback) => {
        this.emit(api, err, params, callback);
        method(err, params, callback);
      });
      this._providedApis.add(api);
    }
  }

  provideAll() {
    Object.keys(this._apiMap).forEach((api) => {
      this.provide(api);
    });
  }

  _onGetUri(err, params, callback) {
    const resp = [ 1, '', `${this._host}:${this._port}`];
    callback(null, resp);
  }

  _onGetParam(err, params, callback) {
    const resp = [0, '', 'Not implemented in stub'];
    callback(null, resp);
  }

  _onRegisterService(err, params, callback) {
    this._clientCache.service = params[2];

    const resp = [1, 'You did it!', []];
    callback(null, resp);
  }

  _onUnregisterService(err, params, callback) {
    const resp = [1, 'You did it!', this._clientCache.service ? 1 : 0];
    callback(null, resp);
    this._clientCache.service = null;
  }

  _onLookupService(err, params, callback) {
    const { service } = this._clientCache;
    if (service) {
      const resp = [1, "you did it", service];
      callback(null, resp);
    }
    else {
      const resp = [-1, "no provider", ""];
      callback(null, resp);
    }
  }

  _onRegisterSubscriber(err, params, callback) {
    this._clientCache.sub = params[3];

    const resp =  [1, 'You did it!', []];
    if (this._clientCache.pub) {
      resp[2].push(this._clientCache.pub);
    }
    callback(null, resp);
  }

  _onUnregisterSubscriber(err, params, callback) {
    const resp =  [1, 'You did it!', this._clientCache.sub ? 1 : 0];
    callback(null, resp);
    this._clientCache.sub = null;
  }

  _onRegisterPublisher(err, params, callback) {
    const pubInfo = params[3];
    this._clientCache.pub = pubInfo;

    const resp =  [1, 'You did it!', []];
    if (this._clientCache.sub) {
      resp[2].push(pubInfo);
      let subAddrParts = this._clientCache.sub.replace('http://', '').split(':');
      let client = xmlrpc.createClient({host: subAddrParts[0], port: subAddrParts[1]});
      let data = [1, topic, [pubInfo]];
      client.methodCall('publisherUpdate', data, (err, response) => { });
    }
    callback(null, resp);
  }

  _onUnregisterPublisher(err, params, callback) {
    const resp =  [1, 'You did it!', this._clientCache.pub ? 1 : 0];
    callback(null, resp);
    this._clientCache.pub = null;
  }

  // Param stubbing
  // NOTE: this is NOT a spec ParamServer implementation,
  // but it provides simple stubs for calls

  _deleteParam(err, params, callback) {
    const key = params[1];
    if (this._params.hasOwnProperty(key)) {
      delete this._params[key];
      const resp = [1, 'delete value for ' + key, 1];
      callback(null, resp);
    }
    else {
      const resp = [0, 'no value for ' + key, 1];
      callback(null, resp);
    }
  }

  _setParam(err, params, callback) {
    const key = params[1];
    const val = params[2];
    this._params[key] = val;
    const resp = [1, 'set value for ' + key, 1];
    callback(null, resp);
  }

  _getParam(err, params, callback) {
    const key = params[1];
    const val = this._params[key];
    if (val !== undefined) {
      const resp = [1, 'data for ' + key, val];
      callback(null, resp);
    }
    else {
      const resp = [0, 'no data for ' + key, null];
      callback(null, resp);
    }
  }

  _hasParam(err, params, callback) {
    const key = params[1];
    const resp = [1, 'check param ' + key, this._params.hasOwnProperty(key)];
    callback(null, resp);
  }

  _getParamNames(err, params, callback) {
    const names = Object.keys(this._params);
    const resp = [1, 'get param names', names];
    callback(null, resp);
  }
}

module.exports = RosMasterStub;
