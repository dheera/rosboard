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

const Logging = require('./Logging.js');
const XmlrpcClient = require('../utils/XmlrpcClient.js');

//-----------------------------------------------------------------------

class ParamServerApiClient {
  constructor(xmlrpcClient) {
    this._log = Logging.getLogger(Logging.DEFAULT_LOGGER_NAME + '.params');

    this._xmlrpcClient = xmlrpcClient;
  }

  _call(method, data, resolve, reject) {
    this._xmlrpcClient.call(method, data, resolve, reject);
  }

  deleteParam(callerId, key) {
    let data = [callerId, key];

    return new Promise((resolve, reject) => {
      this._call('deleteParam', data, resolve, reject);
    });
  }

  setParam(callerId, key, value) {
    let data = [callerId, key, value];

    return new Promise((resolve, reject) => {
      this._call('setParam', data, resolve, reject);
    });
  }

  getParam(callerId, key) {
    let data = [callerId, key];

    return new Promise((resolve, reject) => {
      this._call('getParam', data, resp => {
        // resp[2] is the actual parameter value, and presumably all anyone cares about
        resolve(resp[2]);
      }, reject);
    });
  }

  searchParam(callerId, key) {
    throw new Error('NOT IMPLEMENTED');
  }

  subscribeParam(callerId, key) {
    throw new Error('NOT IMPLEMENTED');
  }

  unsubscribeParam(callerId, key) {
    throw new Error('NOT IMPLEMENTED');
  }

  hasParam(callerId, key) {
    let data = [callerId, key];

    return new Promise((resolve, reject) => {
      this._call('hasParam', data, resp => {
        // resp[2] is whether it actually has param and presumably all anyone  cares about
        resolve(resp[2]);
      }, reject);
    });
  }

  getParamNames(callerId) {
    let data = [callerId];

    return new Promise((resolve, reject) => {
      this._call('getParamNames', data, resp => {
        // resp[2] is parameter name list and presumably all anyone cares about
        resolve(resp[2]);
      }, reject);
    });
  }
}

//-----------------------------------------------------------------------

module.exports = ParamServerApiClient;