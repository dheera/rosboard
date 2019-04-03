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

let xmlrpc = require('xmlrpc');
let networkUtils = require('../utils/network_utils.js');
let Logging = require('./Logging.js');
const XmlrpcClient = require('../utils/XmlrpcClient.js');

//-----------------------------------------------------------------------

class MasterApiClient {

  constructor(rosMasterUri, logName) {
    this._log = Logging.getLogger(Logging.DEFAULT_LOGGER_NAME + '.masterapi');
    this._log.info('Connecting to ROS Master at ' + rosMasterUri);
    this._xmlrpcClient = new XmlrpcClient(networkUtils.getAddressAndPortFromUri(rosMasterUri), this._log);
  }

  getXmlrpcClient() {
    return this._xmlrpcClient;
  }

  _call(method, data, resolve, reject) {
    let options = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : {};

    this._xmlrpcClient.call(method, data, resolve, reject, options);
  }

  registerService(callerId, service, serviceUri, uri, options) {
    let data = [callerId, service, serviceUri, uri];

    return new Promise((resolve, reject) => {
      this._call('registerService', data, resolve, reject, options);
    });
  }

  unregisterService(callerId, service, serviceUri, options) {
    let data = [callerId, service, serviceUri];

    return new Promise((resolve, reject) => {
      this._call('unregisterService', data, resolve, reject, options);
    });
  }

  registerSubscriber(callerId, topic, topicType, uri, options) {
    let data = [callerId, topic, topicType, uri];
    return new Promise((resolve, reject) => {
      this._call('registerSubscriber', data, resolve, reject, options);
    });
  }

  unregisterSubscriber(callerId, topic, uri, options) {
    let data = [callerId, topic, uri];
    return new Promise((resolve, reject) => {
      this._call('unregisterSubscriber', data, resolve, reject, options);
    });
  }

  registerPublisher(callerId, topic, topicType, uri, options) {
    let data = [callerId, topic, topicType, uri];
    return new Promise((resolve, reject) => {
      this._call('registerPublisher', data, resolve, reject, options);
    });
  }

  unregisterPublisher(callerId, topic, uri, options) {
    let data = [callerId, topic, uri];
    return new Promise((resolve, reject) => {
      this._call('unregisterPublisher', data, resolve, reject, options);
    });
  }

  lookupNode(callerId, nodeName, options) {
    let data = [callerId, nodeName];
    return new Promise((resolve, reject) => {
      this._call('lookupNode', data, resolve, reject, options);
    });
  }

  getPublishedTopics(callerId, subgraph, options) {
    let data = [callerId, subgraph];
    return new Promise((resolve, reject) => {
      this._call('getPublishedTopics', data, function (data) {
        return resolve({
          topics: data[2].map(topic => {
            return {
              name: topic[0], type: topic[1]
            };
          })
        });
      }, reject, options);
    });
  }

  getTopicTypes(callerId, options) {
    let data = [callerId];
    return new Promise((resolve, reject) => {
      this._call('getTopicTypes', data, function (data) {
        return resolve({
          topics: data[2].map(topic => {
            return {
              name: topic[0], type: topic[1]
            };
          })
        });
      }, reject, options);
    });
  }

  /** return an object containing all current publishers (by topic),
      subscribers (by topic), and services (by name) */
  getSystemState(callerId, options) {
    function toObject(memo, sublist) {
      memo[sublist[0]] = sublist[1];
      return memo;
    }

    let data = [callerId];
    return new Promise((resolve, reject) => {
      this._call('getSystemState', data, function (data) {
        return resolve({
          publishers: data[2][0].reduce(toObject, {}),
          subscribers: data[2][1].reduce(toObject, {}),
          services: data[2][2].reduce(toObject, {})
        });
      }, reject, options);
    });
  }

  getUri(callerId, options) {
    let data = [callerId];
    return new Promise((resolve, reject) => {
      this._call('getUri', data, resolve, reject, options);
    });
  }

  lookupService(callerId, service, options) {
    let data = [callerId, service];
    return new Promise((resolve, reject) => {
      this._call('lookupService', data, resolve, reject, options);
    });
  }
};

//-----------------------------------------------------------------------

module.exports = MasterApiClient;