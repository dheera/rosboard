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

const os = require('os');
const RemapUtils = require('./remapping_utils');

let HOST = null;

const NetworkUtils = {

  init(remappings) {
    const ip = remappings[RemapUtils.SPECIAL_KEYS.ip];
    const host = remappings[RemapUtils.SPECIAL_KEYS.hostname];

    const ROS_IP = process.env.ROS_IP;
    const ROS_HOSTNAME = process.env.ROS_HOSTNAME;

    HOST = ip || host || ROS_IP || ROS_HOSTNAME || os.hostname();
  },

  /**
   * FIXME: should this just return ROS_IP?
   * get this computer's (non-internal) ip address
   * @param [family] {string} 'IPv4', 'IPv6', ... 'IPv4' default
   * @param [networkInterface] {string} network interface to use ('eth0') else finds first match
   */
  getIpAddress: function getIpAddress(family, networkInterface) {
    family = family || 'IPv4';
    let interfaces = os.networkInterfaces();
    let interfaceNames;
    if (networkInterface && !ifaces.hasOwnProperty(networkInterface)) {
      return null;
    } else if (networkInterface) {
      interfaceNames = [networkInterface];
    } else {
      interfaceNames = Object.keys(interfaces);
    }

    let ipAddress = null;
    interfaceNames.some(ifName => {
      interfaces[ifName].forEach(iface => {
        if (iface.internal || family !== iface.family) {
          // skip over internal (i.e. 127.0.0.1) and addresses from different families
          return false;
        }

        ipAddress = iface.address;
        return true;
      });
    });
    return ipAddress;
  },

  getHost() {
    return HOST;
  },

  getAddressAndPortFromUri(uriString) {
    let regexStr = /(?:http:\/\/|rosrpc:\/\/)?([a-zA-Z\d\-.:]+):(\d+)/;
    let match = uriString.match(regexStr);
    if (match === null) {
      throw new Error('Unable to find host and port from uri ' + uriString + ' with regex ' + regexStr);
    }
    // else
    return {
      host: match[1],
      port: match[2]
    };
  },

  formatServiceUri(port) {
    return 'rosrpc://' + this.getHost() + ':' + port;
  }
};

//------------------------------------------------------------------

module.exports = NetworkUtils;