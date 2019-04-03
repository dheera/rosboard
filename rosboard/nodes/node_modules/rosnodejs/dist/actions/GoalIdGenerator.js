'use strict';

const ThisNode = require('../lib/ThisNode.js');
const Time = require('../lib/Time.js');

let GOAL_COUNT = 0;

module.exports = function (now) {
  if (!now || now.secs === undefined || now.nsecs === undefined) {
    now = Time.now();
  }

  ++GOAL_COUNT;
  if (GOAL_COUNT > Number.MAX_SAFE_INTEGER) {
    GOAL_COUNT = 0;
  }

  return `${ThisNode.getNodeName()}-${GOAL_COUNT}-${now.secs}.${now.nsecs}`;
};