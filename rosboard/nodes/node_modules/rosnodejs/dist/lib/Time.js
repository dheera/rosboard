'use strict';

const timeUtils = require('../utils/time_utils.js');

let simTimeSub = null;
let simTime = timeUtils.dateToRosTime(0);

function handleSimTimeMessage(msg) {
  simTime = msg.clock;
}

const Time = {
  useSimTime: false,

  _initializeRosTime(rosnodejs, notime) {
    //Only for testing purposes!
    if (notime) {
      return Promise.resolve();
    }
    const nh = rosnodejs.nh;
    return nh.getParam('/use_sim_time').then(val => {
      this.useSimTime = val;

      if (val) {
        simTimeSub = nh.subscribe('/clock', 'rosgraph_msgs/Clock', handleSimTimeMessage, { throttleMs: -1 });
      }
    }).catch(err => {
      if (err.statusCode === undefined) {
        throw err;
      }
    });
  },

  now() {
    if (this.useSimTime) {
      return simTime;
    }
    // else
    return timeUtils.now();
  },

  rosTimeToDate: timeUtils.rosTimeToDate,
  dateToRosTime: timeUtils.dateToRosTime,
  epoch: timeUtils.epoch,
  isZeroTime: timeUtils.isZeroTime,
  toNumber: timeUtils.toNumber,
  toSeconds: timeUtils.toSeconds,
  timeComp: timeUtils.timeComp,
  add: timeUtils.add,
  lt: timeUtils.lt
};

module.exports = Time;