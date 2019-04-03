'use strict';

const SPECIAL_KEYS = {
  name: '__name',
  log: '__log', // I don't think rosnodejs is responsible for changing the log directory
  ip: '__ip',
  hostname: '__hostname',
  master: '__master',
  ns: '__ns'
};

const RemapUtils = {
  SPECIAL_KEYS,
  processRemapping(args) {
    const len = args.length;

    const remapping = {};

    for (let i = 0; i < len; ++i) {
      const arg = args[i];
      let p = arg.indexOf(':=');
      if (p >= 0) {
        const local = arg.substring(0, p);
        const external = arg.substring(p + 2);

        remapping[local] = external;
      }
    }

    return remapping;
  }
};

module.exports = RemapUtils;