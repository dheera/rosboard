"use strict";

module.exports = {
  rebroadcast(evt, emitter, rebroadcaster) {
    emitter.on(evt, rebroadcaster.emit.bind(rebroadcaster, evt));
  }
};