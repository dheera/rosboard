"use strict";

module.exports = {
  node: null,

  getNodeName() {
    if (this.node) {
      return this.node.getNodeName();
    }
    // else
    return null;
  },

  ok() {
    return this.node && !this.node.isShutdown();
  },

  shutdown() {
    if (this.ok()) {
      return this.node.shutdown();
    }
    // else
    return Promise.resolve();
  },

  on(evt, handler) {
    if (this.ok()) {
      return this.node.on(evt, handler);
    }
  },

  once(evt, handler) {
    if (this.ok()) {
      return this.node.once(evt, handler);
    }
  },

  removeListener(evt, handler) {
    if (this.ok()) {
      return this.node.removeListener(evt, handler);
    }
  }
};