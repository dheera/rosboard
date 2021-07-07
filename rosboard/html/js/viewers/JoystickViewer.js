"use strict";

class Viewer {
  /**
    * Class constructor.
    * @constructor
  **/
  constructor(card) {
    this.card = card;
    this.onCreate();
  }

  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // for MDL elements to get instantiated
    if(!(typeof(componentHandler) === 'undefined')){
      componentHandler.upgradeAllRegistered();
    }
  }

  /**
    * Gets called when Viewer is about to be destroyed.
    * @override
  **/
  onDestroy() { }

  /**
    * Gets called when the window is resized.
    * @override
  **/
  onResize() { }

  /**
    * Adds a topic to the viewer.
    * @override
  **/
  onData(data) { }

  update(data) { this.onData(data); }
}

Viewer.supportedTypes = [];
