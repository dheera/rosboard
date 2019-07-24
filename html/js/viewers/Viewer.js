"use strict";

class Viewer {
  /**
    * Class constructor.
    * @constructor
  **/
  constructor(cardContentNode) {
    this.cardContentNode = cardContentNode;
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
  addTopic() { }

  /**
    * Removes a topic from the viewer.
    * @override
  **/
  removeTopic() { }

  /**
    * Returns a list of topics subscribed to.
    * @override
  **/
  getTopics() { }
}
