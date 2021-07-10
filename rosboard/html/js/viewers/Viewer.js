"use strict";

class Viewer {
  /**
    * Class constructor.
    * @constructor
  **/
  constructor(card) {
    this.card = card;
    this.onCreate();

    this.lastDataTime = 0.0;
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

  update(data) {
    let time = Date.now();
    if( (time - this.lastDataTime)/1000.0 < 1/this.constructor.maxUpdateRate) {
      return;
    }

    this.lastDataTime = time;
    this.onData(data);
  }
}

Viewer.supportedTypes = [];
Viewer.maxUpdateRate = 50.0;


Viewer.viewers = [];
Viewer.registerViewer = (viewer) => { Viewer.viewers.push(viewer); };
Viewer.getViewerForType = (type) => {
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }
  for(let i in Viewer.viewers) {
    if(Viewer.viewers[i].supportedTypes.includes(type)) {
      return Viewer.viewers[i];
    }
    if(Viewer.viewers[i].supportedTypes.includes("*")) {
      return Viewer.viewers[i];
    }
  }
  return null;
}
