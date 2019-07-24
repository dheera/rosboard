"use strict";

class Int8PairViewer extends MultiGraphViewer {
  onCreate() {
    this.numPlots = 2;
    this.maxPoints = 256;
    this.updateInterval = 100;
    this.fields = {'left': {}, 'right': {}};
    super.onCreate();
  }

  getData(message) {
    return {'left': message.left, 'right': message.right};
  }
}

Int8PairViewer._TYPE = 'robby_msgs/Int8Pair';
Int8PairViewer._SINGLE = true;
addViewer(Int8PairViewer);

