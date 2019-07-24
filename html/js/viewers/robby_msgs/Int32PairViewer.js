"use strict";

class Int32PairViewer extends MultiGraphViewer {
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

Int32PairViewer._TYPE = 'robby_msgs/Int32Pair';
Int32PairViewer._SINGLE = true;
addViewer(Int32PairViewer);

