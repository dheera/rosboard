"use strict";

class Int64PairViewer extends MultiGraphViewer {
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

Int64PairViewer._TYPE = 'robby_msgs/Int64Pair';
Int64PairViewer._SINGLE = true;
addViewer(Int64PairViewer);

