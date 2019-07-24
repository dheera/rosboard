"use strict";

class Float64PairViewer extends MultiGraphViewer {
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

Float64PairViewer._TYPE = 'robby_msgs/Float64Pair';
Float64PairViewer._SINGLE = true;
addViewer(Float64PairViewer);

