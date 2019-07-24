"use strict";

class Float32PairViewer extends MultiGraphViewer {
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

Float32PairViewer._TYPE = 'robby_msgs/Float32Pair';
Float32PairViewer._SINGLE = true;
addViewer(Float32PairViewer);

