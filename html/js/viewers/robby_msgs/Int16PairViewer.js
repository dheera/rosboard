"use strict";

class Int16PairViewer extends MultiGraphViewer {
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

Int16PairViewer._TYPE = 'robby_msgs/Int16Pair';
Int16PairViewer._SINGLE = true;
addViewer(Int16PairViewer);

