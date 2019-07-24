"use strict";

class MotorCountsViewer extends MultiGraphViewer {
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

MotorCountsViewer._TYPE = 'robby_msgs/MotorCounts';
MotorCountsViewer._SINGLE = true;
addViewer(MotorCountsViewer);

