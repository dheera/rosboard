"use strict";

class MotorCommandViewer extends MultiGraphViewer {
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

MotorCommandViewer._TYPE = 'robby_msgs/MotorCommand';
addViewer(MotorCommandViewer);

