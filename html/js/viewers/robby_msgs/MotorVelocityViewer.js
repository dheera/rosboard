"use strict";

class MotorVelocityViewer extends MultiGraphViewer {
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

MotorVelocityViewer._TYPE = 'robby_msgs/MotorVelocity';
MotorVelocityViewer._SINGLE = true;
addViewer(MotorVelocityViewer);

