"use strict";

class RangeViewer extends GraphViewer {
  onCreate() {
    this.maxPoints = 256;
    this.updateInterval = 100;
    super.onCreate();
  }

  getData(message) {
    return { 'r': message.range };
  }
}

RangeViewer._TYPE = 'sensor_msgs/Range';
addViewer(RangeViewer);

