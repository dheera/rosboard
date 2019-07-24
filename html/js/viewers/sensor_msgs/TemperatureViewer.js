"use strict";

class TemperatureViewer extends GraphViewer {
  onCreate() {
    this.maxPoints = 1024;
    this.updateInterval = 500;
    super.onCreate();
  }

  getData(message) {
    return { 'T': message.temperature };
  }
}

TemperatureViewer._TYPE = 'sensor_msgs/Temperature';
addViewer(TemperatureViewer);

