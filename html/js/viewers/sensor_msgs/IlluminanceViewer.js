"use strict";

class IlluminanceViewer extends GraphViewer {
  onCreate() {
    this.maxPoints = 256;
    this.updateInterval = 100;
    super.onCreate();
  }

  getData(message) {
    return { 'I': message.illuminance };
  }
}

IlluminanceViewer._TYPE = 'sensor_msgs/Illuminance';
addViewer(IlluminanceViewer);

