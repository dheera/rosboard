"use strict";

class RelativeHumidityViewer extends GraphViewer {
  onCreate() {
    this.maxPoints = 1024;
    this.updateInterval = 500;
    super.onCreate();
  }

  getData(message) {
    return { 'h': message.relative_humidity };
  }
}

RelativeHumidityViewer._TYPE = 'sensor_msgs/RelativeHumidity';
addViewer(RelativeHumidityViewer);

