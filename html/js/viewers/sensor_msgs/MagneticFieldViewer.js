"use strict";

class MagneticFieldViewer extends GraphViewer {
  onCreate() {
    this.maxPoints = 256;
    this.updateInterval = 100;
    super.onCreate();
  }

  getData(message) {
    return {
     'x': message.magnetic_field.x,
     'y': message.magnetic_field.y,
     'z': message.magnetic_field.z
    };
  }
}

MagneticFieldViewer._TYPE = 'sensor_msgs/MagneticField';
addViewer(MagneticFieldViewer);

