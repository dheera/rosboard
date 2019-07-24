"use strict";

class CameraInfoViewer extends RawViewer {
  onCreate() {
    super.onCreate();
  }

  getData(message) {
    return {
      'height': message.height,
      'width': message.width,
      'distortion_model': message.distortion_model,
      'D': message.D,
      'K': message.K,
      'R': message.R,
      'P': message.P,
      'binning_x': message.binning_x,
      'binning_y': message.binning_y,
      'roi': message.roi,
    };
  }
}

CameraInfoViewer._TYPE = 'sensor_msgs/CameraInfo';
CameraInfoViewer._SINGLE = true;
addViewer(CameraInfoViewer);

