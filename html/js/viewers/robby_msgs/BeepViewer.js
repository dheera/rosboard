"use strict";

class BeepViewer extends RawViewer {
  onCreate() {
    super.onCreate();
  }

  getData(message) {
    return {'frequency': message.frequency, 'duration': message.duration, 'amplitude': message.amplitude};
  }
}

BeepViewer._TYPE = 'robby_msgs/Beep';
addViewer(BeepViewer);

