"use strict";

class StringViewer extends RawViewer {
  onCreate() {
    super.onCreate();

    // This viewer will try to expand JSON strings into multiple fields.
    // If a failure occurs, JSONFailure is set to true and the viewer will not try again.
    this.JSONFailure = false;
  }

  getData(message) {
    if(!this.JSONFailure && message.data.substr(0,1) === '{' && message.data.substr(-1,1) === '}') {
      try {
          return JSON.parse(message.data);
      } catch(e) {
          console.log('Looks like JSON but invalid: ' + message.data);
          this.JSONFailure = true;
      }
    }
    return message;
  }
}

StringViewer._TYPE = 'std_msgs/String';
addViewer(StringViewer);

