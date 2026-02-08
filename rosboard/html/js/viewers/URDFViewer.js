"use strict";

class URDFViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.gotData = false;
  
    this.viewerNode = $(`<div><iframe id="${this.topicName}" src="/urdf/simple.html" width="200" height="400"/></div>`)
      .css({
        'font-size': '11pt',
      })
      .appendTo(this.card.content);

    this.viewerNodeFadeTimeout = null;

    this.expandFields = { };
    this.fieldNodes = { };
    this.dataTable = '';
    this.card.title.text(this.topicName);
    super.onCreate();
  }

  onData(data) {
    const frame = document.getElementById(this.topicName);
    if (frame) {
      frame.contentWindow.postMessage(data, "*");
    } else {
      console.error("URDFViewer: Could not find iframe with id", this.topicName);
    }
  }
}

URDFViewer.friendlyName = "URDF Viewer";

URDFViewer.supportedTypes = [
    "sensor_msgs/msg/JointState"
];

Viewer.registerViewer(URDFViewer);