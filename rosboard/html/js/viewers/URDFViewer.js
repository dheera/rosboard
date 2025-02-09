"use strict";

class URDFViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.gotData = false;
  
    this.viewerNode = $('<div><iframe src="/urdf/simple.html" width="200" height="400"/></div>')
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

    // This is a hack to remove the spinner. It should be removed when the data is received.
    // But data is rarely received for this viewer.
    window.setTimeout(1000, () => {
      if (!this.gotData) {
        this.update(''); // This will remove the spinner
      }
    })
  }

  onData(data) {
    this.gotData = true;
    this.card.title.text(data._topic_name);
  }
}

URDFViewer.friendlyName = "URDF Viewer";

URDFViewer.supportedTypes = [
    "std_msgs/String",
];

Viewer.registerViewer(URDFViewer);