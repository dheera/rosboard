"use strict";

class URDFViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewerNode = $('<div><iframe src="/urdf/simple.html" width="200" height="400"/></div>')
      .css({
        'font-size': '11pt',
      })
      .appendTo(this.card.content);

    this.viewerNodeFadeTimeout = null;

    this.expandFields = { };
    this.fieldNodes = { };
    this.dataTable = '';
    super.onCreate();
  }

  onData(data) {
      this.card.title.text(data._topic_name);
  }
}

URDFViewer.friendlyName = "URDF Viewer";

URDFViewer.supportedTypes = [
    "std_msgs/String",
];

Viewer.registerViewer(URDFViewer);