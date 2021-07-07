"use strict";

class ImageViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.imgNode = $('<img></img>')
      .css({"width": "100%"})
      .appendTo(this.viewerNode);

    super.onCreate();
  }

  onData(data) {
      this.card.title.text(data._topic_name);

      if(data._img_jpeg) {
          this.imgNode[0].src = "data:image/jpeg;base64," + data._img_jpeg;
      }
  }
}

ImageViewer.supportedTypes = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
];
