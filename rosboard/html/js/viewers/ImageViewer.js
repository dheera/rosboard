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

    this.img = $('<img></img>')
      .css({"width": "100%"})
      .appendTo(this.viewerNode);

    let that = this;

    this.img[0].addEventListener('pointermove', function(e) {
      if(!that.lastMsg) return;
      if(!that.img[0].clientWidth || !that.img[0].clientHeight) return;

      let width = that.img[0].naturalWidth;
      let height = that.img[0].naturalHeight;
      if(that.lastMsg._data_shape) {
        height = that.lastMsg._data_shape[0];
        width = that.lastMsg._data_shape[1];
      }
      let x = e.offsetX / that.img[0].clientWidth * width;
      let y = e.offsetY / that.img[0].clientHeight * height;
      x = Math.min(Math.max(x, 0), width);
      y = Math.min(Math.max(y, 0), height);
      that.tip("(" + x.toFixed(0) + ", " + y.toFixed(0) + ")");
    });

    this.img[0].addEventListener('pointerdown', function(e) {
      if(!that.lastMsg) return;
      if(!that.img[0].clientWidth || !that.img[0].clientHeight) return;

      let width = that.img[0].naturalWidth;
      let height = that.img[0].naturalHeight;
      if(that.lastMsg._data_shape) {
        height = that.lastMsg._data_shape[0];
        width = that.lastMsg._data_shape[1];
      }
      let x = e.offsetX / that.img[0].clientWidth * width;
      let y = e.offsetY / that.img[0].clientHeight * height;
      console.log("clicked at " + x + ", " + y);
    });

    this.lastMsg = null;

    super.onCreate();
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    if(msg.__comp) {
      this.decodeAndRenderCompressed(msg);
    } else {
      this.decodeAndRenderUncompressed(msg);
    }
  }
  
  decodeAndRenderCompressed(msg) {
    this.img[0].src = "data:image/jpeg;base64," + msg._data_jpeg;
    this.lastMsg = msg;
  }

  decodeAndRenderUncompressed(msg) {
    this.error("Support for uncompressed images not yet implemented.");
  }
}

ImageViewer.friendlyName = "Image";

ImageViewer.supportedTypes = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "nav_msgs/msg/OccupancyGrid",
];

ImageViewer.maxUpdateRate = 24.0;

Viewer.registerViewer(ImageViewer);