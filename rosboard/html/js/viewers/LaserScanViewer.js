"use strict";

class LaserScanViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewer = $('<div></div>')
      .css({'font-size': '11pt'
    , "filter": "invert(100%) saturate(50%)"})
      .appendTo(this.card.content);
  }

  onData(msg) {
      this.card.title.text(msg._topic_name);
      
  }
}

LaserScanViewer.supportedTypes = [
    "sensor_msgs/msg/LaserScan",
];

LaserScanViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(LaserScanViewer);