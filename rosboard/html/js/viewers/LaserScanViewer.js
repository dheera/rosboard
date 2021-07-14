"use strict";

importJsOnce("js/viewers/Space2DViewer.js");

class LaserScanViewer extends Space2DViewer {
  onData(msg) {
      this.card.title.text(msg._topic_name);

      let angles = [];
      let angle_incr = (msg.angle_max - msg.angle_min) / msg.ranges.length;
      let points = [];
      for(let i = 0; i < msg.ranges.length; i++) {
        let angle = msg.angle_min + i * angle_incr;
        if(-10000.0 < msg.ranges[i] && msg.ranges[i] < 10000.0) {
          points.push([msg.ranges[i] * Math.cos(angle), msg.ranges[i] * Math.sin(angle)]);
        }
      }
      this.draw([
        ["points", points],
      ]);

  }
}

LaserScanViewer.supportedTypes = [
    "sensor_msgs/msg/LaserScan",
];

LaserScanViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(LaserScanViewer);