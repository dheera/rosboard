"use strict";

importJsOnce("js/viewers/Space2DViewer.js");

class LaserScanViewer extends Space2DViewer {
  onData(msg) {
      this.card.title.text(msg._topic_name);

      // angle increment between points
      let angle_incr = (msg.angle_max - msg.angle_min) / msg.ranges.length;

      // pre-allocate an array for xy points that we will feed into the plotter
      // format is [x0, y0, x1, y1, x2, y2, ...]
      // (i suppose JS doesn't have pre-allocated 2D arrays without an insane number of lambda calls (?))
      let points = new Float32Array(msg.ranges.length * 2);

      // convert all the angles/ranges to x,y and fill in the points array
      for(let i = 0; i < msg.ranges.length; i++) {
        let angle = msg.angle_min + i * angle_incr;
        if(-10000.0 < msg.ranges[i] && msg.ranges[i] < 10000.0) {
          points[2*i] = msg.ranges[i] * Math.cos(angle);
          points[2*i+1] = msg.ranges[i] * Math.sin(angle);
        } else {
          points[2*i] = NaN;
          points[2*i+1] = NaN;
        }
      }

      // send it to the plotter to display
      this.draw([
        {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2}, // unit x axis visualization in red
        {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2}, // unit y axis visualization in green
        {type: "points", data: points, color: "#e0e0e0"}, // the laserscan points to be plotted
      ]);
  }
}

LaserScanViewer.supportedTypes = [
    "sensor_msgs/msg/LaserScan",
];

LaserScanViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(LaserScanViewer);
