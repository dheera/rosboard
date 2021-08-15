"use strict";

class LaserScanViewer extends Space2DViewer {
  _base64decode(base64) {
    var binary_string = window.atob(base64);
    var len = binary_string.length;
    var bytes = new Uint8Array(len);
    for (var i = 0; i < len; i++) {
        bytes[i] = binary_string.charCodeAt(i);
    }
    return bytes.buffer;
  }
  onData(msg) {
    this.card.title.text(msg._topic_name);

    let points = null;

    if(msg.__comp) {
      points = this.decodeCompressed(msg);
    } else {
      points = this.decodeUncompressed(msg);
    }

    this.lastPoints = points;

    // send it to the plotter to display
    let drawObjects = ([
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2}, // unit x axis visualization in red
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2}, // unit y axis visualization in green
      {type: "points", data: points, color: "#e0e0e0"}, // the laserscan points to be plotted
    ]);

    if(this.highlightedPoint) {
      this.onSpace2DClick({x: this.highlightedPoint[0], y: this.highlightedPoint[1]});
      if(this.highlightedPoint) {
        drawObjects.push({
          type: "points",
          data: this.highlightedPoint,
          color: "#ff5000",
        });
        drawObjects.push({
          type: "text",
          text: " (" + this.highlightedPoint[0].toFixed(2) + ", " + this.highlightedPoint[1].toFixed(2) + ")",
          fontSize: 12,
          x: this.highlightedPoint[0],
          y: this.highlightedPoint[1],
          color: "#ff5000",
        });
      }
    }

    this.draw(drawObjects);
  }

  onSpace2DClick({x,y}) {
    if(!this.lastPoints) return;
    let closestx = -1.0;
    let closesty = -1.0;
    let closestDist = 1e12;
    for(let i=0;i<this.lastPoints.length/2;i++) {
      let pointx = this.lastPoints[2*i];
      let pointy = this.lastPoints[2*i+1];
      let dist = (pointx - x)**2 + (pointy - y)**2;
      if(dist < closestDist) {
        closestDist = dist;
        closestx = pointx;
        closesty = pointy;
      }
    }
    if(closestDist < 2.0) {
      this.highlightedPoint = [closestx, closesty];
    } else {
      this.highlightedPoint = null;
    }
  }
  
  decodeCompressed(msg) {
    // pre-allocate an array for xy points that we will feed into the plotter
    // format is [x0, y0, x1, y1, x2, y2, ...]
    // (i suppose JS doesn't have pre-allocated 2D arrays without an insane number of lambda calls (?))

    let rbounds = msg._ranges_uint16.bounds;
    let rdata = this._base64decode(msg._ranges_uint16.points);
    let rview = new DataView(rdata);
    let num_ranges = rdata.byteLength / 2; // uint16

    let points = new Float32Array(num_ranges * 2);

    // angle increment between points
    let angle_incr = (msg.angle_max - msg.angle_min) / num_ranges;

    let rrange = rbounds[1] - rbounds[0];
    let rmin = rbounds[0];

    for(let i=0; i<num_ranges; i++) {
      let offset = i * 2;
      let angle = msg.angle_min + i * angle_incr;

      let r_uint16 = rview.getUint16(offset, true);

      if(r_uint16 === 65535) {
        points[2*i] = NaN;
        points[2*i+1] = NaN;
        continue; // nan, -inf, inf mapped to 65535
      }

      let r = r_uint16 / 65534 * rrange + rmin; // valid values mapped to 0-65534
      
      points[2*i] = r * Math.cos(angle);
      points[2*i+1] = r * Math.sin(angle);
    }

    return points;
  }

  decodeUncompressed(msg) {
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

    return points;
  }
}

LaserScanViewer.friendlyName = "Laser scan";

LaserScanViewer.supportedTypes = [
    "sensor_msgs/msg/LaserScan",
];

LaserScanViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(LaserScanViewer);
