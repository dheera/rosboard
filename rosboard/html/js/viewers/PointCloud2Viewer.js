"use strict";

class PointCloud2Viewer extends Space3DViewer {
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

      if(!msg._data_uint16) {
        this.error("Expected compressed data at msg._data_uint16");
        return;
      }

      let bounds = msg._data_uint16.bounds;
      let points_data = this._base64decode(msg._data_uint16.points);
      let points_view = new DataView(points_data);

      let points = new Float32Array(Math.round(points_data.byteLength / 2));

      let xrange = bounds[1] - bounds[0];
      let xmin = bounds[0];
      let yrange = bounds[3] - bounds[2];
      let ymin = bounds[2];
      let zrange = bounds[5] - bounds[4];
      let zmin = bounds[4];

      for(let i=0; i<points_data.byteLength/6; i++) {
        let offset = i * 6;
        points[3*i] = (points_view.getUint16(offset, true) / 65535) * xrange + xmin;
        points[3*i+1] = (points_view.getUint16(offset+2, true) / 65535) * yrange + ymin;
        points[3*i+2] = (points_view.getUint16(offset+4, true) / 65535) * zrange + zmin;
      }
      
      this.draw([
        {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
        {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
        {type: "points", data: points, zmin: zmin, zmax: zmin + zrange},
      ]);
  }
}

PointCloud2Viewer.friendlyName = "Point cloud (3D)";

PointCloud2Viewer.supportedTypes = [
    "sensor_msgs/msg/PointCloud2",
];

PointCloud2Viewer.maxUpdateRate = 30.0;

Viewer.registerViewer(PointCloud2Viewer);
