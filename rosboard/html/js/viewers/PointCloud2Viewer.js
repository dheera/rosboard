"use strict";

importJsOnce("js/viewers/Space2DViewer.js");

class PointCloud2Viewer extends Space2DViewer {
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

      let fields = {};
      let recordSize = 0;
      msg.fields.forEach((field) => {
        fields[field.name] = field;
        recordSize += PointCloud2Viewer.SIZEOF[field.datatype];
      });

      recordSize = 32;

      let data = this._base64decode(msg.data);

      let points = new Float32Array(Math.round(data.byteLength / recordSize * 2));
      let view = new DataView(data);

      for(let i=0;i<data.byteLength/recordSize-1;i++) {
        let offset = i*recordSize;
        points[2*i] = view.getFloat32(offset, true);
        points[2*i+1] = view.getFloat32(offset+4, true);
      }
      // row_step : 1849568
      // point_step: 32
      // is_dense: true
      // is_bigendian: false
      // height: 1
      // width: 57799

      this.draw([
        {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
        {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
        {type: "points", data: points, color: "#e0e0e0"},
      ]);
  }
}


PointCloud2Viewer.INT8 = 1;
PointCloud2Viewer.UINT8 = 2;
PointCloud2Viewer.INT16 = 3;
PointCloud2Viewer.UINT16 = 4;
PointCloud2Viewer.INT32 = 5;
PointCloud2Viewer.UINT32 = 6;
PointCloud2Viewer.FLOAT32 = 7;
PointCloud2Viewer.FLOAT64 = 8;
PointCloud2Viewer.SIZEOF = {
  [PointCloud2Viewer.INT8]: 1,
  [PointCloud2Viewer.UINT8]: 1,
  [PointCloud2Viewer.INT16]: 2,
  [PointCloud2Viewer.UINT16]: 2,
  [PointCloud2Viewer.INT32]: 4,
  [PointCloud2Viewer.UINT32]: 4,
  [PointCloud2Viewer.FLOAT32]: 4,
  [PointCloud2Viewer.FLOAT64]: 8,
}

PointCloud2Viewer.supportedTypes = [
    "sensor_msgs/msg/PointCloud2",
];

PointCloud2Viewer.maxUpdateRate = 10.0;

Viewer.registerViewer(PointCloud2Viewer);
