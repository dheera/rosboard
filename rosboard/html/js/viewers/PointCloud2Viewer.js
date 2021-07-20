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

  _getDataGetter(datatype, view) {
    switch(datatype) {
      case PointCloud2Viewer.INT8:
        return view.getInt8.bind(view);
      case PointCloud2Viewer.UINT8:
        return view.getUInt8.bind(view);
      case PointCloud2Viewer.INT16:
        return view.getInt16.bind(view);
      case PointCloud2Viewer.UINT16:
        return view.getUInt16.bind(view);
      case PointCloud2Viewer.INT32:
        return view.getInt32.bind(view);
      case PointCloud2Viewer.UINT32:
        return view.getUInt32.bind(view);
      case PointCloud2Viewer.FLOAT32:
        return view.getFloat32.bind(view);
      case PointCloud2Viewer.FLOAT64:
        return view.getFloat64.bind(view);
      default:
        return (offset, littleEndian) => {return 0.0};
    }
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    let points = null;

    if(msg.__comp) {
      points = this.decodeAndRenderCompressed(msg);
    } else {
      points = this.decodeAndRenderUncompressed(msg);
    }

  }
  
  decodeAndRenderCompressed(msg) {
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

  decodeAndRenderUncompressed(msg) {
    let fields = {};
    let actualRecordSize = 0;

    msg.fields.forEach((field) => {
      fields[field.name] = field;
      if(!field.datatype in PointCloud2Viewer.SIZEOF) {
        this.error("Invalid PointCloud2 message: field " + field + " has invalid datatype = " + String(field.datatype));
        return;
      }
      actualRecordSize += PointCloud2Viewer.SIZEOF[field.datatype];
    });

    if(!("x" in fields) || !("y") in fields) {
      this.error("Cannot display PointCloud2 message: Must have at least 'x' and 'y' fields or I don't know how to display it.");
    }

    let data = this._base64decode(msg.data);

    if(!(msg.point_step * msg.width * msg.height === data.byteLength)) {
      this.error("Invalid PointCloud2: failed assertion: point_step * width * height === data.length");
      return;
    }

    let points = new Float32Array(Math.round(data.byteLength / msg.point_step * 3));
    let view = new DataView(data);
    let littleEndian = !msg.is_bigendian;

    // cache these into variables to avoid hitting hash repeatedly
    let xOffset = fields["x"].offset;
    let xDataGetter = this._getDataGetter(fields["x"].datatype, view);
    let yOffset = fields["y"].offset;
    let yDataGetter = this._getDataGetter(fields["y"].datatype, view);
    let zOffset = -1;
    let zDataGetter = null;
    if("z" in fields) {
      zOffset = fields["z"].offset;
      zDataGetter = this._getDataGetter(fields["z"].datatype, view);
    }
    
    for(let i=0; i<data.byteLength/msg.point_step-1; i++) {
      let offset = i * msg.point_step;
      points[3*i] = xDataGetter(offset + xOffset, littleEndian); // x
      points[3*i+1] = yDataGetter(offset + yOffset, littleEndian); // y
      points[3*i+2] = zDataGetter(offset + zOffset, littleEndian); // y
    }

    this.draw([
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
      {type: "points", data: points, zmin: -2.0, zmax: 2.0},
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

PointCloud2Viewer.friendlyName = "Point cloud (3D)";

PointCloud2Viewer.supportedTypes = [
    "sensor_msgs/msg/PointCloud2",
];

PointCloud2Viewer.maxUpdateRate = 30.0;

Viewer.registerViewer(PointCloud2Viewer);
