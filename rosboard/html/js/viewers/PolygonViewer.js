"use strict";

class PolygonViewer extends Space2DViewer {
  onData(msg) {
      this.card.title.text(msg._topic_name);

      let polygon = msg;
      if(msg._topic_type.endsWith("Stamped")) {
        polygon = msg.polygon;
      }

      let points = new Float32Array((polygon.points.length + 1) * 2);
      
      for(let i = 0; i < polygon.points.length; i++) {
        points[2*i] = polygon.points[i].x;
        points[2*i+1] = polygon.points[i].y;
      }
      points[2*polygon.points.length] = polygon.points[0].x;
      points[2*polygon.points.length+1] = polygon.points[0].y;
      this.draw([
        {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
        {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
        {type: "path", data: points, color: "#e0e0e0"},
      ]);
  }
}

PolygonViewer.friendlyName = "Polygon";

PolygonViewer.supportedTypes = [
    "geometry_msgs/msg/Polygon",
    "geometry_msgs/msg/PolygonStamped",
];

PolygonViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(PolygonViewer);
