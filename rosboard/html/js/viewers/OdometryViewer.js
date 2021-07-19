"use strict";

importJsOnce("js/viewers/Space2DViewer.js");

class OdometryViewer extends Space2DViewer {
  _quatToEuler(q) {
    let euler = {};

    // roll (x-axis rotation)
    let sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    let cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.roll = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    let sinp = 2 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1)
        euler.pitch = sinp > 0 ? (Math.PI/2) : (-Math.PI/2);
    else
        euler.pitch = Math.asin(sinp);

    // yaw (z-axis rotation)
    let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.yaw = Math.atan2(siny_cosp, cosy_cosp);

    return euler;
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    if(!this.points) this.points = new Array(100).fill(NaN);

    let x = msg.pose.pose.position.x;
    let y = msg.pose.pose.position.y;
    let angles = this._quatToEuler(msg.pose.pose.orientation);

    if(!this.ptr) this.ptr = 0;

    this.points[this.ptr] = x;
    this.points[this.ptr+1] = y;
    this.ptr += 2;
    this.ptr = this.ptr % 1000;

    let pointsSlice = this.points.slice(this.ptr, 1000).concat(this.points.slice(0, this.ptr));

    // send it to the plotter to display
    let drawObjects = ([
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2}, // unit x axis visualization in red
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2}, // unit y axis visualization in green
      // {type: "points", data: points, color: "#e0e0e0"}, // the Odometry points to be plotted
      {type: "points", data: [x, y], color: "#ff5000"},
      {type: "path", data: pointsSlice, color: "#808080"},
      // {type: "path", data: [x + 2*Math.cos(angles.yaw+Math.PI/8), y + 2*Math.sin(angles.yaw+Math.PI/8), x, y, x + 2*Math.cos(angles.yaw-Math.PI/8), y + 2*Math.sin(angles.yaw-Math.PI/8), ], color: "#ffffff"},
      {type: "path", data: [
        x, 
        y, 
        x + 2*Math.cos(angles.yaw), 
        y + 2*Math.sin(angles.yaw),
      ], color: "#ff5000", lineWidth: 1},
      {type: "path", data: [
        x + 2*Math.cos(angles.yaw) + 0.5*Math.cos(13*Math.PI/12+angles.yaw),
        y + 2*Math.sin(angles.yaw) + 0.5*Math.sin(13*Math.PI/12+angles.yaw), 
        x + 2*Math.cos(angles.yaw), 
        y + 2*Math.sin(angles.yaw),
        x + 2*Math.cos(angles.yaw) + 0.5*Math.cos(-13*Math.PI/12+angles.yaw),
        y + 2*Math.sin(angles.yaw) + 0.5*Math.sin(-13*Math.PI/12+angles.yaw), 
      ], color: "#ff5000", lineWidth: 1},
    ]);

    this.draw(drawObjects);
  }
}

OdometryViewer.friendlyName = "Odometry (2D)";

OdometryViewer.supportedTypes = [
    "nav_msgs/msg/Odometry",
];

OdometryViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(OdometryViewer);
