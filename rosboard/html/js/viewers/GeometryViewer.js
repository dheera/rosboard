"use strict";

class GeometryViewer extends Space2DViewer {
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

    if(msg._topic_type.endsWith("/Pose")) { this.processPose(msg); return; }
    if(msg._topic_type.endsWith("/PoseStamped")) { this.processPoseStamped(msg); return; }
    if(msg._topic_type.endsWith("/PoseWithCovariance")) { this.processPoseWithCovariance(msg); return; }
    if(msg._topic_type.endsWith("/PoseWithCovarianceStamped")) { this.processPoseWithCovarianceStamped(msg); return; }
    if(msg._topic_type.endsWith("/Point")) { this.processPoint(msg); return; }
    if(msg._topic_type.endsWith("/Point32")) { this.processPoint(msg); return; }
    if(msg._topic_type.endsWith("/PointStamped")) { this.processPointStamped(msg); return; }
    if(msg._topic_type.endsWith("/Odometry")) { this.processOdometry(msg); return; }
  }

  processPose(msg) {
    let x = msg.position.x;
    let y = msg.position.y;
    let angles = this._quatToEuler(msg.orientation);
    this.renderTrackedPoseView({x: x, y: y, yaw: angles.yaw});
  }

  processPoseStamped(msg) {
    let x = msg.pose.position.x;
    let y = msg.pose.position.y;
    let angles = this._quatToEuler(msg.pose.orientation);
    this.renderTrackedPoseView({x: x, y: y, yaw: angles.yaw});
  }

  processPoseWithCovariance(msg) {
    let x = msg.pose.position.x;
    let y = msg.pose.position.y;
    let angles = this._quatToEuler(msg.pose.orientation);
    this.renderTrackedPoseView({x: x, y: y, yaw: angles.yaw});
  }

  processPoseWithCovarianceStamped(msg) {
    let x = msg.pose.pose.position.x;
    let y = msg.pose.pose.position.y;
    let angles = this._quatToEuler(msg.pose.pose.orientation);
    this.renderTrackedPoseView({x: x, y: y, yaw: angles.yaw});
  }

  processPoint(msg) {
    let x = msg.x;
    let y = msg.y;
    this.renderTrackedPoseView({x: x, y: y, yaw: null});
  }

  processPointStamped(msg) {
    let x = msg.point.x;
    let y = msg.point.y;
    this.renderTrackedPoseView({x:x, y:y, yaw: null});
  }

  processOdometry(msg) {
    let x = msg.pose.pose.position.x;
    let y = msg.pose.pose.position.y;
    let angles = this._quatToEuler(msg.pose.pose.orientation);
    this.renderTrackedPoseView({x: x, y: y, yaw: angles.yaw});
  }
  
  renderTrackedPoseView({x, y, yaw}) {

    if(!this.points) this.points = new Array(100).fill(NaN);
    if(!this.ptr) this.ptr = 0;

    this.points[this.ptr] = x;
    this.points[this.ptr+1] = y;
    this.ptr += 2;
    this.ptr = this.ptr % 1000;

    let pointsSlice = this.points.slice(this.ptr, 1000).concat(this.points.slice(0, this.ptr));

    // send it to the plotter to display
    let drawObjects = ([
      // x axis
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2}, // unit x axis visualization in red
      // y axis
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2}, // unit y axis visualization in green
      // the current point
      {type: "points", data: [x, y], color: "#ff5000"},
      // history path
      {type: "path", data: pointsSlice, color: "#808080"},
    ])
    if(yaw !== null) {
      drawObjects = drawObjects.concat([
        // arrow stem
        {type: "path", data: [
          x, 
          y, 
          x + 2*Math.cos(yaw), 
          y + 2*Math.sin(yaw),
        ], color: "#ff5000", lineWidth: 1},
        // arrow head
        {type: "path", data: [
          x + 2*Math.cos(yaw) + 0.5*Math.cos(13*Math.PI/12+yaw),
          y + 2*Math.sin(yaw) + 0.5*Math.sin(13*Math.PI/12+yaw), 
          x + 2*Math.cos(yaw), 
          y + 2*Math.sin(yaw),
          x + 2*Math.cos(yaw) + 0.5*Math.cos(-13*Math.PI/12+yaw),
          y + 2*Math.sin(yaw) + 0.5*Math.sin(-13*Math.PI/12+yaw), 
        ], color: "#ff5000", lineWidth: 1}
      ]);
    }
    this.setDefaultView({xcenter: x, ycenter: y, scale: 40.0});
    this.draw(drawObjects);
  }
}

GeometryViewer.friendlyName = "2D view";

GeometryViewer.supportedTypes = [
    "geometry_msgs/msg/Pose",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovariance",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    "geometry_msgs/msg/Point",
    "geometry_msgs/msg/Point32",
    "geometry_msgs/msg/PointStamped",
    "nav_msgs/msg/Odometry",
];

GeometryViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(GeometryViewer);
