"use strict";

// IMU Viewer - displays sensor_msgs/Imu data with:
// 1. 3D orientation visualization (quaternion to euler) with ground grid
// 2. Angular velocity time series (separate plot)
// 3. Linear acceleration time series (separate plot)

class ImuViewer extends Viewer {

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

  _radToDeg(rad) {
    return rad * 180 / Math.PI;
  }

  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    // 3D visualization canvas
    this.canvasContainer = $('<div></div>')
      .css({'width': '100%', 'height': '160px', 'position': 'relative'})
      .appendTo(this.viewerNode);

    this.canvas = $('<canvas></canvas>')
      .attr('width', 320)
      .attr('height', 160)
      .css({'width': '100%', 'height': '100%'})
      .appendTo(this.canvasContainer);

    // Compact orientation display
    this.orientationBar = $('<div></div>')
      .css({
        'display': 'flex',
        'justify-content': 'space-around',
        'padding': '8px',
        'background': '#2a2a3e',
        'border-radius': '4px',
        'margin': '8px 0',
        'font-family': 'monospace',
        'font-size': '12px'
      })
      .appendTo(this.viewerNode);

    this.rollField = $('<span></span>').css({'color': '#ff6060'}).appendTo(this.orientationBar);
    this.pitchField = $('<span></span>').css({'color': '#60ff60'}).appendTo(this.orientationBar);
    this.yawField = $('<span></span>').css({'color': '#6080ff'}).appendTo(this.orientationBar);

    // Angular velocity plot
    this.angularLabel = $('<div></div>')
      .css({'font-size': '10px', 'color': '#a0a0a0', 'margin-top': '8px'})
      .text('Angular Velocity (rad/s)')
      .appendTo(this.viewerNode);
    this.angularPlotNode = $('<div></div>').appendTo(this.viewerNode);

    // Linear acceleration plot
    this.linearLabel = $('<div></div>')
      .css({'font-size': '10px', 'color': '#a0a0a0', 'margin-top': '8px'})
      .text('Linear Acceleration (m/s²)')
      .appendTo(this.viewerNode);
    this.linearPlotNode = $('<div></div>').appendTo(this.viewerNode);

    // Initialize ring buffers for time series
    this.size = 200;
    this.angularData = [
      new Array(this.size).fill(0), // time
      new Array(this.size).fill(0), // x
      new Array(this.size).fill(0), // y
      new Array(this.size).fill(0), // z
    ];
    this.linearData = [
      new Array(this.size).fill(0), // time
      new Array(this.size).fill(0), // x
      new Array(this.size).fill(0), // y
      new Array(this.size).fill(0), // z
    ];
    this.ptr = 0;

    // Angular velocity plot options
    let angularOpts = {
      width: 300,
      height: 100,
      legend: { show: true },
      axes: [
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
      ],
      series: [
        {},
        { label: "ωx", stroke: "#ff6060", width: 1 },
        { label: "ωy", stroke: "#60ff60", width: 1 },
        { label: "ωz", stroke: "#6080ff", width: 1 },
      ],
    };

    // Linear acceleration plot options
    let linearOpts = {
      width: 300,
      height: 100,
      legend: { show: true },
      axes: [
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
      ],
      series: [
        {},
        { label: "ax", stroke: "#ff8000", width: 1 },
        { label: "ay", stroke: "#00c080", width: 1 },
        { label: "az", stroke: "#a060ff", width: 1 },
      ],
    };

    this.angularPlot = new uPlot(angularOpts, this.angularData, this.angularPlotNode[0]);
    this.linearPlot = new uPlot(linearOpts, this.linearData, this.linearPlotNode[0]);

    // Update plots periodically
    setInterval(() => {
      this._updatePlots();
    }, 200);

    // Store current euler for 3D rendering
    this.currentEuler = { roll: 0, pitch: 0, yaw: 0 };

    // Start 3D rendering
    this._initCanvas();
    this._startRenderLoop();

    super.onCreate();
  }

  _updatePlots() {
    const getSlicedData = (data) => {
      if (data[0][this.ptr] === 0) {
        return data.map(arr => arr.slice(0, this.ptr));
      } else {
        return data.map(arr =>
          arr.slice(this.ptr, this.size).concat(arr.slice(0, this.ptr))
        );
      }
    };

    const width = this.angularPlotNode[0].clientWidth || 300;

    this.angularPlot.setSize({ width: width, height: 100 });
    this.angularPlot.setData(getSlicedData(this.angularData));

    this.linearPlot.setSize({ width: width, height: 100 });
    this.linearPlot.setData(getSlicedData(this.linearData));
  }

  _initCanvas() {
    this.ctx = this.canvas[0].getContext('2d');
  }

  _startRenderLoop() {
    const render = () => {
      this._render3D();
      requestAnimationFrame(render);
    };
    render();
  }

  _render3D() {
    const canvas = this.canvas[0];
    const ctx = this.ctx;
    const width = canvas.width;
    const height = canvas.height;

    // Clear canvas
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, width, height);

    const cx = width / 2;
    const cy = height / 2;
    const scale = Math.min(width, height) * 0.28;

    const { roll, pitch, yaw } = this.currentEuler;

    // Rotation matrices
    const cosR = Math.cos(roll), sinR = Math.sin(roll);
    const cosP = Math.cos(pitch), sinP = Math.sin(pitch);
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);

    // Combined rotation (ZYX order)
    const rotate = (x, y, z) => {
      // Yaw (Z)
      let x1 = x * cosY - y * sinY;
      let y1 = x * sinY + y * cosY;
      let z1 = z;
      // Pitch (Y)
      let x2 = x1 * cosP + z1 * sinP;
      let y2 = y1;
      let z2 = -x1 * sinP + z1 * cosP;
      // Roll (X)
      let x3 = x2;
      let y3 = y2 * cosR - z2 * sinR;
      let z3 = y2 * sinR + z2 * cosR;
      return [x3, y3, z3];
    };

    // Project 3D to 2D (isometric-like projection)
    const project = (x, y, z) => {
      // Slight tilt for better 3D perception
      const tiltX = 0.15;
      const projY = y * Math.cos(tiltX) - z * Math.sin(tiltX);
      const projZ = y * Math.sin(tiltX) + z * Math.cos(tiltX);
      return [cx + x * scale, cy - projY * scale * 0.8 - projZ * scale * 0.3];
    };

    // Draw ground grid (fixed reference)
    ctx.strokeStyle = '#303050';
    ctx.lineWidth = 0.5;
    const gridSize = 1.5;
    const gridStep = 0.5;
    for (let i = -gridSize; i <= gridSize; i += gridStep) {
      // Lines along X
      const [gx1, gy1] = project(i, -gridSize, 0);
      const [gx2, gy2] = project(i, gridSize, 0);
      ctx.beginPath();
      ctx.moveTo(gx1, gy1);
      ctx.lineTo(gx2, gy2);
      ctx.stroke();
      // Lines along Y
      const [gx3, gy3] = project(-gridSize, i, 0);
      const [gx4, gy4] = project(gridSize, i, 0);
      ctx.beginPath();
      ctx.moveTo(gx3, gy3);
      ctx.lineTo(gx4, gy4);
      ctx.stroke();
    }

    // Draw coordinate axes (rotated with IMU)
    const axes = [
      { start: [0, 0, 0], end: [1.3, 0, 0], color: '#ff4040', label: 'X' },
      { start: [0, 0, 0], end: [0, 1.3, 0], color: '#40ff40', label: 'Y' },
      { start: [0, 0, 0], end: [0, 0, 1.3], color: '#4080ff', label: 'Z' },
    ];

    axes.forEach(axis => {
      const [sx, sy, sz] = rotate(...axis.start);
      const [ex, ey, ez] = rotate(...axis.end);
      const [px1, py1] = project(sx, sy, sz);
      const [px2, py2] = project(ex, ey, ez);

      ctx.beginPath();
      ctx.moveTo(px1, py1);
      ctx.lineTo(px2, py2);
      ctx.strokeStyle = axis.color;
      ctx.lineWidth = 2;
      ctx.stroke();

      // Arrow head
      const arrowLen = 0.15;
      const [ax1, ay1, az1] = rotate(
        axis.end[0] * 0.85 + (axis.end[1] ? 0 : arrowLen * (axis.end[2] ? 0 : 1)),
        axis.end[1] * 0.85 + (axis.end[0] ? 0 : arrowLen * (axis.end[2] ? 0 : 1)),
        axis.end[2] * 0.85
      );

      // Label
      ctx.fillStyle = axis.color;
      ctx.font = 'bold 11px monospace';
      ctx.fillText(axis.label, px2 + 4, py2 - 4);
    });

    // Draw IMU body (elongated box like a circuit board)
    const bx = 0.9, by = 0.5, bz = 0.15;
    const boxVertices = [
      [-bx, -by, -bz], [bx, -by, -bz], [bx, by, -bz], [-bx, by, -bz],
      [-bx, -by, bz], [bx, -by, bz], [bx, by, bz], [-bx, by, bz],
    ];

    const boxFaces = [
      { indices: [0, 1, 2, 3], color: '#404060' }, // bottom
      { indices: [4, 5, 6, 7], color: '#505080' }, // top
      { indices: [0, 1, 5, 4], color: '#454565' }, // front
      { indices: [2, 3, 7, 6], color: '#454565' }, // back
      { indices: [0, 3, 7, 4], color: '#3a3a5a' }, // left
      { indices: [1, 2, 6, 5], color: '#3a3a5a' }, // right
    ];

    // Transform and project vertices
    const transformedVertices = boxVertices.map(v => rotate(...v));
    const projectedVertices = transformedVertices.map(v => project(...v));

    // Calculate face depths for sorting (painter's algorithm)
    const facesWithDepth = boxFaces.map((face, i) => {
      const avgZ = face.indices.reduce((sum, idx) => sum + transformedVertices[idx][2], 0) / 4;
      return { ...face, depth: avgZ, index: i };
    });
    facesWithDepth.sort((a, b) => a.depth - b.depth);

    // Draw faces back to front
    facesWithDepth.forEach(face => {
      ctx.beginPath();
      ctx.moveTo(projectedVertices[face.indices[0]][0], projectedVertices[face.indices[0]][1]);
      for (let i = 1; i < face.indices.length; i++) {
        ctx.lineTo(projectedVertices[face.indices[i]][0], projectedVertices[face.indices[i]][1]);
      }
      ctx.closePath();
      ctx.fillStyle = face.color;
      ctx.fill();
      ctx.strokeStyle = '#606080';
      ctx.lineWidth = 1;
      ctx.stroke();
    });

    // Draw forward direction indicator (arrow on top face)
    const arrowStart = rotate(0.3, 0, bz + 0.02);
    const arrowEnd = rotate(bx + 0.25, 0, bz + 0.02);
    const arrowLeft = rotate(bx - 0.1, 0.12, bz + 0.02);
    const arrowRight = rotate(bx - 0.1, -0.12, bz + 0.02);

    const [asx, asy] = project(...arrowStart);
    const [aex, aey] = project(...arrowEnd);
    const [alx, aly] = project(...arrowLeft);
    const [arx, ary] = project(...arrowRight);

    ctx.beginPath();
    ctx.moveTo(asx, asy);
    ctx.lineTo(aex, aey);
    ctx.strokeStyle = '#ff8000';
    ctx.lineWidth = 3;
    ctx.stroke();

    // Arrow head
    ctx.beginPath();
    ctx.moveTo(aex, aey);
    ctx.lineTo(alx, aly);
    ctx.moveTo(aex, aey);
    ctx.lineTo(arx, ary);
    ctx.stroke();

    // Display euler angles in corner
    ctx.fillStyle = '#ffffff';
    ctx.font = '11px monospace';
    ctx.fillText(`R: ${this._radToDeg(roll).toFixed(1)}°`, 8, 16);
    ctx.fillText(`P: ${this._radToDeg(pitch).toFixed(1)}°`, 8, 30);
    ctx.fillText(`Y: ${this._radToDeg(yaw).toFixed(1)}°`, 8, 44);
  }

  onData(msg) {
    this.card.title.text(_rabo_topic_names[msg._topic_name] || msg._topic_name);

    // Parse orientation
    const q = msg.orientation || { x: 0, y: 0, z: 0, w: 1 };
    const euler = this._quatToEuler(q);
    this.currentEuler = euler;

    // Parse angular velocity
    const av = msg.angular_velocity || { x: 0, y: 0, z: 0 };

    // Parse linear acceleration
    const la = msg.linear_acceleration || { x: 0, y: 0, z: 0 };

    // Update orientation bar
    this.rollField.text(`Roll: ${this._radToDeg(euler.roll).toFixed(1)}°`);
    this.pitchField.text(`Pitch: ${this._radToDeg(euler.pitch).toFixed(1)}°`);
    this.yawField.text(`Yaw: ${this._radToDeg(euler.yaw).toFixed(1)}°`);

    // Update time series data
    const time = Math.floor(Date.now() / 10) / 100;

    this.angularData[0][this.ptr] = time;
    this.angularData[1][this.ptr] = av.x;
    this.angularData[2][this.ptr] = av.y;
    this.angularData[3][this.ptr] = av.z;

    this.linearData[0][this.ptr] = time;
    this.linearData[1][this.ptr] = la.x;
    this.linearData[2][this.ptr] = la.y;
    this.linearData[3][this.ptr] = la.z;

    this.ptr = (this.ptr + 1) % this.size;
  }
}

ImuViewer.friendlyName = "IMU";

ImuViewer.supportedTypes = [
    "sensor_msgs/msg/Imu",
];

ImuViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(ImuViewer);
