"use strict";

// IMU Viewer - displays sensor_msgs/Imu data with:
// 1. 3D orientation visualization (quaternion to euler)
// 2. Angular velocity time series
// 3. Linear acceleration time series

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
      .css({'width': '100%', 'height': '150px', 'position': 'relative'})
      .appendTo(this.viewerNode);

    this.canvas = $('<canvas></canvas>')
      .attr('width', 300)
      .attr('height', 150)
      .css({'width': '100%', 'height': '100%'})
      .appendTo(this.canvasContainer);

    // Data tables
    this.dataTable = $('<table></table>')
      .addClass('mdl-data-table')
      .addClass('mdl-js-data-table')
      .css({'width': '100%', 'table-layout': 'fixed', 'margin-top': '10px'})
      .appendTo(this.viewerNode);

    // Orientation row (Euler angles)
    let trOrientation = $('<tr></tr>').appendTo(this.dataTable);
    $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .text('Orientation (RPY)')
      .css({'width': '40%', 'font-weight': 'bold'})
      .appendTo(trOrientation);
    this.orientationField = $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .addClass('monospace')
      .appendTo(trOrientation);

    // Angular velocity row
    let trAngular = $('<tr></tr>').appendTo(this.dataTable);
    $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .text('Angular Vel (rad/s)')
      .css({'width': '40%', 'font-weight': 'bold'})
      .appendTo(trAngular);
    this.angularField = $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .addClass('monospace')
      .appendTo(trAngular);

    // Linear acceleration row
    let trLinear = $('<tr></tr>').appendTo(this.dataTable);
    $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .text('Linear Accel (m/s²)')
      .css({'width': '40%', 'font-weight': 'bold'})
      .appendTo(trLinear);
    this.linearField = $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .addClass('monospace')
      .appendTo(trLinear);

    // Time series plot for angular velocity
    this.plotContainer = $('<div></div>')
      .css({'margin-top': '10px'})
      .appendTo(this.viewerNode);

    this.plotNode = $('<div></div>').appendTo(this.plotContainer);

    // Initialize ring buffers for time series
    this.size = 200;
    this.data = [
      new Array(this.size).fill(0), // time
      new Array(this.size).fill(0), // angular x
      new Array(this.size).fill(0), // angular y
      new Array(this.size).fill(0), // angular z
      new Array(this.size).fill(0), // linear x
      new Array(this.size).fill(0), // linear y
      new Array(this.size).fill(0), // linear z
    ];
    this.ptr = 0;

    let opts = {
      width: 300,
      height: 150,
      legend: {
        show: true,
      },
      axes: [
        {
          stroke: "#a0a0a0",
          ticks: { stroke: "#404040" },
          grid: { stroke: "#404040" },
        },
        {
          stroke: "#a0a0a0",
          ticks: { stroke: "#404040" },
          grid: { stroke: "#404040" },
        },
      ],
      series: [
        {},
        { label: "ωx", stroke: "#ff6060", width: 1 },
        { label: "ωy", stroke: "#60ff60", width: 1 },
        { label: "ωz", stroke: "#6060ff", width: 1 },
        { label: "ax", stroke: "#ff8000", width: 1 },
        { label: "ay", stroke: "#00ff80", width: 1 },
        { label: "az", stroke: "#8000ff", width: 1 },
      ],
    };

    this.uplot = new uPlot(opts, this.data, this.plotNode[0]);

    // Update plot periodically
    setInterval(() => {
      let data = [];
      if (this.data[0][this.ptr] === 0) {
        data = this.data.map(arr => arr.slice(0, this.ptr));
      } else {
        data = this.data.map(arr =>
          arr.slice(this.ptr, this.size).concat(arr.slice(0, this.ptr))
        );
      }
      this.uplot.setSize({width: this.plotNode[0].clientWidth || 300, height: 150});
      this.uplot.setData(data);
    }, 200);

    // Store current euler for 3D rendering
    this.currentEuler = { roll: 0, pitch: 0, yaw: 0 };

    // Start 3D rendering
    this._initCanvas();
    this._startRenderLoop();

    super.onCreate();
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
    const scale = Math.min(width, height) * 0.3;

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

    // Project 3D to 2D (simple orthographic)
    const project = (x, y, z) => {
      return [cx + x * scale, cy - y * scale];
    };

    // Draw coordinate axes
    const axes = [
      { start: [0, 0, 0], end: [1.2, 0, 0], color: '#ff4040', label: 'X' },
      { start: [0, 0, 0], end: [0, 1.2, 0], color: '#40ff40', label: 'Y' },
      { start: [0, 0, 0], end: [0, 0, 1.2], color: '#4040ff', label: 'Z' },
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

      // Label
      ctx.fillStyle = axis.color;
      ctx.font = '12px monospace';
      ctx.fillText(axis.label, px2 + 5, py2);
    });

    // Draw a simple 3D box to show orientation
    const boxSize = 0.8;
    const boxVertices = [
      [-boxSize, -boxSize/2, -boxSize/3],
      [boxSize, -boxSize/2, -boxSize/3],
      [boxSize, boxSize/2, -boxSize/3],
      [-boxSize, boxSize/2, -boxSize/3],
      [-boxSize, -boxSize/2, boxSize/3],
      [boxSize, -boxSize/2, boxSize/3],
      [boxSize, boxSize/2, boxSize/3],
      [-boxSize, boxSize/2, boxSize/3],
    ];

    const boxEdges = [
      [0, 1], [1, 2], [2, 3], [3, 0], // bottom
      [4, 5], [5, 6], [6, 7], [7, 4], // top
      [0, 4], [1, 5], [2, 6], [3, 7], // sides
    ];

    // Transform vertices
    const transformedVertices = boxVertices.map(v => rotate(...v));
    const projectedVertices = transformedVertices.map(v => project(...v));

    // Draw edges
    ctx.strokeStyle = '#808080';
    ctx.lineWidth = 1;
    boxEdges.forEach(edge => {
      const [i1, i2] = edge;
      ctx.beginPath();
      ctx.moveTo(projectedVertices[i1][0], projectedVertices[i1][1]);
      ctx.lineTo(projectedVertices[i2][0], projectedVertices[i2][1]);
      ctx.stroke();
    });

    // Draw front face indicator (arrow pointing forward)
    const frontArrow = rotate(boxSize + 0.3, 0, 0);
    const frontBase = rotate(boxSize, 0, 0);
    const [fax, fay] = project(...frontArrow);
    const [fbx, fby] = project(...frontBase);

    ctx.beginPath();
    ctx.moveTo(fbx, fby);
    ctx.lineTo(fax, fay);
    ctx.strokeStyle = '#ff8000';
    ctx.lineWidth = 3;
    ctx.stroke();

    // Display euler angles text
    ctx.fillStyle = '#ffffff';
    ctx.font = '11px monospace';
    ctx.fillText(`R: ${this._radToDeg(roll).toFixed(1)}°`, 10, 20);
    ctx.fillText(`P: ${this._radToDeg(pitch).toFixed(1)}°`, 10, 35);
    ctx.fillText(`Y: ${this._radToDeg(yaw).toFixed(1)}°`, 10, 50);
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

    // Update display fields
    this.orientationField.text(
      `R:${this._radToDeg(euler.roll).toFixed(1)}° ` +
      `P:${this._radToDeg(euler.pitch).toFixed(1)}° ` +
      `Y:${this._radToDeg(euler.yaw).toFixed(1)}°`
    );

    this.angularField.text(
      `x:${av.x.toFixed(3)} y:${av.y.toFixed(3)} z:${av.z.toFixed(3)}`
    );

    this.linearField.text(
      `x:${la.x.toFixed(2)} y:${la.y.toFixed(2)} z:${la.z.toFixed(2)}`
    );

    // Update time series data
    const time = Math.floor(Date.now() / 10) / 100;
    this.data[0][this.ptr] = time;
    this.data[1][this.ptr] = av.x;
    this.data[2][this.ptr] = av.y;
    this.data[3][this.ptr] = av.z;
    this.data[4][this.ptr] = la.x;
    this.data[5][this.ptr] = la.y;
    this.data[6][this.ptr] = la.z;
    this.ptr = (this.ptr + 1) % this.size;
  }
}

ImuViewer.friendlyName = "IMU";

ImuViewer.supportedTypes = [
    "sensor_msgs/msg/Imu",
];

ImuViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(ImuViewer);
