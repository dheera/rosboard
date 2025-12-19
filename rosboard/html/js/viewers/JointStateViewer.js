"use strict";

// JointState Viewer - displays sensor_msgs/JointState data with:
// 1. Joint position gauges (circular or linear)
// 2. Velocity bar indicators
// 3. Effort (torque) display
// 4. Time series plot for selected joint

class JointStateViewer extends Viewer {

  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    // Joint gauges container
    this.gaugesContainer = $('<div></div>')
      .css({
        'display': 'flex',
        'flex-wrap': 'wrap',
        'justify-content': 'center',
        'gap': '8px',
        'padding': '8px'
      })
      .appendTo(this.viewerNode);

    // Data table for detailed values
    this.tableContainer = $('<div></div>')
      .css({
        'max-height': '150px',
        'overflow-y': 'auto',
        'margin-top': '8px'
      })
      .appendTo(this.viewerNode);

    this.dataTable = $('<table></table>')
      .addClass('mdl-data-table')
      .addClass('mdl-js-data-table')
      .css({'width': '100%', 'font-size': '10px'})
      .appendTo(this.tableContainer);

    // Table header
    let thead = $('<thead></thead>').appendTo(this.dataTable);
    let headerRow = $('<tr></tr>').appendTo(thead);
    $('<th></th>').addClass('mdl-data-table__cell--non-numeric').text('Joint').css({'padding': '4px 8px'}).appendTo(headerRow);
    $('<th></th>').text('Position').css({'padding': '4px 8px'}).appendTo(headerRow);
    $('<th></th>').text('Velocity').css({'padding': '4px 8px'}).appendTo(headerRow);
    $('<th></th>').text('Effort').css({'padding': '4px 8px'}).appendTo(headerRow);

    this.tableBody = $('<tbody></tbody>').appendTo(this.dataTable);

    // Store joint data
    this.joints = {};
    this.jointGauges = {};

    super.onCreate();
  }

  _createGauge(name, size = 'normal') {
    // Size presets: 'large' for 1-2 joints, 'normal' for 3-6, 'small' for 7+
    const sizes = {
      large: { container: 140, canvas: 120 },
      normal: { container: 80, canvas: 70 },
      small: { container: 65, canvas: 55 }
    };
    const s = sizes[size] || sizes.normal;

    const gaugeContainer = $('<div></div>')
      .addClass('joint-gauge')
      .css({
        'width': s.container + 'px',
        'text-align': 'center',
        'background': '#2a2a3e',
        'border-radius': '8px',
        'padding': '8px 4px'
      });

    const canvas = $('<canvas></canvas>')
      .attr('width', s.canvas)
      .attr('height', s.canvas)
      .appendTo(gaugeContainer);

    // Joint name (truncated)
    const displayName = name.length > 10 ? name.substring(0, 9) + '...' : name;
    const label = $('<div></div>')
      .css({
        'font-size': '9px',
        'color': '#a0a0a0',
        'margin-top': '4px',
        'overflow': 'hidden',
        'text-overflow': 'ellipsis',
        'white-space': 'nowrap'
      })
      .text(displayName)
      .attr('title', name)
      .appendTo(gaugeContainer);

    const valueLabel = $('<div></div>')
      .css({
        'font-size': '11px',
        'color': '#ffffff',
        'font-family': 'monospace'
      })
      .text('0.00')
      .appendTo(gaugeContainer);

    return {
      container: gaugeContainer,
      canvas: canvas[0],
      ctx: canvas[0].getContext('2d'),
      valueLabel: valueLabel,
      position: 0,
      velocity: 0,
      effort: 0
    };
  }

  _renderGauge(gauge) {
    const ctx = gauge.ctx;
    const width = gauge.canvas.width;
    const height = gauge.canvas.height;
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.min(width, height) * 0.4;

    // Clear
    ctx.fillStyle = '#2a2a3e';
    ctx.fillRect(0, 0, width, height);

    // Background arc
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0.75 * Math.PI, 0.25 * Math.PI);
    ctx.strokeStyle = '#404060';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.stroke();

    // Normalize position to -PI to PI range for display
    let normalizedPos = gauge.position;
    while (normalizedPos > Math.PI) normalizedPos -= 2 * Math.PI;
    while (normalizedPos < -Math.PI) normalizedPos += 2 * Math.PI;

    // Position arc (map -PI..PI to 0.75PI..0.25PI)
    const startAngle = 0.75 * Math.PI;
    const endAngle = 0.25 * Math.PI;
    const totalArc = (2 * Math.PI) - (startAngle - endAngle);
    const posRatio = (normalizedPos + Math.PI) / (2 * Math.PI);
    const posAngle = startAngle + posRatio * totalArc;

    // Color based on position
    const hue = 200 - Math.abs(normalizedPos) * 60; // Blue to orange
    ctx.beginPath();
    ctx.arc(cx, cy, radius, startAngle, posAngle);
    ctx.strokeStyle = `hsl(${hue}, 70%, 55%)`;
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.stroke();

    // Velocity indicator (small bar at bottom)
    const velBarWidth = 50;
    const velBarHeight = 4;
    const velBarX = cx - velBarWidth / 2;
    const velBarY = height - 8;

    // Velocity background
    ctx.fillStyle = '#404060';
    ctx.fillRect(velBarX, velBarY, velBarWidth, velBarHeight);

    // Velocity bar (clamp to -5..5 rad/s range)
    const velNorm = Math.max(-1, Math.min(1, gauge.velocity / 5));
    const velWidth = Math.abs(velNorm) * (velBarWidth / 2);
    const velColor = velNorm >= 0 ? '#40c080' : '#c08040';
    ctx.fillStyle = velColor;
    if (velNorm >= 0) {
      ctx.fillRect(cx, velBarY, velWidth, velBarHeight);
    } else {
      ctx.fillRect(cx - velWidth, velBarY, velWidth, velBarHeight);
    }

    // Center line for velocity
    ctx.fillStyle = '#808080';
    ctx.fillRect(cx - 0.5, velBarY - 1, 1, velBarHeight + 2);

    // Position needle
    const needleLen = radius - 8;
    const needleAngle = posAngle;
    const nx = cx + Math.cos(needleAngle) * needleLen;
    const ny = cy + Math.sin(needleAngle) * needleLen;

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(nx, ny);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // Update value label
    const degrees = (normalizedPos * 180 / Math.PI).toFixed(1);
    gauge.valueLabel.text(`${degrees}°`);
  }

  _updateTable(msg) {
    this.tableBody.empty();

    const names = msg.name || [];
    const positions = msg.position || [];
    const velocities = msg.velocity || [];
    const efforts = msg.effort || [];

    for (let i = 0; i < names.length; i++) {
      const tr = $('<tr></tr>').appendTo(this.tableBody);

      // Joint name
      $('<td></td>')
        .addClass('mdl-data-table__cell--non-numeric')
        .css({'padding': '4px 8px', 'max-width': '100px', 'overflow': 'hidden', 'text-overflow': 'ellipsis'})
        .text(names[i])
        .attr('title', names[i])
        .appendTo(tr);

      // Position (radians and degrees)
      const pos = positions[i] !== undefined ? positions[i] : 0;
      const posDeg = (pos * 180 / Math.PI).toFixed(2);
      $('<td></td>')
        .addClass('monospace')
        .css({'padding': '4px 8px', 'font-size': '10px'})
        .text(`${pos.toFixed(3)} (${posDeg}°)`)
        .appendTo(tr);

      // Velocity
      const vel = velocities[i] !== undefined ? velocities[i] : 0;
      $('<td></td>')
        .addClass('monospace')
        .css({'padding': '4px 8px', 'font-size': '10px'})
        .text(vel.toFixed(3))
        .appendTo(tr);

      // Effort
      const eff = efforts[i] !== undefined ? efforts[i] : 0;
      $('<td></td>')
        .addClass('monospace')
        .css({'padding': '4px 8px', 'font-size': '10px'})
        .text(eff.toFixed(2))
        .appendTo(tr);
    }
  }

  _getGaugeSize(jointCount) {
    if (jointCount <= 2) return 'large';
    if (jointCount <= 6) return 'normal';
    return 'small';
  }

  onData(msg) {
    this.card.title.text(_rabo_topic_names[msg._topic_name] || msg._topic_name);

    const names = msg.name || [];
    const positions = msg.position || [];
    const velocities = msg.velocity || [];
    const efforts = msg.effort || [];

    // Determine gauge size based on joint count
    const newSize = this._getGaugeSize(names.length);

    // Check if we need to rebuild gauges (size changed or joints changed)
    const existingNames = Object.keys(this.jointGauges);
    const sizeChanged = this.currentGaugeSize !== newSize;
    const jointsChanged = existingNames.length !== names.length ||
      !names.every(n => this.jointGauges[n]);

    if (sizeChanged || jointsChanged) {
      // Clear all existing gauges
      this.gaugesContainer.empty();
      this.jointGauges = {};
      this.currentGaugeSize = newSize;

      // Create new gauges with correct size
      for (let i = 0; i < names.length; i++) {
        const name = names[i];
        this.jointGauges[name] = this._createGauge(name, newSize);
        this.gaugesContainer.append(this.jointGauges[name].container);
      }
    }

    // Update gauge data and render
    for (let i = 0; i < names.length; i++) {
      const name = names[i];
      const gauge = this.jointGauges[name];
      if (gauge) {
        gauge.position = positions[i] !== undefined ? positions[i] : 0;
        gauge.velocity = velocities[i] !== undefined ? velocities[i] : 0;
        gauge.effort = efforts[i] !== undefined ? efforts[i] : 0;
        this._renderGauge(gauge);
      }
    }

    // Update table
    this._updateTable(msg);
  }
}

JointStateViewer.friendlyName = "Joint State";

JointStateViewer.supportedTypes = [
    "sensor_msgs/msg/JointState",
];

JointStateViewer.maxUpdateRate = 20.0;

Viewer.registerViewer(JointStateViewer);
