"use strict";

// JointState Viewer - displays sensor_msgs/JointState data with:
// 1. Time series plot for joint positions
// 2. Data table with position, velocity, effort

class JointStateViewer extends Viewer {

  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    // Position plot
    this.positionLabel = $('<div></div>')
      .css({'font-size': '10px', 'color': '#a0a0a0', 'margin-bottom': '4px'})
      .text('Position (rad)')
      .appendTo(this.viewerNode);
    this.positionPlotNode = $('<div></div>').appendTo(this.viewerNode);

    // Velocity plot
    this.velocityLabel = $('<div></div>')
      .css({'font-size': '10px', 'color': '#a0a0a0', 'margin-top': '8px', 'margin-bottom': '4px'})
      .text('Velocity (rad/s)')
      .appendTo(this.viewerNode);
    this.velocityPlotNode = $('<div></div>').appendTo(this.viewerNode);

    // Data table
    this.tableContainer = $('<div></div>')
      .css({
        'max-height': '120px',
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

    // Initialize data storage
    this.size = 200;
    this.ptr = 0;
    this.jointNames = [];
    this.positionData = [new Array(this.size).fill(0)]; // First array is time
    this.velocityData = [new Array(this.size).fill(0)];

    // Color palette for joints
    this.colors = [
      '#ff6060', '#60ff60', '#6080ff', '#ffff60',
      '#ff60ff', '#60ffff', '#ff8040', '#40ff80',
      '#8040ff', '#ff4080', '#80ff40', '#4080ff'
    ];

    // Create plots (will be recreated when joints are known)
    this.positionPlot = null;
    this.velocityPlot = null;

    super.onCreate();
  }

  _createPlots() {
    // Destroy existing plots
    if (this.positionPlot) {
      this.positionPlot.destroy();
      this.positionPlotNode.empty();
    }
    if (this.velocityPlot) {
      this.velocityPlot.destroy();
      this.velocityPlotNode.empty();
    }

    // Build series config
    const positionSeries = [{}];
    const velocitySeries = [{}];

    for (let i = 0; i < this.jointNames.length; i++) {
      const color = this.colors[i % this.colors.length];
      const name = this.jointNames[i];
      const shortName = name.length > 12 ? name.substring(0, 11) + '…' : name;

      positionSeries.push({
        label: shortName,
        stroke: color,
        width: 1.5
      });
      velocitySeries.push({
        label: shortName,
        stroke: color,
        width: 1.5
      });
    }

    const commonOpts = {
      width: 300,
      height: 100,
      legend: { show: true },
      axes: [
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
        { stroke: "#a0a0a0", ticks: { stroke: "#404040" }, grid: { stroke: "#404040" } },
      ],
    };

    this.positionPlot = new uPlot(
      { ...commonOpts, series: positionSeries },
      this.positionData,
      this.positionPlotNode[0]
    );

    this.velocityPlot = new uPlot(
      { ...commonOpts, series: velocitySeries },
      this.velocityData,
      this.velocityPlotNode[0]
    );

    // Start update interval
    if (!this.updateInterval) {
      this.updateInterval = setInterval(() => this._updatePlots(), 200);
    }
  }

  _updatePlots() {
    if (!this.positionPlot || !this.velocityPlot) return;

    const getSlicedData = (data) => {
      if (data[0][this.ptr] === 0) {
        return data.map(arr => arr.slice(0, this.ptr));
      } else {
        return data.map(arr =>
          arr.slice(this.ptr, this.size).concat(arr.slice(0, this.ptr))
        );
      }
    };

    const width = this.positionPlotNode[0].clientWidth || 300;

    this.positionPlot.setSize({ width: width, height: 100 });
    this.positionPlot.setData(getSlicedData(this.positionData));

    this.velocityPlot.setSize({ width: width, height: 100 });
    this.velocityPlot.setData(getSlicedData(this.velocityData));
  }

  _updateTable(msg) {
    this.tableBody.empty();

    const names = msg.name || [];
    const positions = msg.position || [];
    const velocities = msg.velocity || [];
    const efforts = msg.effort || [];

    for (let i = 0; i < names.length; i++) {
      const tr = $('<tr></tr>').appendTo(this.tableBody);
      const color = this.colors[i % this.colors.length];

      // Joint name with color indicator
      const nameCell = $('<td></td>')
        .addClass('mdl-data-table__cell--non-numeric')
        .css({'padding': '4px 8px', 'max-width': '100px', 'overflow': 'hidden', 'text-overflow': 'ellipsis'})
        .appendTo(tr);

      $('<span></span>')
        .css({
          'display': 'inline-block',
          'width': '8px',
          'height': '8px',
          'background': color,
          'border-radius': '2px',
          'margin-right': '6px'
        })
        .appendTo(nameCell);

      $('<span></span>')
        .text(names[i])
        .attr('title', names[i])
        .appendTo(nameCell);

      // Position
      const pos = positions[i] !== undefined ? positions[i] : 0;
      const posDeg = (pos * 180 / Math.PI).toFixed(1);
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

  onData(msg) {
    this.card.title.text(_rabo_topic_names[msg._topic_name] || msg._topic_name);

    const names = msg.name || [];
    const positions = msg.position || [];
    const velocities = msg.velocity || [];

    // Check if joints changed
    const jointsChanged = this.jointNames.length !== names.length ||
      !names.every((n, i) => this.jointNames[i] === n);

    if (jointsChanged) {
      this.jointNames = [...names];
      this.ptr = 0;

      // Reinitialize data arrays
      this.positionData = [new Array(this.size).fill(0)];
      this.velocityData = [new Array(this.size).fill(0)];

      for (let i = 0; i < names.length; i++) {
        this.positionData.push(new Array(this.size).fill(0));
        this.velocityData.push(new Array(this.size).fill(0));
      }

      // Recreate plots with new series
      this._createPlots();
    }

    // Update time series data
    const time = Math.floor(Date.now() / 10) / 100;
    this.positionData[0][this.ptr] = time;
    this.velocityData[0][this.ptr] = time;

    for (let i = 0; i < names.length; i++) {
      this.positionData[i + 1][this.ptr] = positions[i] !== undefined ? positions[i] : 0;
      this.velocityData[i + 1][this.ptr] = velocities[i] !== undefined ? velocities[i] : 0;
    }

    this.ptr = (this.ptr + 1) % this.size;

    // Update table
    this._updateTable(msg);
  }
}

JointStateViewer.friendlyName = "Joint State";

JointStateViewer.supportedTypes = [
    "sensor_msgs/msg/JointState",
];

JointStateViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(JointStateViewer);
