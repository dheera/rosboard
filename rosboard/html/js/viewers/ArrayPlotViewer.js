"use strict";

// Plots time series data for any number of Float32 variables in Float32MultiArray.

class ArrayPlotViewer extends Viewer {
  /**
   * Gets called when Viewer is first initialized.
   * @override
   **/
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({ 'font-size': '11pt' })
      .appendTo(this.card.content);

    this.plotNode = $('<div></div>')
      .appendTo(this.viewerNode);

    this.dataTable = $('<table></table>')
      .addClass('mdl-data-table')
      .addClass('mdl-js-data-table')
      .css({ 'width': '100%', 'table-layout': 'fixed' })
      .appendTo(this.viewerNode);

    this.lastData = {};
    this.numFields = 0; // Will be dynamically updated
    this.size = 500; // Buffer size
    this.data = [[...new Array(this.size).fill(0)]]; // Initialize with timestamp

    this.ptr = 0;
    this.uplot = null; // To hold the dynamic uPlot instance

    this.createEmptyPlot();
    super.onCreate();
  }

  /**
   * Dynamically creates or updates the plot based on the number of fields.
   **/
  createDynamicPlot(numFields) {
    this.numFields = numFields;

    let opts = {
      id: "chart1",
      class: "my-chart",
      width: 300,
      height: 200,
      legend: {
        show: true,
      },
      axes: [
        {
          stroke: "#a0a0a0",
          ticks: {
            stroke: "#404040",
          },
          grid: {
            stroke: "#404040",
          },
        },
        {
          stroke: "#a0a0a0",
          ticks: {
            stroke: "#404040",
          },
          grid: {
            stroke: "#404040",
          },
        },
      ],
      series: [
        {}, // Timestamp (X-axis)
        ...Array.from({ length: numFields }, (_, i) => ({
          label: `Variable ${i + 1}`,
          show: true,
          spanGaps: false,
          stroke: `hsl(${(i * 360) / numFields}, 100%, 50%)`,
          width: 1,
        })),
      ],
    };

    // Initialize new data structure with dynamic field count
    this.data = Array.from({ length: numFields + 1 }, () =>
      new Array(this.size).fill(0)
    );

    // Destroy the old plot if it exists
    if (this.uplot) {
      this.uplot.destroy();
    }

    // Create a new plot instance
    this.uplot = new uPlot(opts, this.data, this.plotNode[0]);

    // Setup periodic update for the plot
    setInterval(() => {
      let chartData = [];
      if (this.data[0][this.ptr] === 0) {
        chartData = this.data.map((series) =>
          series.slice(0, this.ptr)
        );
      } else {
        chartData = this.data.map((series) =>
          series.slice(this.ptr, this.size).concat(series.slice(0, this.ptr))
        );
      }
      this.uplot.setSize({
        width: this.plotNode[0].clientWidth,
        height: 200,
      });
      this.uplot.setData(chartData);
    }, 200);
  }

  /**
   * Placeholder empty plot until data arrives.
   **/
  createEmptyPlot() {
    this.createDynamicPlot(0); // Start with no fields
  }

  /**
   * Called whenever new data is received for this Viewer.
   * @override
   **/
  onData(msg) {
    this.card.title.text(msg._topic_name);

    // Infer the number of fields from the incoming data
    const numFields = msg.data.length;

    // Dynamically recreate plot if the field count changes
    if (numFields !== this.numFields) {
      this.createDynamicPlot(numFields);
    }

    // Update data for each field
    this.data[0][this.ptr] = Math.floor(Date.now() / 10) / 100; // Timestamp
    for (let i = 0; i < numFields; i++) {
      this.data[i + 1][this.ptr] = msg.data[i];
    }
    this.ptr = (this.ptr + 1) % this.size;
  }
}

// Register Viewer
ArrayPlotViewer.friendlyName = "Dynamic Multi Float32 Time Series Plot";

ArrayPlotViewer.supportedTypes = [
  "std_msgs/msg/Float32MultiArray",
  "std_msgs/msg/Float64MultiArray",
  "std_msgs/msg/Int8MultiArray",
  "std_msgs/msg/Int16MultiArray",
  "std_msgs/msg/Int32MultiArray",
  "std_msgs/msg/Int64MultiArray",
  "std_msgs/msg/UInt8MultiArray",
  "std_msgs/msg/UInt16MultiArray",
  "std_msgs/msg/UInt32MultiArray",
  "std_msgs/msg/UInt64MultiArray",
];

ArrayPlotViewer.maxUpdateRate = 100.0;

Viewer.registerViewer(ArrayPlotViewer);

