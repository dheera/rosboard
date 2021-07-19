"use strict";

// Plots time series data of a single .data variable.
// Works on all ROS single value std_msgs types.

class TimeSeriesPlotViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.plotNode = $('<div></div>')
      .appendTo(this.viewerNode);

    this.dataTable = $('<table></table>')
      .addClass('mdl-data-table')
      .addClass('mdl-js-data-table')
      .css({'width': '100%', 'table-layout': 'fixed' })
      .appendTo(this.viewerNode);

    let tr = $('<tr></tr>')
        .appendTo(this.dataTable);

    $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .text("data")
      .css({'width': '40%', 'font-weight': 'bold', 'overflow': 'hidden', 'text-overflow': 'ellipsis'})
      .appendTo(tr);
  this.valueField = $('<td></td>')
      .addClass('mdl-data-table__cell--non-numeric')
      .addClass('monospace')
      .css({'overflow': 'hidden', 'text-overflow': 'ellipsis'})
      .appendTo(tr);

    this.lastData = {};

    let opts = {
      id: "chart1",
      class: "my-chart",
      width: 300,
      height: 200,
      legend: {
        show: false,
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
        {},
        {
          show: true,
          spanGaps: false,
          stroke: "#00c080",
          width: 1,
        }
      ],
    };
    
    this.size = 500;
    this.data = [
      new Array(this.size).fill(0),
      new Array(this.size).fill(0),
    ];
    
    this.ptr = 0;

    this.uplot = new uPlot(opts, this.data, this.plotNode[0]);
    
    setInterval(()=> {
      let data = [];
      if(this.data[0][this.ptr] === 0) {
        data = [
          this.data[0].slice(0, this.ptr),
          this.data[1].slice(0, this.ptr),
        ];
      } else {
        data = [
          this.data[0].slice(this.ptr, this.size).concat(this.data[0].slice(0, this.ptr)),
          this.data[1].slice(this.ptr, this.size).concat(this.data[1].slice(0, this.ptr)),
        ];
      }
      this.uplot.setSize({width:this.plotNode[0].clientWidth, height:200});
      this.uplot.setData(data);
    }, 200);

    super.onCreate();
  }

  onData(msg) {
      this.card.title.text(msg._topic_name);
      this.valueField.text(msg.data);
      this.data[0][this.ptr] = Math.floor(Date.now() / 10)/ 100;
      this.data[1][this.ptr] = msg.data;
      this.ptr = (this.ptr + 1) % this.size;
  }
}

TimeSeriesPlotViewer.friendlyName = "Time series plot";

TimeSeriesPlotViewer.supportedTypes = [
    "std_msgs/msg/Bool",
    "std_msgs/msg/Byte",
    "std_msgs/msg/Char",
    "std_msgs/msg/Int8",
    "std_msgs/msg/Int16",
    "std_msgs/msg/Int32",
    "std_msgs/msg/Int64",
    "std_msgs/msg/UInt8",
    "std_msgs/msg/UInt16",
    "std_msgs/msg/UInt32",
    "std_msgs/msg/UInt64",
    "std_msgs/msg/Float32",
    "std_msgs/msg/Float64",
];

TimeSeriesPlotViewer.maxUpdateRate = 100.0;

Viewer.registerViewer(TimeSeriesPlotViewer);