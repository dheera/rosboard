"use strict";

// Plots time series data of a single .data variable.
// Works on all ROS single value std_msgs types.

class CurrentPlotViewer extends Viewer {
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

    // TODO: fill in how many plots to show
    this.num_plots = 4;
    // TODO: fill in what the plots should be called
    let plots_title = ["current_fr", "current_rr", "current_rl", "current_fl"];

    let opts_list = [];
    for (let i = 0; i < this.num_plots; i++) {
      opts_list.push({
        id: "chart" + i,
        title: plots_title[i],
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
      });
    }

    this.size = 500;
    
    this.data_list = [];
    this.ptr_list = []
    this.uplot_list = [];
    for (let i = 0; i < this.num_plots; i++) {
        this.data_list.push([
          new Array(this.size).fill(0),
          new Array(this.size).fill(0),
        ]);
        console.log(this.data_list[0]);
        this.ptr_list.push(0);
        this.uplot_list.push(new uPlot(opts_list[i], this.data_list[i], this.plotNode[0]));
    }
    
    setInterval(()=> {
      for (let i = 0; i < this.num_plots; i++) {
        let data = [];
        if(this.data_list[i][0][this.ptr_list[i]] === 0) {
          data = [
            this.data_list[i][0].slice(0, this.ptr_list[i]),
            this.data_list[i][1].slice(0, this.ptr_list[i]),
          ];
        } else {
          data = [
            this.data_list[i][0].slice(this.ptr_list[i], this.size).concat(this.data_list[i][0].slice(0, this.ptr_list[i])),
            this.data_list[i][1].slice(this.ptr_list[i], this.size).concat(this.data_list[i][1].slice(0, this.ptr_list[i])),
          ];
        }
        this.uplot_list[i].setSize({width:this.plotNode[0].clientWidth, height:200});
        this.uplot_list[i].setData(data);
    }
    }, 200);

    super.onCreate();
  }

  onData(msg) {
      this.card.title.text(msg._topic_name);
      this.valueField.text(msg.data);
      // TODO: fill in which parts of message to show, must match this.num_plots
      let data_to_plot = [msg.current_fr, msg.current_rr, msg.current_rl, msg.current_fl];
      for (let i = 0; i < this.num_plots; i++) {
        this.data_list[i][0][this.ptr_list[i]] = Math.floor(Date.now() / 10)/ 100;
        this.data_list[i][1][this.ptr_list[i]] = data_to_plot[i];
        this.ptr_list[i] = (this.ptr_list[i] + 1) % this.size;
      }
  }
}

// TODO: Customize name
CurrentPlotViewer.friendlyName = "Motors Current View";

// TODO: Fill here message type
CurrentPlotViewer.supportedTypes = [
    "can_msgs/msg/MotorsCurrent",
];

CurrentPlotViewer.maxUpdateRate = 100.0;

Viewer.registerViewer(CurrentPlotViewer);