"use strict";

class GraphViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // we don't want any padding for this card because we are using a tab interface
    this.cardContentNode.css('padding', 0);

    // keeps track of ROS topic listeners
    this.listeners = {};

    this.plotContainerNode = $('<div></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.cardContentNode);
    this.plotNode = $('<div></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.plotContainerNode);

    // x-axis values are constant for all plots
    // this is basically range(256) in JS
    this.plotXValues = Array.apply(null, Array(this.maxPoints)).map(function (_, i) {return i;});

    // holds time series data (y-axis values)
    this.plotData = {};

    // plot display options
    this.plotOptions = {
        'autosize': true,
        'xaxis': { 'range': [0, this.maxPoints] },
        'margin': { 'l': 50, 'r': 50, 'b': 50, 't': 20, 'pad': 20 },
    };

    // instantiate the plots
    this.redrawPlotTraces();

    // updates the graph at a fixed frequency (10Hz) to avoid consuming
    // high CPU if the messages come in at a high rate
    var that = this;
    this.updateInterval = setInterval(function() {
      that.plotNode[0].data = Object.values(that.plotData);
      Plotly.redraw(that.plotNode[0]);
    }, this.updateInterval);
    super.onCreate();
  }

  /**
    * Gets called when Viewer is about to be destroyed.
    * @override
  **/
  onDestroy() {
    clearInterval(this.updateInterval);
    for(var topic in this.listeners) {
      this.removeTopic(topic);
    }
    super.onDestroy();
  }

  onResize() {
    Plotly.relayout(this.plotNode[0], {'autosize': true});
  }

  /**
    * Adds a topic to the viewer.
    * @override
  **/
  addTopic(topic) {
    if(topic in this.listeners) return;
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : this.constructor._TYPE,
    });

    this.listeners[topic] = listener;

    this.redrawPlotTraces();

    var that = this;
    listener.subscribe(function(message) {
      var data = that.getData(message);
      for(var param in data) {
        var key = topic + '/' + param;
        if(!(key in that.plotData)) {
          that.plotData[key] = {'topic': topic, 'name': param, 'y': [], 'line': { } };
          that.redrawPlotTraces();
        }
        that.plotData[key]['y'].push(data[param]);
      }

      for(var key in that.plotData) {
        while(that.plotData[key]['y'].length > that.maxPoints) {
           that.plotData[key]['y'].shift();
        }
      }
    });
  }

  /**
    * Removes a topic from the viewer.
    * @override
  **/
  removeTopic(topic) {
    if(!(topic in this.listeners)) return;
    this.listeners[topic].unsubscribe();
    delete(this.listeners[topic]);

    // get rid of traces associated with the topic we are deleting
    delete(this.plotData[topic])
    this.redrawPlotTraces();
  }

  /**
    * Returns a list of topics subscribed to.
    * @override
  **/
  getTopics() {
    return Object.keys(this.listeners);
  }

  /**
    * Returns the variable to be plotted given a ROS message.
    * @override
  **/
  getData(message) {
    return {'y': message.data };
  }

  redrawPlotTraces() {
    Plotly.purge(this.plotNode[0]);
    Plotly.plot(this.plotNode[0], Object.values(this.plotData), this.plotOptions);
  }
}

