"use strict";

class MultiGraphViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // keeps track of ROS topic listeners
    this.listeners = {};

    this.displayNodesByField = {};
    this.plotContainerNodesByField = {};
    this.plotNodesByField = {};
    this.plotDataByField = {};

    // to be overriden before calling super.onCreate when extending this class
    if(!this.maxPoints) this.maxPoints = 256;
    if(!this.updateInterval) this.updateInterval = 100;
    if(!this.fields) this.fields = { 'data': { } };

    this.numFields = Object.keys(this.fields).length;

    for(var field in this.fields) {
      this.displayNodesByField[field] = $('<div></div>')
        .appendTo(this.cardContentNode);
      this.plotContainerNodesByField[field] = $('<div></div>')
        .css({'width': '100%', 'padding-bottom': '35%', 'position': 'relative'})
        .appendTo(this.cardContentNode);
      this.plotNodesByField[field] = $('<div></div>')
        .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
        .appendTo(this.plotContainerNodesByField[field]);
      this.plotDataByField[field] = {};
    }

    // x-axis values are constant for all plots
    // this is basically range(256) in JS
    this.plotXValues = Array.apply(null, Array(this.maxPoints)).map(function (_, i) {return i;});

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
      for(var field in that.fields) {
        that.plotNodesByField[field][0].data = Object.values(that.plotDataByField[field]);
        Plotly.redraw(that.plotNodesByField[field][0]);
      }
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
    for(var field in this.fields) {
      Plotly.relayout(this.plotNodesByField[field][0], {'autosize': true});
    }
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

    for(var field in this.fields) {
      this.plotDataByField[field][topic] = {'topic': topic, 'name': topic, 'y': [], 'line': { } };
    }

    this.redrawPlotTraces();

    var that = this;
    listener.subscribe(function(message) {
      var data = that.getData(message);
      for(field in that.fields) {
        that.plotDataByField[field][topic]['y'].push(data[field]);
        while(that.plotDataByField[field][topic]['y'].length > that.maxPoints) {
          that.plotDataByField[field][topic]['y'].shift();
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
    for(var field in this.fields) {
      delete(this.plotDataByField[field][topic]);
    }
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
    for(var field in this.fields) {
      Plotly.purge(this.plotNodesByField[field][0]);
      Plotly.plot(this.plotNodesByField[field][0], Object.values(this.plotDataByField[field]), this.plotOptions);
    }
  }
}

