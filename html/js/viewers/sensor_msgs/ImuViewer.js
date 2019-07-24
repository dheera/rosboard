"use strict";

class ImuViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // we don't want any padding for this card because we are using a tab interface
    this.cardContentNode.css('padding', 0);

    // keeps track of ROS topic listeners
    this.listeners = {};

    this.currentActivePlot = 'o';
    this.lastActivePlot = 'o';

    this.tabContainerNode = $('<div class="mdl-tabs mdl-js-tabs mdl-js-ripple-effect"></div>')
      .appendTo(this.cardContentNode);

    var that = this;
    this.tabBarNode = $('<div class="mdl-tabs__tab-bar"></div>')
      .html(
        '<a href="#sensor_msgs-Imu-panel-o" class="mdl-tabs__tab is-active">orientation</a>' +
        '<a href="#sensor_msgs-Imu-panel-l" class="mdl-tabs__tab">linear</a>' +
        '<a href="#sensor_msgs-Imu-panel-a" class="mdl-tabs__tab">angular</a>'
      )
      .appendTo(this.tabContainerNode)
      .find('a').click(function() {
        that.onResize();
      });
    this.plotOrientationContainerNode = $('<div class="mdl-tabs__panel is-active" id="sensor_msgs-Imu-panel-o"></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.tabContainerNode);
    this.plotOrientationNode = $('<div></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.plotOrientationContainerNode);
    this.plotLinearContainerNode = $('<div class="mdl-tabs__panel" id="sensor_msgs-Imu-panel-l"></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.tabContainerNode);
    this.plotLinearNode = $('<div></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.plotLinearContainerNode);
    this.plotAngularContainerNode = $('<div class="mdl-tabs__panel" id="sensor_msgs-Imu-panel-a"></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.tabContainerNode);
    this.plotAngularNode = $('<div></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.plotAngularContainerNode);

    // x-axis values are constant for all plots
    // this is basically range(256) in JS
    this.plotXValues = Array.apply(null, Array(256)).map(function (_, i) {return i;});

    // holds time series data (y-axis values)
    this.plotOrientationData = {};
    this.plotLinearData = {};
    this.plotAngularData = {};

    // plot display options
    this.plotOrientationOptions = {
        'autosize': true,
        'xaxis': { 'range': [0, 256] },
        'yaxis': { 'range': [-Math.PI, Math.PI] },
        'margin': { 'l': 50, 'r': 50, 'b': 50, 't': 20, 'pad': 20 },
    };

    this.plotLinearOptions = {
        'autosize': true,
        'xaxis': { 'range': [0, 256] },
        'yaxis': { 'range': [-19.6, 19.6] },
        'margin': { 'l': 50, 'r': 50, 'b': 50, 't': 20, 'pad': 20 },
    };
    this.plotAngularOptions = {
        'autosize': true,
        'xaxis': { 'range': [0, 256] },
        'yaxis': { 'range': [-19.6, 19.6] },
        'margin': { 'l': 50, 'r': 50, 'b': 50, 't': 20, 'pad': 20 },
    };

    // instantiate the plots
    this.redrawPlotTraces();

    // updates the graph at a fixed frequency (10Hz) to avoid consuming
    // high CPU if the messages come in at a high rate
    var that = this;
    this.updateInterval = setInterval(function() {
      if($('#sensor_msgs-Imu-panel-o').hasClass('is-active')) that.currentActivePlot = 'o';
      if($('#sensor_msgs-Imu-panel-l').hasClass('is-active')) that.currentActivePlot = 'l';
      if($('#sensor_msgs-Imu-panel-a').hasClass('is-active')) that.currentActivePlot = 'a';

      if(that.currentActivePlot === 'o') {
        that.plotOrientationNode[0].data = Object.values(that.plotOrientationData);
        Plotly.redraw(that.plotOrientationNode[0]);
      }

      if(that.currentActivePlot === 'l') {
        that.plotLinearNode[0].data = Object.values(that.plotLinearData);
        Plotly.redraw(that.plotLinearNode[0]);
      }

      if(that.currentActivePlot === 'a') {
        that.plotAngularNode[0].data = Object.values(that.plotAngularData);
        Plotly.redraw(that.plotAngularNode[0]);
      }

      if(that.lastActivePlot !== that.currentActivePlot) {
        that.onResize();
      }

      that.lastActivePlot = that.currentActivePlot;
    }, 100);
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
    Plotly.relayout(this.plotOrientationNode[0], {'autosize': true});
    Plotly.relayout(this.plotLinearNode[0], {'autosize': true});
    Plotly.relayout(this.plotAngularNode[0], {'autosize': true});
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
      messageType : 'sensor_msgs/Imu'
    });

    this.listeners[topic] = listener;

    this.redrawPlotTraces();
    this.plotOrientationData[topic + '/y'] = { 'topic': topic, 'legendgroup': topic, 'name': 'y', 'y': [], 'line': { } };
    this.plotOrientationData[topic + '/p'] = { 'topic': topic, 'legendgroup': topic, 'name': 'p', 'y': [], 'line': { } };
    this.plotOrientationData[topic + '/r'] = { 'topic': topic, 'legendgroup': topic, 'name': 'r', 'y': [], 'line': { } };
    this.plotLinearData[topic + '/lax'] = { 'topic': topic, 'legendgroup': topic, 'name': 'lax', 'y': [], 'line': { } };
    this.plotLinearData[topic + '/lay'] = { 'topic': topic, 'legendgroup': topic, 'name': 'lay', 'y': [], 'line': { } };
    this.plotLinearData[topic + '/laz'] = { 'topic': topic, 'legendgroup': topic, 'name': 'laz', 'y': [], 'line': { } };
    this.plotAngularData[topic + '/avx'] = { 'topic': topic, 'legendgroup': topic, 'name': 'avx', 'y': [], 'line': { } };
    this.plotAngularData[topic + '/avy'] = { 'topic': topic, 'legendgroup': topic, 'name': 'avy', 'y': [], 'line': { } };
    this.plotAngularData[topic + '/avz'] = { 'topic': topic, 'legendgroup': topic, 'name': 'avz', 'y': [], 'line': { } };

    var that = this;
    listener.subscribe(function(message) {
      // compute Euler from quaternion
      var a = message.orientation.x / 16384.
      var b = message.orientation.y / 16384.
      var c = message.orientation.z / 16384.
      var d = message.orientation.w / 16384.
      var y = Math.atan2(2*a*b+2*c*d, 1-2*b*b-2*c*c)
      var p = Math.asin(2*(a*c-b*d))
      var r = Math.atan2(2*a*d+2*b*c, 1-2*c*c-2*d*d) + Math.PI
      if(r > Math.PI) {
         r -= 2*Math.PI
      }

      that.plotOrientationData[topic + '/y']['y'].push(y);
      that.plotOrientationData[topic + '/p']['y'].push(p);
      that.plotOrientationData[topic + '/r']['y'].push(r);
      that.plotLinearData[topic + '/lax']['y'].push(message.linear_acceleration.x);
      that.plotLinearData[topic + '/lay']['y'].push(message.linear_acceleration.y);
      that.plotLinearData[topic + '/laz']['y'].push(message.linear_acceleration.z);
      that.plotAngularData[topic + '/avx']['y'].push(message.angular_velocity.x);
      that.plotAngularData[topic + '/avy']['y'].push(message.angular_velocity.y);
      that.plotAngularData[topic + '/avz']['y'].push(message.angular_velocity.z);

      for(var key in that.plotOrientationData) {
        while(that.plotOrientationData[key]['y'].length > 256) {
          that.plotOrientationData[key]['y'].shift();
        }
      }
      for(var key in that.plotLinearData) {
        while(that.plotLinearData[key]['y'].length > 256) {
          that.plotLinearData[key]['y'].shift();
        }
      }
      for(var key in that.plotAngularData) {
        while(that.plotAngularData[key]['y'].length > 256) {
          that.plotAngularData[key]['y'].shift();
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
    delete(this.plotOrientationData[topic + '/y'])
    delete(this.plotOrientationData[topic + '/p'])
    delete(this.plotOrientationData[topic + '/r'])
    delete(this.plotLinearData[topic + '/lax'])
    delete(this.plotLinearData[topic + '/lay'])
    delete(this.plotLinearData[topic + '/laz'])
    delete(this.plotAngularData[topic + '/avx'])
    delete(this.plotAngularData[topic + '/avy'])
    delete(this.plotAngularData[topic + '/avz'])
    this.redrawPlotTraces();
  }

  /**
    * Returns a list of topics subscribed to.
    * @override
  **/
  getTopics() {
    return Object.keys(this.listeners);
  }

  redrawPlotTraces() {
    Plotly.purge(this.plotOrientationNode[0]);
    Plotly.purge(this.plotLinearNode[0]);
    Plotly.purge(this.plotAngularNode[0]);
    Plotly.plot(this.plotOrientationNode[0], Object.values(this.plotOrientationData), this.plotOrientationOptions);
    Plotly.plot(this.plotLinearNode[0], Object.values(this.plotLinearData), this.plotLinearOptions);
    Plotly.plot(this.plotAngularNode[0], Object.values(this.plotAngularData), this.plotAngularOptions);
  }
}

ImuViewer._TYPE = 'sensor_msgs/Imu';
addViewer(ImuViewer);

