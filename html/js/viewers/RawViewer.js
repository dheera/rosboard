"use strict";

class RawViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // keeps track of ROS topic listeners
    this.listeners = {};

    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.cardContentNode);

    // fade out slightly if topic not published to
    // can be overriden before super.onCreate() is called
    if(!this.fadeDelay) {
       this.fadeDelay = 200;
    }

    super.onCreate();
  }

  /**
    * Gets called when Viewer is about to be destroyed.
    * @override
  **/
  onDestroy() {
    for(var topic in this.listeners) {
      this.removeTopic(topic);
    }
    super.onDestroy();
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

    var that = this;
    listener.subscribe(function(message) {
      var data = that.getData(message);
      if(that.fadeTimeout) window.clearTimeout(that.fadeTimeout);
      that.fadeTimeout = window.setTimeout(function() {
        that.cardContentNode.css({'opacity': 0.5});
      }, that.fadeDelay);
      that.cardContentNode.css({'opacity': 1});
      data['_time'] = Date.now();
      if(!that.fieldNodes) {
        that.fieldNodes = { };
        var table = $('<table></table>')
          .addClass('mdl-data-table')
          .addClass('mdl-js-data-table')
          .css({'width': '100%', 'table-layout': 'fixed' })
          .appendTo(that.viewerNode);
        for(var field in data) {
          var tr = $('<tr></tr>')
            .appendTo(table);
          $('<td></td>')
            .addClass('mdl-data-table__cell--non-numeric')
            .text(field)
            .css({'width': '25%', 'font-weight': 'bold'})
            .appendTo(tr);
          that.fieldNodes[field] = $('<td></td>')
            .addClass('mdl-data-table__cell--non-numeric')
            .appendTo(tr);
        }
      }
      for(var field in data) {
        that.fieldNodes[field].text(JSON.stringify(data[field], null, '  '));
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
    return message;
  }
}

RawViewer._SINGLE = true;
