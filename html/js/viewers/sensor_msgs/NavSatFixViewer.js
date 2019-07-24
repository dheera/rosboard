"use strict";

importCssOnce('/css/leaflet.css');
importCssOnce('/css/leaflet.easy-button.css');

importJsOnce('/js/leaflet.js');
importJsOnce('/js/leaflet.easy-button.js');

class NavSatFixViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.COLORS = ['#0080cc', '#cc8000', '#80cc00', '#8000cc', '#cc0080', '#00cc80', '#cccc00', '#00cccc', '#cc00cc'];
    this.topicN = 0;
    this.listeners = {};
    this.markersByTopic = {};
    this.viewerContainerNode = $('<div></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.cardContentNode);
    this.viewerNode = $('<div></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.viewerContainerNode);
    this.map = L.map(this.viewerNode[0]).setView([51.505, -0.09], 19);
    this.renderer = L.canvas({ padding: 0.5 });

    L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {
      maxZoom: 22,
      maxNativeZoom: 19
    }).addTo(this.map);

    var that = this;
    L.easyButton('<i class="material-icons" style="font-size:20px!important;margin-top:-2px;vertical-align:middle;line-height:20px;eight:20px!important;width:20px!important;">clear_all</i>', function(btn, map) {
      for(var topic in that.listeners) {
      while(that.markersByTopic[topic].length > 0) {
        that.map.removeLayer(that.markersByTopic[topic][0]);
        that.markersByTopic[topic].shift();
      }
      }
    }).addTo(this.map);

    this.panInterval = setInterval(function() {
      if(that.lastMessage) {
          that.map.panTo(new L.LatLng(that.lastMessage.latitude, that.lastMessage.longitude));
      }
    }, 500);

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
    clearInterval(this.panInterval);
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
      messageType : 'sensor_msgs/NavSatFix'
    });
    listener.color = this.COLORS[this.topicN++ % this.COLORS.length];
    this.listeners[topic] = listener;
    var that = this;
    listener.subscribe(function(message) {
      if(!message) return;
      if(!message.latitude || !message.longitude) return;
      var marker = L.circleMarker([message.latitude, message.longitude], { renderer: that.renderer, color: this.color, radius: 3 });
      marker.addTo(that.map);
      if(!(topic in that.markersByTopic)) {
        that.markersByTopic[topic] = [];
      }
      that.markersByTopic[topic].push(marker);
      while(that.markersByTopic[topic].length > 1024) {
        that.map.removeLayer(that.markersByTopic[topic][0]);
        that.markersByTopic[topic].shift();
      }
      that.lastMessage = message;
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
}

NavSatFixViewer._TYPE = 'sensor_msgs/NavSatFix';
addViewer(NavSatFixViewer);
