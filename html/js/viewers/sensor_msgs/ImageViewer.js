"use strict";

class ImageViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.listeners = {};
    this.viewerNode = $('<div></div>')
      .appendTo(this.cardContentNode);
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
    this.listeners[topic] = {
      'quality': 60,
      'width': 640,
      'height': 480,
    };
    this.listeners[topic]['displayNode'] = $('<img style="width:100%;" src="http://' + window.location.hostname + ':8080/stream?topic=' + topic + '&quality=' + this.listeners[topic]['quality'] + '&width=' + this.listeners[topic]['width'] + '&height=' + this.listeners[topic]['height'] + '"></img>').appendTo(this.viewerNode);
    this.updateSizes();
  }

  /**
    * Removes a topic from the viewer.
    * @override
  **/
  removeTopic(topic){
    if(!(topic in this.listeners)) return;
    this.listeners[topic]['displayNode'].remove();
    delete(this.listeners[topic]);
    this.updateSizes();
  }

  /**
    * Returns a list of topics subscribed to.
    * @override
  **/
  getTopics() {
    return Object.keys(this.listeners);
  }

  updateSizes() {
    var numTopics = Object.keys(this.listeners).length;
    if(numTopics<=1) {
      this.viewerNode.find('img').css({'width': '100%'});
    } else if(numTopics >=2 && numTopics <=4) {
      this.viewerNode.find('img').css({'width': '50%'});
    } else {
      this.viewerNode.find('img').css({'width': '33%'});
    }
  }
};

ImageViewer._TYPE = 'sensor_msgs/Image';
ImageViewer._SINGLE = true;
addViewer(ImageViewer);

