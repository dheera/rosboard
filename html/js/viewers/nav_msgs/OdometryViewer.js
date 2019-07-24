"use strict";

class OdometryViewer extends Viewer {
  onCreate() {
    this.listeners = {};
    this.viewerContainerNode = $('<div></div>')
      .css({'width': '100%', 'padding-bottom': '75%', 'position': 'relative'})
      .appendTo(this.cardContentNode);
    this.viewerNode = $('<div id="OdometryViewer-viewer"></div>')
      .css({'position': 'absolute', 'top': '0', 'bottom': '0', 'left': '0', 'right': '0'})
      .appendTo(this.viewerContainerNode);

    this.viewer = new ROS3D.Viewer({
       divID : 'OdometryViewer-viewer',
       width : this.viewerNode[0].clientWidth,
       height : this.viewerNode[0].clientHeight,
       antialias : true
    });

    this.viewer.addObject(new ROS3D.Grid({
       cellSize: 1
    }));

    this.tfClient = new ROSLIB.TFClient({
        ros: ros,
        angularThres: 0.1,
        transThres: 0.1,
        rate: 10.0,
        fixedFrame: '/base_frame'
    });

    super.onCreate();
  }

  onDestroy() {
    for(topic in this.listeners) {
      this.removeTopic(topic);
    }
    super.onDestroy();
  }

  addTopic(topic) {
    if(topic in this.listeners) return;
    this.listeners[topic] = new ROS3D.Odometry({
        ros: ros,
        topic: topic,
        tfClient: this.tfClient,
        rootObject: this.viewer.scene,
        size: 0.1,
        max_pts: 10000
    });
  }

  removeTopic(topic) {
    if(!(topic in this.listeners)) return;
    for(var i in this.viewer.scene.children) {
      if(this.listeners[topic].particles.sn === this.viewer.scene.children[i]) {
        this.viewer.scene.remove(this.viewer.scene.children[i]);
      }
    }
    delete(this.listeners[topic]);
  }

  getTopics() {
    return Object.keys(this.listeners);
  }

  onResize() {
    this.viewer.resize(this.viewerNode[0].clientWidth, this.viewerNode[0].clientHeight);
  }
}

OdometryViewer._TYPE = 'nav_msgs/Odometry';
addViewer(OdometryViewer);

