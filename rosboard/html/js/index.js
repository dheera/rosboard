"use strict";

let viewers = [];
let registerViewer = (viewer) => { viewers.push(viewer); };
let getViewerForType = (type) => {
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }
  for(let i in viewers) {
    if(viewers[i].supportedTypes.includes(type)) {
      return viewers[i];
    }
    if(viewers[i].supportedTypes.includes("*")) {
      return viewers[i];
    }
  }
  return null;
}

importJsOnce("js/viewers/Viewer.js");
importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/GenericViewer.js");

let viewersByTopic = {};

let $grid = null;
$(() => {
  $grid = $('.grid').packery({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
});

$(function() {
  let dummy = newCard();
  dummy.title.text("Loading ...");
  setTimeout(() => { dummy.remove() }, 1500);
  // rosoutViewer = new LogViewer(newCard());
});

setInterval(() => {
  $grid.packery("reloadItems");
  $grid.packery();
}, 500);

setInterval(() => {
  if(currentTransport && !currentTransport.isConnected()) {
    console.log("attempting to reconnect ...");
    currentTransport.connect();
  }
}, 5000);

function newCard() {
  // creates a new card, adds it to the grid, and returns it.
  let card = $("<div></div>").addClass('card')
    .appendTo($('.grid'));
  card.title = $('<div></div>').addClass('card-title').text('').appendTo(card);
  card.content = $('<div></div>').addClass('card-content').text('').appendTo(card);
  return card;
}

class RosbagTransport {
  // not yet implemented
}

class Rosbag2Transport {
  // not yet implemented
}

class WebSocketV1Transport {
  constructor({path, onopen, onclose, on_ros_msg, on_topics}) {
    this.path = path;
    this.onopen = onopen ? onopen.bind(this) : null;
    this.onclose = onclose ? onclose.bind(this) : null;
    this.on_ros_msg = on_ros_msg ? on_ros_msg.bind(this) : null;
    this.on_topics = on_topics ? on_topics.bind(this) : null;
    this.ws = null;
  }

  connect() {
    var protocolPrefix = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
    let abspath = protocolPrefix + '//' + location.host + this.path;

    let that = this;

    this.ws = new WebSocket(abspath);

    this.ws.onopen = function(){
      console.log("connected");
      if(that.onopen) that.onopen(that);
    }
    
    this.ws.onclose = function(){
      console.log("disconnected");
      if(that.onclose) that.onclose(that);
    }

    this.ws.onmessage = function(wsmsg) {
      let data = JSON.parse(wsmsg.data);
      let wsMsgType = data[0];

      if(wsMsgType === "ping") this.send(JSON.stringify(["pong", Date.now()]));
      else if(wsMsgType === "ros_msg" && that.on_ros_msg) that.on_ros_msg(data[1]);
      else if(wsMsgType === "topics" && that.on_topics) that.on_topics(data[1]);
      else console.log("received unknown message: " + wsmsg);
    }
  }

  isConnected() {
    return (this.ws && this.ws.readyState === this.ws.OPEN);
  }

  subscribe(topic_name) {
    this.ws.send(JSON.stringify(["sub", topic_name]));
  }
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onopen: function() {
      for(let topic_name in viewersByTopic) {
        this.subscribe(topic_name);
      }
    },
    on_ros_msg: function(msg) {
      if(!viewersByTopic[msg._topic_name]) {
        let card = newCard();
        let viewer = getViewerForType(msg._topic_type);
        try {
          viewersByTopic[msg._topic_name] = new viewer(card);
          viewersByTopic[msg._topic_name].update(msg);
        } catch(e) {
          console.log(e);
          card.remove();
        }
        $grid.packery("appended", card);
      } else {
        viewersByTopic[msg._topic_name].update(msg);
      }
    },
    on_topics: function(topics) {
      $("#topics-nav-supported").empty();
      $("<a></a>")
              .text("dmesg")
              .addClass("mdl-navigation__link")
              .click(() => { this.subscribe("_dmesg"); })
              .appendTo($("#topics-nav-supported"));
      for(let topic_name in topics) {
          let topic_type = topics[topic_name];
          $("<a></a>")
              .text(topic_name)
              .addClass("mdl-navigation__link")
              .click(() => { this.subscribe(topic_name); })
              .appendTo($("#topics-nav-supported"));
      }
    },
  });
  currentTransport.connect();
}


if(window.location.href.indexOf("rosboard.com") === -1) {
  initDefaultTransport();
}