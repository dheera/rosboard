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
  if(ws.readyState === ws.CLOSED) {
      console.log("attempting to reconnect ...");
      connect();
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

function createWebSocket(path) {
    // creates and returns a websocket with the given path but uses ws for http and wss for https
    var protocolPrefix = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
    return new WebSocket(protocolPrefix + '//' + location.host + path);
}

let ws = null;

function subscribe(topic_name) {
    ws.send(JSON.stringify(["sub", topic_name]));
}

function connect() {
    if(ws) {
        ws.onmessage = null;
        ws.close();
    }

    ws = createWebSocket("/rosboard/v1");

    ws.onopen = function(){
      console.log("connected");
    }
    
    ws.onclose = function(){
      console.log("disconnected");
    }

    ws.onmessage = function(wsmsg) {
      let data = JSON.parse(wsmsg.data);
      let wsMsgType = data[0];

      if(wsMsgType === "ping") ws.send(JSON.stringify(["pong", Date.now()]));
      else if(wsMsgType === "ros_msg") ws.on_ros_msg(data[1]);
      else if(wsMsgType === "log_msg") ws.on_log_msg(data[1]);
      else if(wsMsgType === "topics") ws.on_topics(data[1]);
      else if(wsMsgType === "pong") ws.on_pong(data[1]);
      else console.log(wsmsg);
    }

    ws.on_pong = function(time) {
        console.log(time);
    }

    ws.on_log_msg = function(msg) {
        console.log(msg);
    }

    ws.on_ros_msg = function(msg) {
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
    }

    ws.on_topics = function(topics) {
        $("#topics-nav-supported").empty();
        for(let topic_name in topics) {
            let topic_type = topics[topic_name];
            $("<a></a>")
                .text(topic_name)
                .addClass("mdl-navigation__link")
                .click(() => { subscribe(topic_name); })
                .appendTo($("#topics-nav-supported"));
        }
    }
}

connect();

