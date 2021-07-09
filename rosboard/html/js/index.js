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

importJsOnce("js/transports/WebSocketV1Transport.js");

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

let onOpen = function() {
  for(let topic_name in viewersByTopic) {
    this.subscribe(topic_name);
  }
}

let onRosMsg = function(msg) {
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

let onTopics = function(topics) {
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
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onOpen: onOpen,
    onRosMsg: onRosMsg,
    onTopics: onTopics,
  });
  currentTransport.connect();
}

if(window.location.href.indexOf("rosboard.com") === -1) {
  initDefaultTransport();
}