"use strict";

importJsOnce("js/viewers/Viewer.js");
importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/MapViewer.js");
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
    this.subscribe({topic_name: topic_name});
  }
}

let onMsg = function(msg) {
  if(!viewersByTopic[msg._topic_name]) {
    let card = newCard();
    let viewer = Viewer.getViewerForType(msg._topic_type);
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

let currentTopics = null;
let onTopics = function(topics) {
  
  let newTopics = JSON.stringify(topics);
  if(newTopics === currentTopics) return;
  currentTopics = newTopics;
  
  let topicTree = treeifyPaths(Object.keys(topics));
  $("#topics-nav-supported").empty();
  $("<a></a>")
          .text("dmesg")
          .addClass("mdl-navigation__link")
          .click(() => { this.subscribe({topicName: "_dmesg"}); })
          .appendTo($("#topics-nav-supported"));
  for(let topic_name in topics) {
      let topic_type = topics[topic_name];
      $("<a></a>")
          .text(topic_name)
          .addClass("mdl-navigation__link")
          .click(() => { this.subscribe({topicName: topic_name}); })
          .appendTo($("#topics-nav-supported"));
  }
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onOpen: onOpen,
    onMsg: onMsg,
    onTopics: onTopics,
  });
  currentTransport.connect();
}

function treeifyPaths(paths) {
  let result = [];
  let level = {result};

  paths.forEach(path => {
    path.split('/').reduce((r, name, i, a) => {
      if(!r[name]) {
        r[name] = {result: []};
        r.result.push({name, children: r[name].result})
      }
      
      return r[name];
    }, level)
  });
  return result;
}

if(window.location.href.indexOf("rosboard.com") === -1) {
  initDefaultTransport();
}