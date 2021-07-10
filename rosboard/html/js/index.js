"use strict";

let __version__ = "1.1.0";

importJsOnce("js/viewers/Viewer.js");
importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/GenericViewer.js");

importJsOnce("js/transports/WebSocketV1Transport.js");

var snackbarContainer = document.querySelector('#demo-toast-example');

let viewersByTopic = {};

let $grid = null;
$(() => {
  $grid = $('.grid').packery({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
});

setTimeout(versionCheck, 5000);

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
  card.buttons = $('<div></div>').addClass('card-buttons').text('').appendTo(card);
  card.title = $('<div></div>').addClass('card-title').text('').appendTo(card);
  card.content = $('<div></div>').addClass('card-content').text('').appendTo(card);
  card.closeButton = $('<div></div>').addClass("card-button").text("X").appendTo(card.buttons);
  card.closeButton.click(() => {
    for(let topicName in viewersByTopic) {
      if(viewersByTopic[topicName].card === card) {
        delete(viewersByTopic[topicName]);
        currentTransport.unsubscribe({topicName:topicName});
      }
    }
    card.remove();
  })
  return card;
}

let onOpen = function() {
  for(let topic_name in viewersByTopic) {
    console.log("Re-subscribing to " + topic_name);
    this.subscribe({topicName: topic_name});
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

function versionCheck() {
  $.get("https://raw.githubusercontent.com/dheera/rosboard/master/setup.py").done((data) => {
    let matches = data.match(/version='(.*)'/);
    if(matches.length < 2) return;
    let latestVersion = matches[1].split(".").map(num => parseInt(num, 10));
    let currentVersion = __version__.split(".").map(num => parseInt(num, 10));
    let latestVersionInt = latestVersion[0] * 1000000 + latestVersion[1] * 1000 + latestVersion[2];
    let currentVersionInt = currentVersion[0] * 1000000 + currentVersion[1] * 1000 + currentVersion[2];
    if(currentVersion < latestVersion) {
      snackbarContainer.MaterialSnackbar.showSnackbar({
        message: "New version of ROSboard available (" + __version__ + " -> " + matches[1] + ").",
        actionText: "Check it out",
        actionHandler: ()=> {window.location.href="https://github.com/dheera/rosboard/"},
      });
    }
  });
}

if(window.location.href.indexOf("rosboard.com") === -1) {
  initDefaultTransport();
}

