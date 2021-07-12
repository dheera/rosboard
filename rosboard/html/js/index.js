"use strict";

let __version__ = "1.1.1";

importJsOnce("js/viewers/Viewer.js");
importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/LaserScanViewer.js");
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
    console.log("Received unsolicited message", msg);
  } else {
    viewersByTopic[msg._topic_name].update(msg);
  }
}

let currentTopics = {};
let currentTopicsStr = "";

let onTopics = function(topics) {
  
  // check if topics has actually changed, if not, don't do anything
  // lazy shortcut to deep compares, might possibly even be faster than
  // implementing a deep compare due to
  // native optimization of JSON.stringify
  let newTopicsStr = JSON.stringify(topics);
  if(newTopicsStr === currentTopicsStr) return;
  currentTopics = topics;
  currentTopicsStr = newTopicsStr;
  
  let topicTree = treeifyPaths(Object.keys(topics));
  
  $("#topics-nav-ros").empty();
  $("#topics-nav-system").empty();
  
  addTopicTreeToNav(topicTree[0], $('#topics-nav-ros'));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_dmesg", topicType: "rcl_interfaces/msg/Log"}); })
  .text("dmesg")
  .appendTo($("#topics-nav-system"));
}

function addTopicTreeToNav(topicTree, el, level = 0, path = "") {
  topicTree.children.sort((a, b) => {
    if(a.name>b.name) return 1;
    if(a.name<b.name) return -1;
    return 0;
  });
  topicTree.children.forEach((subTree, i) => {
    let subEl = $('<div></div>')
    .css(level < 1 ? {} : {
      "padding-left": "0pt",
      "margin-left": "12pt",
      "border-left": "1px dashed #808080",
    })
    .appendTo(el);
    let fullTopicName = path + "/" + subTree.name;
    let topicType = currentTopics[fullTopicName];
    if(topicType) {
      $('<a></a>')
        .addClass("mdl-navigation__link")
        .css({
          "padding-left": "12pt",
          "margin-left": 0,
        })
        .click(() => { initSubscribe({topicName: fullTopicName, topicType: topicType}); })
        .text(subTree.name)
        .appendTo(subEl);
    } else {
      $('<a></a>')
      .addClass("mdl-navigation__link")
      .attr("disabled", "disabled")
      .css({
        "padding-left": "12pt",
        "margin-left": 0,
        opacity: 0.5,
      })
      .text(subTree.name)
      .appendTo(subEl);
    }
    addTopicTreeToNav(subTree, subEl, level + 1, path + "/" + subTree.name);
  });
}

function initSubscribe({topicName, topicType}) {
  // creates a subscriber for topicName
  // and also initializes a viewer (if it doesn't already exist)
  // in advance of arrival of the first data
  // this way the user gets a snappy UI response because the viewer appears immediately
  if(!viewersByTopic[topicName]) {
    let card = newCard();
    let viewer = Viewer.getViewerForType(topicType);
    try {
      viewersByTopic[topicName] = new viewer(card);
      viewersByTopic[topicName].onClose = function() {
        if(viewersByTopic[topicName] === this) {
          delete(viewersByTopic[topicName]);
          currentTransport.unsubscribe({topicName:topicName});
        }
        $grid.packery("remove", card);
      }
    } catch(e) {
      console.log(e);
      card.remove();
    }
    $grid.packery("appended", card);
  }
  currentTransport.subscribe({topicName: topicName});
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
  // turn a bunch of ros topics into a tree
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
  $.get("https://raw.githubusercontent.com/dheera/rosboard/release/setup.py").done((data) => {
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

