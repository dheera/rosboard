"use strict";

importJsOnce("js/viewers/meta/Viewer.js");
importJsOnce("js/viewers/meta/Space2DViewer.js");
importJsOnce("js/viewers/meta/Space3DViewer.js");

importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/ProcessListViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/LaserScanViewer.js");
importJsOnce("js/viewers/GeometryViewer.js");
importJsOnce("js/viewers/PolygonViewer.js");
importJsOnce("js/viewers/DiagnosticViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/PointCloud2Viewer.js");
importJsOnce("js/viewers/ImuViewer.js");
importJsOnce("js/viewers/JointStateViewer.js");

// GenericViewer must be last
importJsOnce("js/viewers/GenericViewer.js");

importJsOnce("js/transports/WebSocketV1Transport.js");

var snackbarContainer = document.querySelector('#demo-toast-example');

let subscriptions = {};

if(window.localStorage && window.localStorage.subscriptions) {
  if(window.location.search && window.location.search.indexOf("reset") !== -1) {
    subscriptions = {};
    updateStoredSubscriptions();
    window.location.href = "?";
  } else {
    try {
      subscriptions = JSON.parse(window.localStorage.subscriptions);
    } catch(e) {
      console.log(e);
      subscriptions = {};
    }
  }
}

let $grid = null;
$(() => {
  $grid = $('.grid').masonry({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
  $grid.masonry("layout");
});

setInterval(() => {
  if(currentTransport && !currentTransport.isConnected()) {
    console.log("attempting to reconnect ...");
    currentTransport.connect();
  }
}, 5000);

function updateStoredSubscriptions() {
  if(window.localStorage) {
    let storedSubscriptions = {};
    for(let topicName in subscriptions) {
      storedSubscriptions[topicName] = {
        topicType: subscriptions[topicName].topicType,
      };
    }
    window.localStorage['subscriptions'] = JSON.stringify(storedSubscriptions);
  }
}

function newCard() {
  // creates a new card, adds it to the grid, and returns it.
  let card = $("<div></div>").addClass('card')
    .appendTo($('.grid'));
  return card;
}

let onOpen = function() {
  const urlParams = new URLSearchParams(window.location.search);

  for( let [key, value] of urlParams ){
    key = key.replace(/\\/g, '/');
    value = value.replace(/\\/g, '/');

    console.log("Auto subscribing to " + key + " of type " + value);
      
    const subscriptions = JSON.parse(window.localStorage.getItem('subscriptions') || '{}');
    if (!(key in subscriptions)) {
      initSubscribe({topicName: key, topicType: value});
    }
  }          
  
  for(let topic_name in subscriptions) {
    console.log("Re-subscribing to " + topic_name);
    initSubscribe({topicName: topic_name, topicType: subscriptions[topic_name].topicType});
  }


}

let onSystem = function(system) {
  if(system.hostname) {
    console.log("hostname: " + system.hostname);
    $('.mdl-layout-title').text("ROSboard: " + system.hostname);
  }

  if(system.version) {
    console.log("server version: " + system.version);
    versionCheck(system.version);
  }
}

let onMsg = function(msg) {
  if(!subscriptions[msg._topic_name]) {
    console.log("Received unsolicited message", msg);
  } else if(!subscriptions[msg._topic_name].viewer) {
    console.log("Received msg but no viewer", msg);
  } else {
    subscriptions[msg._topic_name].viewer.update(msg);
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

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_top", topicType: "rosboard_msgs/msg/ProcessList"}); })
  .text("Processes")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_system_stats", topicType: "rosboard_msgs/msg/SystemStats"}); })
  .text("System stats")
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
  console.log( "Subscribing to " + topicName + " of type " + topicType);
  // creates a subscriber for topicName
  // and also initializes a viewer (if it doesn't already exist)
  // in advance of arrival of the first data
  // this way the user gets a snappy UI response because the viewer appears immediately
  if(!subscriptions[topicName]) {
    subscriptions[topicName] = {
      topicType: topicType,
    }
  }  
  currentTransport.subscribe({topicName: topicName});
  if(!subscriptions[topicName].viewer) {
    let card = newCard();
    let viewer = Viewer.getDefaultViewerForType(topicType);
    try {
      subscriptions[topicName].viewer = new viewer(card, topicName, topicType);
    } catch(e) {
      console.log(e);
      card.remove();
    }
    $grid.masonry("appended", card);
    $grid.masonry("layout");
  }
  updateStoredSubscriptions();
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onOpen: onOpen,
    onMsg: onMsg,
    onTopics: onTopics,
    onSystem: onSystem,
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

let lastBotherTime = 0.0;
function versionCheck(currentVersionText) {
  $.get("https://raw.githubusercontent.com/dheera/rosboard/release/setup.py").done((data) => {
    let matches = data.match(/version='(.*)'/);
    if(matches.length < 2) return;
    let latestVersion = matches[1].split(".").map(num => parseInt(num, 10));
    let currentVersion = currentVersionText.split(".").map(num => parseInt(num, 10));
    let latestVersionInt = latestVersion[0] * 1000000 + latestVersion[1] * 1000 + latestVersion[2];
    let currentVersionInt = currentVersion[0] * 1000000 + currentVersion[1] * 1000 + currentVersion[2];
    if(currentVersion < latestVersion && Date.now() - lastBotherTime > 1800000) {
      lastBotherTime = Date.now();
      snackbarContainer.MaterialSnackbar.showSnackbar({
        message: "New version of ROSboard available (" + currentVersionText + " -> " + matches[1] + ").",
        actionText: "Check it out",
        actionHandler: ()=> {window.location.href="https://github.com/dheera/rosboard/"},
      });
    }
  });
}

$(() => {
  if(window.location.href.indexOf("rosboard.com") === -1) {
    initDefaultTransport();
  }
  
  // Initialize publish dialog functionality
  initPublishDialog();
});

// Publishing variables
let publishInterval = null;
let isPublishing = false;

// Message templates for different types
const messageTemplates = {
  'std_msgs/String': { data: "Hello World" },
  'std_msgs/Int32': { data: 42 },
  'std_msgs/Float32': { data: 3.14 },
  'std_msgs/Bool': { data: true },
  'geometry_msgs/Twist': {
    linear: { x: 0.0, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 }
  },
  'geometry_msgs/Point': { x: 0.0, y: 0.0, z: 0.0 },
  'geometry_msgs/Pose': {
    position: { x: 0.0, y: 0.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  },
  'sensor_msgs/Joy': {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" },
    axes: [0.0, 0.0],
    buttons: [0, 0]
  },
  'nav_msgs/OccupancyGrid': {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
    info: {
      map_load_time: { sec: 0, nanosec: 0 },
      resolution: 0.05,
      width: 100,
      height: 100,
      origin: {
        position: { x: 0.0, y: 0.0, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    },
    data: []
  }
};

function initPublishDialog() {
  const dialog = document.getElementById('publish-dialog');
  const publishLink = document.getElementById('publish-link');
  const publishOnceBtn = document.getElementById('publish-once-btn');
  const publishStartBtn = document.getElementById('publish-start-btn');
  const publishStopBtn = document.getElementById('publish-stop-btn');
  const publishCancelBtn = document.getElementById('publish-cancel-btn');
  const continuousCheckbox = document.getElementById('publish-continuous');
  const rateContainer = document.getElementById('publish-rate-container');
  const loadTemplateLink = document.getElementById('load-template-link');
  
  // Initialize Material Design components
  dialog.addEventListener('click', function(event) {
    if (event.target === dialog) {
      dialog.close();
    }
  });

  // Open dialog
  publishLink.addEventListener('click', function() {
    // Close the left navigation drawer
    const layout = document.querySelector('.mdl-layout');
    if (layout && layout.MaterialLayout) {
      layout.MaterialLayout.toggleDrawer();
    } else {
      // Fallback method - remove is-visible class
      const drawer = document.querySelector('.mdl-layout__drawer');
      const overlay = document.querySelector('.mdl-layout__obfuscator');
      if (drawer) drawer.classList.remove('is-visible');
      if (overlay) overlay.classList.remove('is-visible');
    }
    
    // Clear the dialog when opened from navigation (not pre-populated)
    clearPublishDialog();
    
    dialog.showModal();
  });

  // Close dialog
  publishCancelBtn.addEventListener('click', function() {
    stopPublishing();
    clearPublishDialog();
    dialog.close();
  });

  // Handle continuous publishing checkbox
  continuousCheckbox.addEventListener('change', function() {
    if (this.checked) {
      rateContainer.style.display = 'block';
      publishOnceBtn.style.display = 'none';
      publishStartBtn.style.display = 'inline-block';
    } else {
      rateContainer.style.display = 'none';
      publishOnceBtn.style.display = 'inline-block';
      publishStartBtn.style.display = 'none';
      publishStopBtn.style.display = 'none';
      stopPublishing();
    }
  });

  // Publish once
  publishOnceBtn.addEventListener('click', function() {
    const success = publishMessage();
    if (success) {
      // Close dialog after a brief delay to show success message
      setTimeout(() => {
        document.getElementById('publish-dialog').close();
      }, 1500);
    }
  });

  // Start continuous publishing
  publishStartBtn.addEventListener('click', function() {
    startContinuousPublishing();
  });

  // Stop continuous publishing  
  publishStopBtn.addEventListener('click', function() {
    stopPublishing();
  });

  // Load message template
  loadTemplateLink.addEventListener('click', function(e) {
    e.preventDefault();
    loadMessageTemplate();
  });
}

function publishMessage() {
  const topicName = document.getElementById('publish-topic-name').value.trim();
  const topicType = document.getElementById('publish-topic-type').value.trim();
  const msgData = document.getElementById('publish-msg-data').value.trim();

  // Validation
  if (!topicName) {
    showPublishStatus('Please enter a topic name', 'error');
    return false;
  }

  if (!topicType) {
    showPublishStatus('Please enter a message type', 'error');
    return false;
  }

  if (!msgData) {
    showPublishStatus('Please enter message data', 'error');
    return false;
  }

  try {
    const msg = JSON.parse(msgData);
    currentTransport.publish({
      topicName: topicName,
      topicType: topicType,
      msg: msg
    });
    
    showPublishStatus(`Published to ${topicName}`, 'success');
    console.log(`Published message to ${topicName}: ${msgData}`);
    return true;
  } catch (e) {
    showPublishStatus('Invalid JSON format: ' + e.message, 'error');
    return false;
  }
}

function startContinuousPublishing() {
  if (isPublishing) return;

  const rate = parseFloat(document.getElementById('publish-rate').value) || 1.0;
  const interval = 1000 / rate; // Convert Hz to milliseconds

  publishMessage(); // Publish immediately
  
  publishInterval = setInterval(() => {
    publishMessage();
  }, interval);
  
  isPublishing = true;
  document.getElementById('publish-start-btn').style.display = 'none';
  document.getElementById('publish-stop-btn').style.display = 'inline-block';
  
  showPublishStatus(`Publishing at ${rate} Hz...`, 'success');
}

function stopPublishing() {
  if (publishInterval) {
    clearInterval(publishInterval);
    publishInterval = null;
  }
  
  isPublishing = false;
  
  // Only show start button if continuous publishing is still enabled
  const continuousCheckbox = document.getElementById('publish-continuous');
  if (continuousCheckbox && continuousCheckbox.checked) {
    document.getElementById('publish-start-btn').style.display = 'inline-block';
  } else {
    document.getElementById('publish-start-btn').style.display = 'none';
  }
  document.getElementById('publish-stop-btn').style.display = 'none';
  
  removePublishStatus();
}

function loadMessageTemplate() {
  const topicType = document.getElementById('publish-topic-type').value.trim();
  const msgDataField = document.getElementById('publish-msg-data');
  
  if (!topicType) {
    showPublishStatus('Please enter a message type first', 'error');
    return;
  }
  
  const template = messageTemplates[topicType];
  if (template) {
    msgDataField.value = JSON.stringify(template, null, 2);
    msgDataField.parentElement.classList.add('is-dirty');
    showPublishStatus(`Loaded template for ${topicType}`, 'success');
  } else {
    showPublishStatus(`No template available for ${topicType}`, 'error');
  }
}

function showPublishStatus(message, type) {
  removePublishStatus();
  
  const statusDiv = document.createElement('div');
  statusDiv.className = `publish-status ${type}`;
  statusDiv.textContent = message;
  statusDiv.id = 'publish-status-message';
  
  const dialogContent = document.querySelector('#publish-dialog .mdl-dialog__content');
  dialogContent.appendChild(statusDiv);
  
  if (type === 'success') {
    setTimeout(() => {
      removePublishStatus();
    }, 3000);
  }
}

function removePublishStatus() {
  const existingStatus = document.getElementById('publish-status-message');
  if (existingStatus) {
    existingStatus.remove();
  }
}

function clearPublishDialog() {
  // Clear all form fields
  document.getElementById('publish-topic-name').value = '';
  document.getElementById('publish-topic-type').value = '';
  document.getElementById('publish-msg-data').value = '';
  
  // Clear Material Design textfield states
  const fields = ['publish-topic-name', 'publish-topic-type', 'publish-msg-data'];
  fields.forEach(fieldId => {
    const field = document.getElementById(fieldId).parentElement;
    if (field) field.classList.remove('is-dirty');
  });
  
  // Reset to single publish mode
  const continuousCheckbox = document.getElementById('publish-continuous');
  if (continuousCheckbox) {
    continuousCheckbox.checked = false;
    document.getElementById('publish-rate-container').style.display = 'none';
    document.getElementById('publish-once-btn').style.display = 'inline-block';
    document.getElementById('publish-start-btn').style.display = 'none';
    document.getElementById('publish-stop-btn').style.display = 'none';
  }
  
  // Stop any ongoing publishing
  stopPublishing();
  
  // Remove status messages
  removePublishStatus();
}

Viewer.onClose = function(viewerInstance) {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  currentTransport.unsubscribe({topicName:topicName});
  $grid.masonry("remove", viewerInstance.card);
  $grid.masonry("layout");
  delete(subscriptions[topicName].viewer);
  delete(subscriptions[topicName]);
  updateStoredSubscriptions();
}

Viewer.onSwitchViewer = (viewerInstance, newViewerType) => {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  if(!subscriptions[topicName].viewer === viewerInstance) console.error("viewerInstance does not match subscribed instance");
  let card = subscriptions[topicName].viewer.card;
  subscriptions[topicName].viewer.destroy();
  delete(subscriptions[topicName].viewer);
  subscriptions[topicName].viewer = new newViewerType(card, topicName, topicType);
};


