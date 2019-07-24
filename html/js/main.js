"use strict";

// constantly-updated list of topics
// e.g. [ '/imu/data', '/motor/status' ]
var topics = [];

// caches topic types
// e.g. { '/imu/data': 'sensor_msgs/Imu' }
var topicTypesByTopic = {};

// Viewer classes by the types they handle
// (NOT instances of viewers)
// e.g. { '/imu/data': class ImuViewer ... }
var viewersByType = { };

// instances of Viewers by topic type
// e.g. { 'sensor_msgs/Imu': <ImuViewer> }
var viewerInstancesByType = {};

// instances of Viewers by topic
// e.g. { '/imu/data': <ImuViewer> }
var viewerInstancesByTopic = {};

// paths of already imported JS/CSS
var importedPaths = {};

var ros = new ROSLIB.Ros({
  url: 'ws://' + window.location.hostname + ':9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

/**
  * Updates the list of ROS topics.
  * Called by an interval at a fixed frequency.
**/
function updateTopics() {
  if(!ros.isConnected) return;
  ros.getTopics(function(result) {
    $('#topics-nav-supported').empty();
    $('#topics-nav-unsupported').empty();
    result.topics.sort();
    $.each(result.topics, function(i, topic) {
      if(!(topic in topicTypesByTopic)) {
        // we don't know what type the topic is yet; fetch and cache the topic type
        topicTypesByTopic[topic] = 'unknown';
        ros.getTopicType(topic, function(result) {
          topicTypesByTopic[topic] = result;
        });
      }
      if(topic in topicTypesByTopic && topicTypesByTopic[topic] in viewerModules) {
        // if we have the type cached, and if a viewer exists for it, create a link to view it.
        $('<a class="mdl-navigation__link" href="#" onclick="topicToggle(\'' + topic + '\');return false;">' + topic + '</a>').appendTo($('#topics-nav-supported'));
        importJsOnce(viewerModulesPath + viewerModules[topicTypesByTopic[topic]]);
      } else {
        // else display it in the unsupported list
        $('<a class="mdl-navigation__link" href="#" onclick="topicToggle(\'' + topic + '\');return false;">' + topic + '</a>').appendTo($('#topics-nav-unsupported'));
      }
    });
  });
}

var $grid = null;
function updateGrid() {
  $grid = $('.grid').packery({
    itemSelector: '.card',
    gutter: 10
  });
}

$(function() {
  setInterval(updateTopics, 10000);
  updateTopics();
  updateGrid();
});

function addViewer(viewer) {
  viewersByType[viewer._TYPE] = viewer;
}

function getViewerInstanceForType(topicType) {
  if(!topicType in viewersByType) {
    console.log('Unsupported type: ' + topicType);
    return;
  }
  if(!(topicType in viewerInstancesByType)) {
    var card = $('<div></div>')
      .addClass('card')
      .addClass('viewer-' + topicType.replace('/', '-'))
      .data('type', topicType)
      .appendTo($('.grid'));
    var cardTitle = $('<div></div>')
      .addClass('card-title')
      .text(topicType.substring(topicType.indexOf('/') + 1))
      .data('type', topicType)
      .appendTo(card);
    var cardContent = $('<div></div>')
      .addClass('card-content')
      .data('type', topicType)
      .appendTo(card);
    var viewer = new viewersByType[topicType](cardContent);
    viewer.onCreate();
    viewer._CARD = card;
    if(!viewersByType[topicType]._SINGLE) {
      viewerInstancesByType[topicType] = viewer;
    }
    $grid.packery('appended', card);
    updateGrid();
    return viewer;
  }
  return viewerInstancesByType[topicType];
}

function topicToggle(topic) {
  console.log("topicToggle", topic);
  if(topic in viewerInstancesByTopic) {
    viewerInstancesByTopic[topic].removeTopic(topic);
    viewerInstancesByTopic[topic]._CARD.children('.card-title').children(':contains(' + topic + ')').remove();
    if(viewerInstancesByTopic[topic].getTopics().length === 0) {
      viewerInstancesByTopic[topic].onDestroy();
      viewerInstancesByTopic[topic]._CARD.remove();
      for(var topicType in viewerInstancesByType) {
        if(viewerInstancesByType[topicType] === viewerInstancesByTopic[topic]) {
          delete(viewerInstancesByType[topicType]);
        }
      }
      updateGrid();
    }
    delete(viewerInstancesByTopic[topic]);
  } else {
    ros.getTopicType(topic, function(topicType) {
      if(topicType in viewersByType) {
        viewerInstancesByTopic[topic] = getViewerInstanceForType(topicType);
        viewerInstancesByTopic[topic].addTopic(topic);
        $('<div></div>')
          .text(topic)
          .addClass('card-title-tag')
          .appendTo(viewerInstancesByTopic[topic]._CARD.children('.card-title'));
      } else {
        console.log('topic type not supported: ' + topicType);
      }
    });
  }
}

window.onresize = function() {
  scheduleResize();
}

var resizeTimeout = null;
function scheduleResize() {
  if(resizeTimeout) clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(function() {
    for(var topic in viewerInstancesByTopic) {
      // TODO: avoid doubly-resizing viewers
      viewerInstancesByTopic[topic].onResize();
    }
  }, 200);
}

function resetListeners() {
  ros.removeAllListeners();
}

function importCssOnce(path) {
  if(path in importedPaths) return;
  $('<link>').appendTo('head').attr({
    type: 'text/css',
    rel: 'stylesheet',
    href: path
  });
  importedPaths[path] = 1;
}

function importJsOnce(path) {
  if(path in importedPaths) return;
  var result = $.ajax({ url: path, dataType: "script", async: false })
  if(result.status === 200) {
    importedPaths[path] = 1;
  } else {
    console.log(result.status + " error while importing " + path);
  }
}

var viewerModulesPath = '/js/viewers/';
var viewerModules = {
  "geometry_msgs/Twist": "geometry_msgs/TwistViewer.js",
  "geometry_msgs/Point": "geometry_msgs/PointViewer.js",
  "nav_msgs/Odometry": "nav_msgs/OdometryViewer.js",
  "robby_msgs/Int8Pair": "robby_msgs/Int8PairViewer.js",
  "robby_msgs/Int16Pair": "robby_msgs/Int16PairViewer.js",
  "robby_msgs/Int32Pair": "robby_msgs/Int32PairViewer.js",
  "robby_msgs/Int64Pair": "robby_msgs/Int64PairViewer.js",
  "robby_msgs/Float32Pair": "robby_msgs/Float32PairViewer.js",
  "robby_msgs/Float64Pair": "robby_msgs/Float64PairViewer.js",
  "robby_msgs/MotorCounts": "robby_msgs/MotorCountsViewer.js",
  "robby_msgs/MotorVelocity": "robby_msgs/MotorVelocityViewer.js",
  "robby_msgs/Beep": "robby_msgs/BeepViewer.js",
  "std_msgs/Duration": "std_msgs/DurationViewer.js",
  "std_msgs/Float32": "std_msgs/Float32Viewer.js",
  "std_msgs/Int8": "std_msgs/Int8Viewer.js",
  "std_msgs/Float64": "std_msgs/Float64Viewer.js",
  "std_msgs/UInt64": "std_msgs/UInt64Viewer.js",
  "std_msgs/Int64": "std_msgs/Int64Viewer.js",
  "std_msgs/Bool": "std_msgs/BoolViewer.js",
  "std_msgs/Int32": "std_msgs/Int32Viewer.js",
  "std_msgs/Int16": "std_msgs/Int16Viewer.js",
  "std_msgs/UInt16": "std_msgs/UInt16Viewer.js",
  "std_msgs/UInt8": "std_msgs/UInt8Viewer.js",
  "std_msgs/String": "std_msgs/StringViewer.js",
  "std_msgs/UInt32": "std_msgs/UInt32Viewer.js",
  "std_msgs/Time": "std_msgs/TimeViewer.js",
  "std_msgs/Char": "std_msgs/CharViewer.js",
  "sensor_msgs/CameraInfo": "sensor_msgs/CameraInfoViewer.js",
  "sensor_msgs/Temperature": "sensor_msgs/TemperatureViewer.js",
  "sensor_msgs/LaserScan": "sensor_msgs/LaserScanViewer.js",
  "sensor_msgs/MagneticField": "sensor_msgs/MagneticFieldViewer.js",
  "sensor_msgs/Imu": "sensor_msgs/ImuViewer.js",
  "sensor_msgs/Illuminance": "sensor_msgs/IlluminanceViewer.js",
  "sensor_msgs/Range": "sensor_msgs/RangeViewer.js",
  "sensor_msgs/RelativeHumidity": "sensor_msgs/RelativeHumidity.js",
  "sensor_msgs/Image": "sensor_msgs/ImageViewer.js",
  "sensor_msgs/PointCloud2": "sensor_msgs/PointCloud2Viewer.js",
  "sensor_msgs/NavSatFix": "sensor_msgs/NavSatFixViewer.js",
}
