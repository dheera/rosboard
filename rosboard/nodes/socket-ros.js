"use strict";

const uuidv4 = require('uuid/v4');

module.exports = (app) => {

app.io.of('/ros').on('connection', (socket) => {

  socket.subscribers = {};
  socket.publishers = {};

  socket.on('connect', () => {
    ros.log.info('connection from ' + socket.addr);
  });

  socket.on('subscribe', (topicName, topicType, callbackMsg, callbackSubscriberId) => {
    if(!topicName || typeof(topicName) !== "string") return;
    if(!topicType || typeof(topicType) !== "string") return;
    if(!callback) return;
    if(!app.nh) { callback(null); return; }

    let subscriberId = uuidv4().replace(/-/g,'');
    subscribers[subscriberId] = app.nh.subscribe(topicName, topicType, (msg) => {
      msg._topicName = topicName;
      msg._topicType = topicType;
      msg._time = Date.now();
      callbackMsg(msg);
    });
    callbackSubscriberId(subscriberId);
  });

  socket.on('unsubscribe', (subscriberId) => {
    delete(socket.subscribers[subscriberId]);
  });


  socket.on('disconnect', () => {
    for(subscriberId in socket.subscribers) {
      delete(socket.subscribers[subscriberId]);
    }
    for(publisherId in socket.publishers) {
      delete(socket.publishers[publisherId]);
    }
  });
});

}
