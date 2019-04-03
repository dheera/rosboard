'use strict';

let rosnodejs = require('../index.js');

rosnodejs.initNode('/my_node', { onTheFly: true }).then(rosNode => {

  const std_msgs = rosnodejs.require('std_msgs').msg;
  const msg = new std_msgs.String();

  const SetBool = rosnodejs.require('std_srvs').srv.SetBool;
  const request = new SetBool.Request();

  // EXP 1) Service Server
  let service = rosNode.advertiseService('/set_bool', 'std_srvs/SetBool', (req, resp) => {
    console.log('Handling request! ' + JSON.stringify(req));
    resp.success = !req.data;
    resp.message = 'Inverted!';
    return true;
  });

  // EXP 2) Service Client
  setTimeout(function () {
    let serviceClient = rosNode.serviceClient('/set_bool', 'std_srvs/SetBool');
    rosNode.waitForService(serviceClient.getService(), 2000).then(available => {
      if (available) {
        const request = new SetBool.Request();
        request.data = true;
        serviceClient.call(request).then(resp => {
          console.log('Service response ' + JSON.stringify(resp));
        });
      } else {
        console.log('Service not available');
      }
    });
  }, 1000); // wait a second before calling our service

  // EXP 3) Params
  rosNode.setParam('~junk', { 'hi': 2 }).then(() => {
    rosNode.getParam('~junk').then(val => {
      console.log('Got Param!!! ' + JSON.stringify(val));
    });
  });

  // // EXP 4) Publisher
  let pub = rosNode.advertise('/my_topic', 'std_msgs/String', {
    queueSize: 1,
    latching: true,
    throttleMs: 9
  });

  // EXP 5) Subscriber
  let sub = rosNode.subscribe('/my_topic', 'std_msgs/String', data => {
    console.log('SUB DATA ', data, data.data);
  }, { queueSize: 1,
    throttleMs: 1000 });

  let msgStart = 'my message ';
  let iter = 0;
  setInterval(() => {
    msg.data = msgStart + iter;
    pub.publish(msg);
    ++iter;
    if (iter > 200) {
      iter = 0;
    }
  }, 5);
});