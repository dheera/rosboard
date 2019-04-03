'use strict';

let rosnodejs = require('../index.js');
const std_msgs = rosnodejs.require('std_msgs').msg;
const SetBool = rosnodejs.require('std_srvs').srv.SetBool;

rosnodejs.initNode('/test_node').then(rosNode => {
  // EXP 1) Service Server
  let service = rosNode.advertiseService('set_bool', SetBool, (req, resp) => {
    rosnodejs.log.info('Handling request! ' + JSON.stringify(req));
    resp.success = !req.data;
    resp.message = 'Inverted!';
    return true;
  });

  // EXP 2) Service Client
  let serviceClient = rosNode.serviceClient('set_bool', 'std_srvs/SetBool', { persist: true });
  rosNode.waitForService(serviceClient.getService(), 2000).then(available => {
    if (available) {
      const request = new SetBool.Request();
      request.data = true;
      serviceClient.call(request).then(resp => {
        rosnodejs.log.info('Service response ' + JSON.stringify(resp));
      }).then(() => {
        request.data = false;
        serviceClient.call(request).then(resp => {
          rosnodejs.log.info('Service response 2 ' + JSON.stringify(resp));
        });
      }).then(() => {
        let serviceClient2 = rosNode.serviceClient('set_bool', 'std_srvs/SetBool');
        serviceClient2.call(request).then(resp => {
          rosnodejs.log.info('Non persistent response ' + JSON.stringify(resp));
        });
      });
    }
  });

  // EXP 3) Params
  rosNode.setParam('junk', { 'hi': 2 }).then(() => {
    rosNode.getParam('junk').then(val => {
      rosnodejs.log.info('Got Param!!! ' + JSON.stringify(val));
    });
  });

  // EXP 4) Publisher
  let pub = rosNode.advertise('my_topic', std_msgs.String, {
    queueSize: 1,
    latching: true,
    throttleMs: 9
  });

  let msgStart = 'my message ';
  let iter = 0;
  const msg = new std_msgs.String();
  setInterval(() => {
    msg.data = msgStart + iter;
    pub.publish(msg);
    ++iter;
    if (iter > 200) {
      iter = 0;
    }
  }, 5);

  // EXP 5) Subscriber
  let sub = rosNode.subscribe('my_topic', 'std_msgs/String', data => {
    rosnodejs.log.info('SUB DATA ' + data.data);
  }, {
    queueSize: 1,
    throttleMs: 1000
  });
}).catch(err => {
  rosnodejs.log.error(err.stack);
});