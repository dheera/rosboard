'use strict'

const net = require('net');
const chai = require('chai');
const expect = chai.expect;
const rosnodejs = require('../src/index.js');
const Subscriber = require('../src/lib/Subscriber.js');
const SubscriberImpl = require('../src/lib/impl/SubscriberImpl.js');
const xmlrpc = require('xmlrpc');
const netUtils = require('../src/utils/network_utils.js');
const MasterStub = require('./utils/MasterStub.js');

const MASTER_PORT = 11234;

// helper function to throw errors outside a promise scope
// so they actually trigger failures
function throwNext(msg) {
  process.nextTick(() => { throw new Error(msg)});
}

describe('Protocol Test', () => {
  // NOTE: make sure a roscore is not running (or something else at this address)
  rosnodejs.require('std_msgs');
  rosnodejs.require('std_srvs');
  let masterStub;
  const initArgs = {
    rosMasterUri: `http://localhost:${MASTER_PORT}`,
    logging: {skipRosLogging: true},
    notime: true
  };
  const nodeName = '/testNode';

  function startMasterStub(done) {
    masterStub = xmlrpc.createServer({host: 'localhost', port: MASTER_PORT}, () => { done(); });
  }

  function stopMasterStub(done) {
    masterStub.close(() => { done(); });
  }

  function clearOutClients(node) {
    Object.keys(node._services).forEach((service) => {
      node._services[service].disconnect();
    });

    Object.keys(node._subscribers).forEach((sub) => {
      node._subscribers[sub].shutdown();
    });

    Object.keys(node._publishers).forEach((pub) => {
      node._publishers[pub].shutdown();
    });

    node._services = {};
    node._subscribers = {};
    node._publishers = {};
  }

  before((done) => {
    startMasterStub(() => {
      rosnodejs.initNode(nodeName, initArgs)
      .then(() => {
        done()
      });
    });

    masterStub.on('getUri', (err, params, callback) => {
      const resp = [ 1, '', `localhost:${MASTER_PORT}/` ];
      callback(null, resp);
    });
  });

  after((done) => {
    stopMasterStub(() => {
      rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
        done();
      })
    });
  });

  describe('Xmlrpc', () => {
    afterEach(() => {
      const nh = rosnodejs.nh;

      // clear out any service, subs, pubs
      clearOutClients(nh._node);

      nh._node._spinner.clear();

      // remove any master api handlers we set up
      masterStub.removeAllListeners();
    });

    it('registerSubscriber', (done) => {
      const topic = '/test_topic';
      const msgType = 'std_msgs/String';
      masterStub.on('registerSubscriber', (err, params, callback) => {
        expect(params.length).to.equal(4);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(topic);
        expect(params[2]).to.equal(msgType);
        expect(params[3].startsWith('http://')).to.be.true;

        const info = netUtils.getAddressAndPortFromUri(params[3]);
        expect(info.host).to.be.a('string');
        expect(info.host.length).to.not.equal(0);
        expect(info.port).to.be.a('string');
        expect(info.port.length).to.not.equal(0);

        const resp = [ 1, 'registered!', [] ];
        callback(null, resp);
      });

      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, msgType,
        (data) => {},
        { queueSize: 1, throttleMs: 1000 }
      );

      sub.on('registered', () => {
        done();
      })
    });

    it('unregisterSubscriber', (done) => {
      const topic = '/test_topic';
      const msgType = 'std_msgs/String';
      let nodeUri;

      masterStub.on('registerSubscriber', (err, params, callback) => {
        nodeUri = params[3];

        const resp = [ 1, 'registered!', [] ];
        callback(null, resp);
      });

      masterStub.on('unregisterSubscriber', (err, params, callback) => {
        expect(params.length).to.equal(3);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(topic);
        expect(params[2]).to.equal(nodeUri);

        const resp = [ 1, 'unregistered!', [] ];
        callback(null, resp);
        done();
      });

      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, msgType,
        (data) => {},
        { queueSize: 1, throttleMs: 1000 }
      );

      sub.on('registered', () => {
        nh.unsubscribe(topic);
      });
    });

    it('registerPublisher', (done) => {
      const topic = '/test_topic';
      const msgType = 'std_msgs/String';
      masterStub.on('registerPublisher', (err, params, callback) => {
        expect(params.length).to.equal(4);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(topic);
        expect(params[2]).to.equal(msgType);
        expect(params[3].startsWith('http://')).to.be.true;

        const info = netUtils.getAddressAndPortFromUri(params[3]);
        expect(info.host).to.be.a('string');
        expect(info.host.length).to.not.equal(0);
        expect(info.port).to.be.a('string');
        expect(info.port.length).to.not.equal(0);

        const resp = [ 1, 'registered!', [] ];
        callback(null, resp);
        done();
      });

      const nh = rosnodejs.getNodeHandle();
      const pub = nh.advertise(topic, msgType, { latching: true,
                                                 queueSize: 1,
                                                 throttleMs: 1000 }
      );
    });

    it('unregisterPublisher', (done) => {
      const topic = '/test_topic';
      const msgType = 'std_msgs/String';
      let nodeUri;

      masterStub.on('registerPublisher', (err, params, callback) => {
        nodeUri = params[3];

        const resp = [ 1, 'registered!', [] ];
        callback(null, resp);
      });

      masterStub.on('unregisterPublisher', (err, params, callback) => {
        expect(params.length).to.equal(3);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(topic);
        expect(params[2]).to.equal(nodeUri);

        const resp = [ 1, 'unregistered!', [] ];
        callback(null, resp);
        done();
      });

      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType, { latching: true,
                                                 queueSize: 1,
                                                 throttleMs: 1000 }
      );

      pub.on('registered', () => {
        nh.unadvertise(topic);
      });
    });

    it('registerService', (done) => {
      const service = '/test_service';
      const srvType = 'std_srvs/Empty';
      masterStub.on('registerService', (err, params, callback) => {
        expect(params.length).to.equal(4);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(service);
        expect(params[2].startsWith('rosrpc://')).to.be.true;

        let info = netUtils.getAddressAndPortFromUri(params[2]);
        expect(info.host).to.be.a('string');
        expect(info.host.length).to.not.equal(0);
        expect(info.port).to.be.a('string');
        expect(info.port.length).to.not.equal(0);

        expect(params[3].startsWith('http://')).to.be.true;

        info = netUtils.getAddressAndPortFromUri(params[3]);
        expect(info.host).to.be.a('string');
        expect(info.host.length).to.not.equal(0);
        expect(info.port).to.be.a('string');
        expect(info.port.length).to.not.equal(0);

        const resp = [ 1, 'registered!', [] ];
        callback(null, resp);
      });

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {});
      serv.on('registered', done);
    });

    it('unregisterService', (done) => {
      const service = '/test_service';
      const srvType = 'std_srvs/Empty';
      let serviceUri = null;
      masterStub.on('registerService', (err, params, callback) => {
        serviceUri = params[2];

        const resp = [1, 'registered!', ''];
        callback(null, resp);
      });

      masterStub.on('unregisterService', (err, params, callback) => {
        expect(params.length).to.equal(3);
        expect(params[0]).to.equal(nodeName);
        expect(params[1]).to.equal(service);
        expect(params[2]).to.equal(serviceUri);

        const resp = [ 1, 'unregistered!', [] ];
        callback(null, resp);
        done();
      });

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {});

      serv.on('registered', () => {
        nh.unadvertiseService(service);
      });
    });
  });

  describe('Pub-Sub', () => {
    const topic = '/test_topic';
    const msgType = 'std_msgs/Int8';

    beforeEach(() => {
      let pubInfo = null;
      let subInfo = null;

      masterStub.on('getUri', (err, params, callback) => {
        const resp = [ 1, '', 'localhost:11311/' ]
        callback(null, resp);
      });

      masterStub.on('registerSubscriber', (err, params, callback) => {
        subInfo = params[3];
        //console.log('sub reg ' + params);
        //console.log(pubInfo);

        const resp =  [1, 'You did it!', []];
        if (pubInfo) {
          resp[2].push(pubInfo);
        }
        callback(null, resp);
      });

      masterStub.on('unregisterSubscriber', (err, params, callback) => {
        const resp =  [1, 'You did it!', subInfo ? 1 : 0];
        callback(null, resp);
        subInfo = null;
      });

      masterStub.on('registerPublisher', (err, params, callback) => {
        //console.log('pub reg');
        pubInfo = params[3];
        const resp =  [1, 'You did it!', []];
        if (subInfo) {
          resp[2].push(pubInfo);
          let subAddrParts = subInfo.replace('http://', '').split(':');
          let client = xmlrpc.createClient({host: subAddrParts[0], port: subAddrParts[1]});
          let data = [1, topic, [pubInfo]];
          client.methodCall('publisherUpdate', data, (err, response) => { });
        }
        callback(null, resp);
      });

      masterStub.on('unregisterPublisher', (err, params, callback) => {
        const resp =  [1, 'You did it!', pubInfo ? 1 : 0];
        callback(null, resp);
        pubInfo = null;
      });

      return rosnodejs.initNode(nodeName, initArgs);
    });

    afterEach(() => {
      const nh = rosnodejs.nh;

      // clear out any service, subs, pubs
      clearOutClients(nh._node);

      nh._node._spinner.clear();

      // remove any master api handlers we set up
      masterStub.removeAllListeners();
    });

    it('Basic', (done) => {
      const nh = rosnodejs.nh;
      const valsToSend = [1,2,3];
      const valsReceived = new Set(valsToSend);
      const pub = nh.advertise(topic, msgType, { queueSize: 3 });

      const sub = nh.subscribe(topic, msgType, (data) => {
        valsReceived.delete(data.data);
        if (valsReceived.size === 0) {
          done();
        }
      }, {queueSize: 3});

      pub.on('connection', () => {
        valsToSend.forEach((val) => {
          pub.publish({data: val});
        });
      });
    });

    it('Latch', (done) => {
      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType, { latching: true });

      pub.publish({data: 1});

      pub.on('registered', () => {
        const sub = nh.subscribe(topic, msgType, (data) => {
          done();
        });
      });
    });

    it('Invalid Without Resolve Causes Error', (done) => {
      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, 'std_msgs/String');

      // NOTE: you'll see an error logged here - THAT'S OK
      // WE'RE EXPECTING AN ERROR TO LOG
      const logCapture = {
        write(rec) {
          if (rec.level === rosnodejs.log.levelFromName['error'] &&
              rec.msg.startsWith('Error when publishing'))
          {
            done();
          }
        }
      };

      rosnodejs.log.addStream({
        type: 'raw',
        name: 'testCapture',
        stream: logCapture,
        level: 'error'
      });

      sub.on('registered', () => {
        const pub = nh.advertise(topic, 'std_msgs/String', {latching: true});

        pub.on('connection', () => {
          pub.publish({});
        });

        pub.on('error', (err) => {
          nh._node._spinner._queueLocked = false;
        });
      });
    });

    it('Resolve', (done) => {
      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, 'std_msgs/String', (data) => {
        done();
      });

      sub.on('registered', () => {
        const pub = nh.advertise(topic, 'std_msgs/String', { latching: true, resolve: true });

        pub.on('registered', () => {
          pub.publish({});
        });
      });
    });

    it('Throttle Pub', function(done) {
      this.slow(1000);

      const nh = rosnodejs.nh;
      const valsToSend = [1,2,3,4,5,6,7,8,9,10];
      const pub = nh.advertise(topic, msgType, { queueSize: 1, throttleMs: 100});
      let numMsgsReceived = 0;

      const sub = nh.subscribe(topic, msgType, (data) => {
        ++numMsgsReceived;
        if (data.data === valsToSend[valsToSend.length -1]) {
          expect(numMsgsReceived).to.equal(valsToSend.length/2 + 1);
          done();
        }
      }, {queueSize: 1});

      pub.on('connection', () => {
        valsToSend.forEach((val, index) => {
          setTimeout(() => {
            pub.publish({data: val});
          }, 50*index);
        });
      });
    });

    it('Disconnect Pub', (done) => {
      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType);
      const sub = nh.subscribe(topic, msgType, (data) => {
        expect(pub.getNumSubscribers()).to.equal(1);
        expect(sub.getNumPublishers()).to.equal(1);

        pub.shutdown();

        expect(pub.getNumSubscribers()).to.equal(0);
        sub.on('disconnect', () => {
          expect(sub.getNumPublishers()).to.equal(0);
          done()
        });
      });

      pub.on('connection', () => { pub.publish({data: 1}); });
    });

    it('Disconnect Sub', (done) => {
      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType);
      const sub = nh.subscribe(topic, msgType, (data) => {
        expect(pub.getNumSubscribers()).to.equal(1);
        expect(sub.getNumPublishers()).to.equal(1);

        sub.shutdown();

        expect(sub.getNumPublishers()).to.equal(0);
        pub.on('disconnect', () => {
          expect(pub.getNumSubscribers()).to.equal(0);
          done()
        });
      });

      pub.on('connection', () => { pub.publish({data: 1}); });
    });

    it('Shutdown Subscriber During Registration', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, msgType);

      sub.on('registered', () => {
        throwNext('Subscriber should never have registered!');
      });

      sub.shutdown();

      // if we haven't seen the 'registered' event by now we should be good
      setTimeout(done, 500);
    });

    it('Shutdown Subscriber Requesting Topic', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType);

      pub.on('registered', () => {
        const sub = nh.subscribe(topic, msgType);
        sub.on('registered', () => {
          sub.shutdown();
        });
        sub.on('connection', () => {
          throwNext('Sub should not have gotten connection');
        })
      });

      // if we haven't seen thrown by now we should be good
      setTimeout(done, 500);
    });

    it('Shutdown Subscriber Connecting to Publisher', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      // manually construct a subscriber...
      const subImpl = new SubscriberImpl({
          topic,
          type: 'std_msgs/String',
          typeClass: rosnodejs.require('std_msgs').msg.String
        },
        nh._node);

      const sub = new Subscriber(subImpl);

      const SOCKET_CONNECT_CACHED = net.Socket.prototype.connect;
      const SOCKET_END_CACHED = net.Socket.prototype.end;

      sub.on('registered', () => {

        net.Socket.prototype.connect = function(port, address, callback) {
          process.nextTick(() => {
            callback();
          });
        };

        net.Socket.prototype.end = function() {
          process.nextTick(() => {
            net.Socket.prototype.connect = SOCKET_CONNECT_CACHED;
            net.Socket.prototype.end = SOCKET_END_CACHED;

            done();
          });

          // even though we didn't actually connect, this socket seems to make
          // the suite hang unless we call the actual Socket.prototype.end()
          SOCKET_END_CACHED.call(this);
        };

        subImpl._handleTopicRequestResponse([1, 'ok', ['TCPROS', 'junk_address', 1234]], 'http://junk_address:1234');
        sub.shutdown();
      });
    });

    it('Shutdown Publisher During Registration', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      const pub = nh.advertise(topic, msgType);

      pub.on('registered', () => {
        throwNext('Publisher should never have registered!');
      });

      pub.shutdown();

      // if we haven't seen the 'registered' event by now we should be good
      setTimeout(done, 500);
    });

    it('Shutdown Publisher With Queued Message', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, msgType, () => {
        throwNext('Subscriber should never have gotten messages!');
      });
      let pub = nh.advertise(topic, msgType);

      pub.on('connection', () => {
        pub.publish({data: 1});
        pub.shutdown();
      });

      // if we haven't received a message by now we should be good
      setTimeout(done, 500);
    });

    it('Shutdown Subscriber With Pending Publisher Client', function(done) {
      this.slow(1600);
      const nh = rosnodejs.nh;
      const sub = nh.subscribe(topic, msgType, () => {
        throwNext('Subscriber should never have gotten messages!');
      });
      let pub = nh.advertise(topic, msgType);

      // when publisher emits 'connection', it has validated
      // the subscriber's connection header and sent a response back
      // the subscriber will not have validated the publisher's connection
      // header though so it should have a pending publisher entry.
      pub.on('connection', () => {
        const impl = sub._impl;

        expect(Object.keys(impl._pendingPubClients)).to.have.lengthOf(1);
        expect(impl._pubClients).to.be.empty;

        sub.shutdown();

        expect(impl._pendingPubClients).to.be.empty;
        expect(impl._pubClients).to.be.empty;

        setTimeout(() => {
          expect(impl._pendingPubClients).to.be.empty;
          expect(impl._pubClients).to.be.empty;
          done();
        }, 500);
      });
    });

    it('2 Publishers on Same Topic', function(done) {
      this.slow(2000);
      const nh = rosnodejs.nh;

      let msg1;
      const sub = nh.subscribe(topic, msgType, (msg) => {
          msg1 = msg.data;
      });

      const pub1 = nh.advertise(topic, msgType, {latching: true});
      const pub2 = nh.advertise(topic, msgType, {latching: true});

      expect(pub1).to.not.equal(pub2);
      expect(pub1._impl.listenerCount('connection')).to.equal(2);
      expect(pub2._impl.listenerCount('connection')).to.equal(2);

      pub1.publish({data: 1});

      sub.once('message', ({data}) => {
        expect(sub.getNumPublishers()).to.equal(1);
        expect(data).to.equal(1);

        pub2.publish({data: 2});
        sub.once('message', ({data}) => {
          expect(data).to.equal(2);

          pub1.shutdown()
          .then(() => {
            expect(pub1._impl).to.equal(null);
            expect(pub2._impl.listenerCount('connection')).to.equal(1);

            expect(sub.getNumPublishers()).to.equal(1);

            pub2.publish({data: 3});

            sub.once('message', ({data}) => {
              expect(data).to.equal(3);

              pub2.shutdown()
              .then(() =>  {
                expect(sub.getNumPublishers()).to.equal(0);
                expect(pub2._impl).to.equal(null);
                done();
              });
            });
          })
        });
      })
    });

    it('2 Subscribers on Same Topic', function(done) {
      this.slow(2000);
      const nh = rosnodejs.nh;

      let msg1;
      const sub1 = nh.subscribe(topic, msgType, (msg) => {
          msg1 = msg.data;
      });

      let msg2;
      const sub2 = nh.subscribe(topic, msgType, (msg) => {
        msg2 = msg.data;
      });

      expect(sub1).to.not.equal(sub2);
      expect(sub1._impl.listenerCount('connection')).to.equal(2);
      expect(sub2._impl.listenerCount('connection')).to.equal(2);

      const pub = nh.advertise(topic, msgType, {latching: true});

      pub.publish({data: 1});

      sub2.once('message', () => {
        expect(pub.getNumSubscribers()).to.equal(1);

        expect(msg1).to.equal(msg2);
        pub.publish({data: 25});

        sub2.once('message', () => {
          expect(msg1).to.equal(msg2);
          msg1 = null;
          msg2 = null;

          sub1.shutdown()
          .then(() => {
            expect(sub1._impl).to.equal(null);
            expect(sub2._impl.listenerCount('connection')).to.equal(1);
            pub.publish({data: 30});

            sub2.once('message', () => {
              expect(msg1).to.equal(null);
              expect(msg2).to.equal(30);
              expect(pub.getNumSubscribers()).to.equal(1);

              sub2.shutdown()
              .then(() => {
                expect(sub2._impl).to.equal(null);
                expect(pub.getNumSubscribers()).to.equal(0);
                done();
              });
            });
          });
        });
      });
    });
  });

  describe('Service', () => {
    const service = '/test_service';
    const srvType = 'std_srvs/Empty';

    beforeEach(() => {
      let serviceInfo = null;

      masterStub.on('getUri', (err, params, callback) => {
        const resp = [1, '', 'localhost:11311/'];
        callback(null, resp);
      });

      masterStub.on('registerService', (err, params, callback) => {
        serviceInfo = params[2];

        const resp = [1, 'You did it!', []];
        callback(null, resp);
      });

      masterStub.on('unregisterService', (err, params, callback) => {
        const resp = [1, 'You did it!', serviceInfo ? 1 : 0];
        callback(null, resp);
        serviceInfo = null;
      });

      masterStub.on('lookupService', (err, params, callback) => {
        if (serviceInfo) {
          const resp = [1, "you did it", serviceInfo];
          callback(null, resp);
        }
        else {
          const resp = [-1, "no provider", ""];
          callback(null, resp);
        }
      });

      return rosnodejs.initNode(nodeName, initArgs);
    });

    afterEach(() => {
      const nh = rosnodejs.nh;

      // clear out any service, subs, pubs
      clearOutClients(nh._node);

      nh._node._spinner.clear();

      // remove any master api handlers we set up
      masterStub.removeAllListeners();
    });

    it('Call and Response', (done) => {
      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        return true;
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        return client.call({});
      })
      .then(() => {
        done();
      })
      .catch((err) => {
        throwNext(err);
      })
    });

    it('Asynchronous Call and Response', (done) => {
      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        return Promise.resolve(true);
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        return client.call({});
      })
      .then(() => {
        done();
      })
      .catch((err) => {
        throwNext(err);
      })
    });

    it('Service Failure', (done) => {
      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        return false;
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        return client.call({});
      })
      .then(() => {
        throwNext('Service call succeeded when it shouldn\'t have');
      })
      .catch((err) => {
        if (err.code === 'E_ROSSERVICEFAILED') {
          done();
        }
        else {
          console.error('Service call failed with unexpected error');
        }
      });
    });

    it('Service Shutdown While Registering', function (done) {
      this.slow(1600);

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        return true;
      });

      // hook into registered event - this should not fire
      serv.on('registered', () => {
        throw new Error('Service should never have registered!');
      });

      // kill the service while the asynchronous registration is happening
      serv.shutdown();

      // if we haven't seen the 'registered' event by now we should be good
      setTimeout(done, 500);
    });

    it('Service Shutdown During Call', function(done) {
      this.slow(1600);

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        throw new Error('Service callback should never have been called!');
      });

      let connected = false;
      serv.on('connection', () => {
        // we've received the client header but not the request - SHUT IT DOWN
        connected = true;
        serv.shutdown();
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        client.call({});
      });

      // if the service callback hasn't been called by now we should be good
      setTimeout(done, 500);
    });

    it('Service Shutdown Handling Call', function(done) {
      this.slow(1600);

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        serv.shutdown();

        return true;
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        return client.call({});
      })
      .catch((err) => {
        expect(err.message).to.equal('Connection was closed');
        done();
      });
    });

    it('Service Shutdown Handling Asynchronous Call', function(done) {
      this.slow(1600);

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {

        return new Promise((resolve) => {
          setTimeout(() => {
            serv.shutdown();
            resolve(true);
          }, 0);
        });
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {
        return client.call({});
      })
      .catch((err) => {
        expect(err.message).to.equal('Connection was closed');
        done();
      });
    });

    it('Service Unregistered During Call', (done) => {
      // simulate a service disconnecting between the lookupService call to ROS Master
      // and the connection to the service node's TCPROS server

      // cache a reference to net.connect - we'll replace it
      const NET_CONNECT_FUNC = net.connect;

      const nh = rosnodejs.nh;
      const serv = nh.advertiseService(service, srvType, (req, resp) => {
        return true;
      });

      const client = nh.serviceClient(service, srvType);
      nh.waitForService(service)
      .then(() => {

        // we've verified that the service exists - replace the net.connect call (used to initiate the TCPROS
        // connection) with a bogus one that throws an error
        net.connect = (info) => {
          const sock = new net.Socket();
          process.nextTick(() => {
            const error = new Error(`connect ECONNREFUSED ${info.host}:${info.port}`);
            error.code = 'ECONNREFUSED';
            error.errno = 'ECONNREFUSED';
            error.address = info.host;
            error.port = info.port;

            // just to make sure there isn't some other error that comes through - should be unnecessary
            error.rosnodejstesting = true;
            sock.emit('error', error);
          });
          return sock;
        };

        return client.call({});
      })
      .catch((err) => {
        if (err.code === 'ECONNREFUSED' && err.rosnodejstesting) {
          // nice! restore net.connect and close up shop
          net.connect = NET_CONNECT_FUNC;
          done();
        }
      })
    });
  });

  describe('Shutdown', () => {
    let masterStub;
    before(() => {
      stopMasterStub(() => {
        masterStub = new MasterStub('localhost', MASTER_PORT);
        masterStub.provideAll();
      });

      return rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
      });
    });

    after((done) => {
      masterStub.shutdown()
      .then(() => {
        done();
      });
    })

    it('Shutdown after successful start with master running', function(done) {
      rosnodejs.initNode(nodeName, initArgs)
      .then(() => {
        return rosnodejs.shutdown();
      })
      .then(() => {
        rosnodejs.reset();
        expect(gotEvent).to.be.true;
        done();
      });

      let gotEvent = false;
      rosnodejs.on('shutdown', () => {
        gotEvent = true;
      });
    });

    it('Shutdown after successful start with master down', function(done) {
      rosnodejs.initNode(nodeName, initArgs)
      .then(() => {
        return masterStub.shutdown();
      })
      .then(() => {
        return rosnodejs.shutdown();
      })
      .then(() => {
        rosnodejs.reset();
        expect(gotEvent).to.be.true;
        done();
      });

      let gotEvent = false;
      rosnodejs.on('shutdown', () => {
        gotEvent = true;
      });
    });

    it('Shutdown when unable to connect to master', function(done) {
      rosnodejs.initNode(nodeName, initArgs)
      .then(() => {
        throwNext('Node should not have initialized!');
      });

      let gotEvent = false;
      rosnodejs.on('shutdown', () => {
        gotEvent = true;
      });

      setTimeout(() => {
        rosnodejs.shutdown()
        .then(() => {
          rosnodejs.reset();
          expect(gotEvent).to.be.true;
          done();
        });
      }, 500);
    });

    it('Spinner is cleared out when shutdown', function(done) {
      masterStub.listen();
      masterStub.provideAll();

      let gotEvent = false;

      rosnodejs.initNode(nodeName, initArgs)
      .then((nh) => {
        
        const pub = nh.advertise('/chatter', 'std_msgs/String');
        const sub = nh.subscribe('/chatter', 'std_msgs/String');
        pub.publish({data: 'hi'});
      

        rosnodejs.shutdown()
        .then(() => {
          rosnodejs.reset();
          expect(gotEvent).to.be.true;
          expect(nh._node._spinner._spinTimer.clientCallQueue).to.be.undefined;
          done();
        });
      });

      rosnodejs.on('shutdown', () => {
        gotEvent = true;
      });
    });
  });

  describe('Parameters', function() {
    let masterStub;

    before(() => {
      masterStub = new MasterStub('localhost', MASTER_PORT);
      masterStub.provideAll();

      return rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
        return rosnodejs.initNode(nodeName, initArgs);
      })
    });

    after(() => {
      masterStub.shutdown();
      return rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
      });
    });

    it('Set', function(done) {
      const nh = rosnodejs.nh;

      nh.setParam('/key', 2)
      .then(() => {
        expect(masterStub._params['/key']).to.equal(2);
        done();
      });
    });

    it('Get', function(done) {
      const nh = rosnodejs.nh;

      nh.getParam('/key')
      .then((result) => {
        expect(result).to.equal(2);
        expect(masterStub._params['/key']).to.equal(2);
        done();
      });
    });

    it('Has', function(done) {
      const nh = rosnodejs.nh;

      nh.hasParam('/key')
      .then((result) => {
        expect(result).to.be.true;
        expect(masterStub._params['/key']).to.equal(2);
        done();
      });
    });

    it('Delete', function(done) {
      const nh = rosnodejs.nh;

      nh.deleteParam('/key')
      .then(() => {
        expect(masterStub._params['/key']).to.be.undefined;
        done();
      });
    });

    it('Full', function(done) {
      const nh = rosnodejs.nh;

      nh.getParam('/missing')
      .then(() => throwNext('Get should reject'))
      .catch(() => {
        return nh.hasParam('/missing')
      })
      .catch(() => throwNext('Has should resolve'))
      .then((result) => {
        expect(result).to.be.false;
        return nh.setParam('/exists', 1);
      })
      .catch(() => throwNext('Set should resolve'))
      .then(() => {
        expect(masterStub._params['/exists']).to.equal(1);
        return nh.hasParam('/exists');
      })
      .catch(() => throwNext('Has should resolve'))
      .then((result) => {
        expect(result).to.be.true;
        return nh.getParam('/exists');
      })
      .catch(() => throwNext('Get should resolve'))
      .then((result) => {
        expect(result).to.equal(1);
        return nh.deleteParam('/missing');
      })
      .then(() => throwNext('Delete should reject'))
      .catch((err) => {
        return nh.deleteParam('/exists');
      })
      .catch(() => throwNext('Delete should resolve'))
      .then(() => {
        return nh.hasParam('/exists');
      })
      .catch(() => throwNext('Has should resolve'))
      .then((result) => {
        expect(result).to.be.false;
        done();
      });
    });
  });

  describe('initialization', () => {
    const MASTER_PORT = 55599;
    const rosMasterUri = `http://localhost:${MASTER_PORT}`;
    // don't provide any of the expected APIs so the node won't initialize
    const masterStub = new MasterStub('localhost', MASTER_PORT);
    // turn off warnings about methods not existing since we're purposefully
    // not providing any methods
    masterStub.verbose = false;

    afterEach((done) => {
      rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
        done();
      });
    })

    after((done) => {
      masterStub.shutdown()
      .then(() => { done() });
    })

    it('wait forever', function(done) {
      this.slow(3000);
      this.timeout(5000);

      rosnodejs.initNode('test_node', { rosMasterUri })
      .then(() => {
        process.nextTick(() => {
          throw new Error('Node shouldnt initialize');
        });
      });

      setTimeout(done, 2000);
    });

    it('wait a little', function(done) {
      this.slow(3000);
      this.timeout(5000);
      let timeout;
      rosnodejs.initNode('test_node', { rosMasterUri, timeout: 1000 })
      .then(() => {
        process.nextTick(() => {
          throw new Error('Node shouldnt initialize');
        });
      })
      .catch((err) => {
        clearTimeout(timeout);
        done();
      })

      timeout = setTimeout(() => {
        throw new Error('Should have timed out!');
      }, 2000);
    });

    it('wait once', function(done) {
      this.slow(500);

      let timeout;
      rosnodejs.initNode('test_node', { rosMasterUri, timeout: 0 })
      .then(() => {
        process.nextTick(() => {
          throw new Error('Node shouldnt initialize');
        });
      })
      .catch((err) => {
        clearTimeout(timeout);
        done();
      })

      timeout = setTimeout(() => {
        throw new Error('Should have timed out!');
      }, 500);
    });
  });
});
