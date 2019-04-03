
const chai = require('chai');
const xmlrpc = require('xmlrpc');
const rosnodejs = require('rosnodejs');

const TOPIC = '/topic';
const TYPE = 'std_msgs/String';
const SERVICE = '/service';
const SRV = 'std_srvs/Empty';

const MASTER_PORT = 11234;

// Each Test in this suite simulates rapid fire connection/disconnection
// of TCPROS clients
describe('ClientShutdown', function() {

  this.timeout(10000);
  this.slow(10000);

  let sub = null;
  let pub = null;
  let service = null;
  let client = null;

  let interval1;
  let interval2;
  let interval3;

  let masterStub;

  function startSub(nh) {
    sub = nh.subscribe(TOPIC, TYPE, (msg) => {
      console.log('%j', msg);
    });

    return sub;
  }

  function stopSub() {
    if (sub) {
      sub.shutdown();
      sub = null;
    }
  }

  function startPub(nh) {
    pub = nh.advertise(TOPIC, TYPE);
    return pub;
  }

  function stopPub() {
    if (pub) {
      pub.shutdown();
      pub = null;
    }
  }

  function startService(nh) {
    service = nh.advertiseService(SERVICE, SRV, () => {
      console.log('handling service call');
      return true;
    });
    return service;
  }

  function stopService() {
    if (service) {
      service.shutdown();
      service = null;
    }
  }

  function startClient(nh) {
    client = nh.serviceClient(SERVICE, SRV);
    return client;
  }

  function stopClient() {
    if (client) {
      client.shutdown();
      client = null;
    }
  }

  before((done) => {
    masterStub = xmlrpc.createServer({host: 'localhost', port: MASTER_PORT}, () => { done(); });
  });

  after((done) => {
    masterStub.close(() => { done(); });
  });

  beforeEach(() => {
    let pubInfo = null;
    let subInfo = null;
    let serviceInfo = null;

    masterStub.on('getUri', (err, params, callback) => {
      const resp = [ 1, '', 'localhost:11311/' ];
      callback(null, resp);
    });

    masterStub.on('registerSubscriber', (err, params, callback) => {
      subInfo = params[3];
      // console.log('sub reg ' + params);
      //console.log(pubInfo);

      const resp =  [1, 'You did it!', []];
      if (pubInfo) {
        resp[2].push(pubInfo);
      }
      callback(null, resp);
    });

    masterStub.on('unregisterSubscriber', (err, params, callback) => {
      // console.log('unregister subscriber!');
      const resp =  [1, 'You did it!', subInfo ? 1 : 0];
      callback(null, resp);
      subInfo = null;
    });

    masterStub.on('registerPublisher', (err, params, callback) => {
      // console.log('pub reg ' + Date.now());
      pubInfo = params[3];
      const resp =  [1, 'You did it!', []];
      if (subInfo) {
        resp[2].push(pubInfo);
        let subAddrParts = subInfo.replace('http://', '').split(':');
        let client = xmlrpc.createClient({host: subAddrParts[0], port: subAddrParts[1]});
        let data = [1, TOPIC, [pubInfo]];
        client.methodCall('publisherUpdate', data, (err, response) => { });
      }
      callback(null, resp);
    });

    masterStub.on('unregisterPublisher', (err, params, callback) => {
      // console.log('pub unreg ' + Date.now());
      const resp =  [1, 'You did it!', pubInfo ? 1 : 0];
      callback(null, resp);
      if (subInfo) {
        let subAddrParts = subInfo.replace('http://', '').split(':');
        let client = xmlrpc.createClient({host: subAddrParts[0], port: subAddrParts[1]});
        let data = [1, TOPIC, []];
        client.methodCall('publisherUpdate', data, (err, response) => { });
      }
      pubInfo = null;
    });

    masterStub.on('registerService', (err, params, callback) => {
      serviceInfo = params[2];

      const resp = [1, 'You did it!', []];
      callback(null, resp);
    });

    masterStub.on('unregisterService', (err, params, callback) => {
      const resp = [1, 'You did it!', subInfo ? 1 : 0];
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

    masterStub.on('NotFound', (method, params) => {
      console.error('Got unknown method call %s: %j', method, params);
    });

    return rosnodejs.initNode('/my_node', {rosMasterUri: `http://localhost:${MASTER_PORT}`, logging: {testing: true}, notime:true});
  });

  afterEach(() => {
    sub = null;
    pub = null;
    service = null;
    client = null;

    clearInterval(interval1);
    clearInterval(interval2);
    clearInterval(interval3);

    const nh = rosnodejs.nh;

    // clear out any service, subs, pubs
    nh._node._services = {};
    nh._node._subscribers = {};
    nh._node._publishers = {};

    // remove any master api handlers we set up
    masterStub.removeAllListeners();
  });

  it('Subscriber Shutdown', (done) => {
    const nh = rosnodejs.nh;
    const pub = startPub(nh);

    const msg = {data: 'This shouldn\'t crash'};

    interval1 = setInterval(() => {
      pub.publish(msg);
    }, 3);

    interval2 = setInterval(() => {
      if (sub === null) {
        startSub(nh);
      }
      else {
        stopSub();
      }
    }, 10);

    setTimeout(done, 8000);
  });

  it('Publisher Shutdown', (done) => {
    const nh = rosnodejs.nh;
    startSub(nh);

    const msg = {data: 'This shouldn\'t crash'};

    interval1 = setInterval(() => {
      if (pub) {
        pub.publish(msg, -1);
      }
    }, 3);

    interval2 = setInterval(() => {
      if (pub === null) {
        startPub(nh);
      }
      else {
        stopPub();
      }
    }, 10);

    setTimeout(done, 8000);
  });

  it('Pub Sub Shutdown', (done) => {
    const nh = rosnodejs.nh;

    const msg = {data: 'This shouldn\'t crash'};

    interval1 = setInterval(() => {
      if (pub) {
        pub.publish(msg);
      }
    }, 3);

    interval2 = setInterval(() => {
      if (pub === null) {
        startPub(nh);
      }
      else {
        stopPub();
      }
    }, 10);

    interval3 = setInterval(() => {
      if (sub === null) {
        startSub(nh);
      }
      else {
        stopSub();
      }
    }, 7);

    setTimeout(done, 8000);
  });

  it('Service Shutdown', (done) => {
    const nh = rosnodejs.nh;
    const client = startClient(nh);

    const req = {};

    interval1 = setInterval(() => {
      client.call(req);
    }, 3);

    interval2 = setInterval(() => {
      if (service === null) {
        startService(nh);
      }
      else {
        stopService();
      }
    }, 10);

    setTimeout(done, 8000);
  });

  it('Client Shutdown', (done) => {
    const nh = rosnodejs.nh;
    startService(nh);

    const req = {};

    interval1 = setInterval(() => {
      if (client) {
        client.call(req);
      }
    }, 1);

    interval2 = setInterval(() => {
      if (client === null) {
        startClient(nh);
      }
      else {
        stopClient();
      }
    }, 10);

    setTimeout(done, 8000);
  });

  it('Client Service Shutdown', (done) => {
    const nh = rosnodejs.nh;

    const req = {};

    interval1 = setInterval(() => {
      if (client) {
        client.call(req);
      }
    }, 1);

    interval2 = setInterval(() => {
      if (client === null) {
        startClient(nh);
      }
      else {
        stopClient();
      }
    }, 10);

    interval3 = setInterval(() => {
      if (service === null) {
        startService(nh);
      }
      else {
        stopService();
      }
    }, 7);

    setTimeout(done, 8000);
  });

});
