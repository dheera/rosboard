'use strict';

const chai = require('chai');
const expect = chai.expect;
const bunyan = require('bunyan');
const xmlrpc = require('xmlrpc');
const rosnodejs = require('../src/index.js');

const MASTER_PORT = 11234;

/** setup pipe to stdout **/
class OutputCapture {
  constructor() {
    this.flush();
  }

  write(data) {
    this.lastMsg = data;
  }

  flush() {
    this.lastMsg = null;
  }

  get() {
    return this.lastMsg;
  }
}

describe('Logging', () => {
  const outputCapture = new OutputCapture();
  let masterStub;

  const reset = function() {
    outputCapture.flush();
    expect(outputCapture.get()).to.equal(null);

    rosnodejs.log.rootLogger._throttledLogs = new Map();
    rosnodejs.log.rootLogger._onceLogs = new Set();
  };

  before((done) => {
    masterStub = xmlrpc.createServer({host: 'localhost', port: MASTER_PORT}, () => {
      rosnodejs.initNode('/testNode', {
        rosMasterUri: `http://localhost:${MASTER_PORT}`,
        logging: {skipRosLogging: true},
        notime: true
      })
      .then(() => {
        rosnodejs.log.addStream({
          type: 'raw',
          level: 'info',
          stream: outputCapture
        });

        rosnodejs.log.setLevel('trace');
        done();
      });
    });

    masterStub.on('getUri', (err, params, callback) => {
        const resp = [ 1, '', `localhost:${MASTER_PORT}/` ];
        callback(null, resp);
      });
  });

  after((done)=> {
    rosnodejs.shutdown()
    .then(() => {
      rosnodejs.reset();
      masterStub.close(() => { done(); });
    });
  });

  it('Levels', () => {
    const message = 'This is my message';
    reset();

    rosnodejs.log.setLevel('fatal');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get()).to.equal(null);
    reset();

    rosnodejs.log.setLevel('error');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get()).to.equal(null);
    reset();

    rosnodejs.log.setLevel('warn');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get()).to.equal(null);
    reset();

    rosnodejs.log.setLevel('info');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get()).to.equal(null);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get()).to.equal(null);
    reset();

    rosnodejs.log.setLevel('debug');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get()).to.equal(null);
    reset();

    rosnodejs.log.setLevel('trace');
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get().msg).to.have.string(message);
    reset();
  });

  it('Throttling', () => {
    const message = 'This is my message';
    reset();
    rosnodejs.log.infoThrottle(1000, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.INFO);

    outputCapture.flush();
    rosnodejs.log.infoThrottle(1000, message);
    expect(outputCapture.get()).to.be.null;
  });

  it('Bound Log Methods', () => {
    const message = 'This is my message';

    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.TRACE);

    reset();
    rosnodejs.log.debug(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.DEBUG);

    reset();
    rosnodejs.log.info(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.INFO);

    reset();
    rosnodejs.log.warn(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.WARN);

    reset();
    rosnodejs.log.error(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.ERROR);

    reset();
    rosnodejs.log.fatal(message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.FATAL);

    reset();
    rosnodejs.log.traceThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.TRACE);

    reset();
    rosnodejs.log.debugThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.DEBUG);

    reset();
    rosnodejs.log.infoThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.INFO);

    reset();
    rosnodejs.log.warnThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.WARN);

    reset();
    rosnodejs.log.errorThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.ERROR);

    reset();
    rosnodejs.log.fatalThrottle(1, message);
    expect(outputCapture.get().msg).to.have.string(message);
    expect(outputCapture.get().level).to.equal(bunyan.FATAL);
  });

  it('Child Loggers', () => {
    const message = 'This is my message';

    let testLogger = rosnodejs.log.getLogger('testLogger');

    // individually set log level
    reset();
    testLogger.setLevel('info');
    testLogger.info(message);
    expect(outputCapture.get().msg).to.have.string(message);

    reset();
    testLogger.trace(message);
    expect(outputCapture.get()).to.equal(null);

    // root log should still be at trace
    reset();
    rosnodejs.log.trace(message);
    expect(outputCapture.get().msg).to.have.string(message);

    // setting through rosnodejs should set all loggers
    rosnodejs.log.setLevel('trace');
    reset();
    testLogger.trace(message);
    expect(outputCapture.get().msg).to.have.string(message);
  });

  describe('Rosout', () => {
    let pubInfo = null;
    let subInfo = null;

    before((done) => {
      rosnodejs.shutdown()
      .then(() => {
        rosnodejs.reset();
        return rosnodejs.initNode('/testNode', {logging: {waitOnRosOut: false, level: 'info'},
                                  rosMasterUri: `http://localhost:${MASTER_PORT}`, notime: true});
      })
      .then(() => {
        done();
      });

      masterStub.on('getUri', (err, params, callback) => {
        const resp = [ 1, '', `localhost:${MASTER_PORT}/` ];
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

      masterStub.on('registerService', (err, params, callback) => {
        const resp =  [1, 'You did it!', 1];
        callback(null, resp);
      });

      masterStub.on('unregisterService', (err, params, callback) => {
        const resp =  [1, 'You did it!', 1];
        callback(null, resp);
      });
    });

    after(() => {
      // remove any master api handlers we set up
      masterStub.removeAllListeners();
    });

    it('Check Publishing', (done) => {
      rosnodejs.log.setLevel('fatal');
      const testLogger = rosnodejs.log.getLogger('testLogger');
      testLogger.setLevel('info');
      const nh = rosnodejs.nh;
      const message = 'This is my message';
      let intervalId = null;

      let timeout = setTimeout(() => {
        throw new Error('Didn\'t receive log message within 500ms...');
      }, 500);

      const rosoutCallback = (msg) => {
        if (msg.msg.indexOf(message) > -1) {
          nh.unsubscribe('/rosout');
          clearInterval(intervalId);
          clearTimeout(timeout);
          intervalId = null;
          done();
        }
      };

      const sub = nh.subscribe('/rosout', 'rosgraph_msgs/Log', rosoutCallback);

      intervalId = setInterval(() => {
        testLogger.info(message);
      }, 50);
    });
  });

});
