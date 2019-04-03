'use strict';

const chai = require('chai');
const expect = chai.expect;
const GlobalSpinner = require('../src/utils/spinners/GlobalSpinner.js');

let handleList = [];
let spinner;

class DummyClient {
  constructor() {
    this.id = 'xxxxxxxx'.replace(/[x]/g, function(c) {
      return (Math.random()*8).toString(16);
    });
  }

  _handleMsgQueue(queue) {
    handleList.push({id: this.id, queue});
  }
}

describe('GlobalSpinner', () => {

  beforeEach(() => {
    spinner = new GlobalSpinner({emit: true});
    handleList = [];
  });

  it('Ping',  (done) => {
    const clients = [new DummyClient(), new DummyClient(), new DummyClient(), new DummyClient()];

    clients.forEach((client) => {
      spinner.addClient(client, client.id, 3, 0);
    });

    spinner.ping(clients[0].id, "junk");
    expect(spinner._spinTimer).to.not.be.null;

    spinner.ping(clients[3].id, "junk");
    spinner.ping(clients[2].id, "junk");
    spinner.ping(clients[1].id, "junk");

    expect(spinner._clientCallQueue.length).to.equal(4);

    spinner.disconnect(clients[3].id);

    expect(spinner._clientCallQueue.length).to.equal(3);

    spinner.once('tick', () => {
      expect(handleList.length).to.equal(3);
      expect(handleList[0].id).to.equal(clients[0].id);
      expect(handleList[1].id).to.equal(clients[2].id);
      expect(handleList[2].id).to.equal(clients[1].id);

      done();
    });
  });

  it('QueueSize', (done) => {
    const client = new DummyClient();

    const messages = ["a", "b", "c", "d", "e"];

    spinner.addClient(client, client.id, 3, 0);

    messages.forEach((message) => {
      spinner.ping(client.id, message);
    });

    spinner.once('tick', () => {
      expect(handleList.length).to.equal(1);
      const handledQueue = handleList[0].queue;
      expect(handledQueue.length).to.equal(3);
      expect(handledQueue[0]).to.equal(messages[2]);
      expect(handledQueue[1]).to.equal(messages[3]);
      expect(handledQueue[2]).to.equal(messages[4]);

      done();
    })
  });

  it('Locking', (done) => {
    const client = new DummyClient();

    spinner.addClient(client, client.id, 3, 0);

    spinner.ping(client.id, 'junk');

    // lock the queue so that the next ping is cached
    spinner._queueLocked = true;

    spinner.ping(client.id, 'junk');

    expect(spinner._lockedOpCache.length).to.equal(1);

    spinner.once('tick', () => {
      expect(handleList.length).to.equal(1);
      expect(handleList[0].queue.length).to.equal(1);
      expect(spinner._lockedOpCache.length).to.equal(0);
      expect(spinner._clientCallQueue.length).to.equal(1);
      expect(spinner._spinTimer).to.not.be.null;
      handleList = [];

      spinner.once('tick', () => {
        expect(handleList.length).to.equal(1);
        expect(handleList[0].queue.length).to.equal(1);
        expect(spinner._lockedOpCache.length).to.equal(0);
        expect(spinner._clientCallQueue.length).to.equal(0);
        handleList = [];

        spinner.ping(client.id, 'junk');

        // lock the queue so the next disconnect is cached
        spinner._queueLocked = true;

        spinner.disconnect(client.id);
        expect(spinner._lockedOpCache.length).to.equal(1);

        spinner.once('tick', () => {
          expect(handleList.length).to.equal(1);
          expect(handleList[0].queue.length).to.equal(1);
          expect(spinner._lockedOpCache.length).to.equal(0);
          expect(spinner._clientCallQueue.length).to.equal(0);
          expect(spinner._clientQueueMap.has(client.id)).to.be.false;
          handleList = [];

          spinner.addClient(client, client.id, 3, 0);
          spinner.ping(client.id, 'junk');

          expect(spinner._lockedOpCache.length).to.equal(0);

          // lock the queue so the next disconnect is cached
          spinner._queueLocked = true;

          spinner.disconnect(client.id);
          spinner.addClient(client, client.id, 3, 0);
          spinner.ping(client.id, 'junk');
          spinner.ping(client.id, 'junk');

          expect(spinner._lockedOpCache.length).to.equal(4);

          spinner.once('tick', () => {
            // things that got in before we locked the spinner
            expect(handleList.length).to.equal(1);
            expect(handleList[0].queue.length).to.equal(1);
            expect(spinner._lockedOpCache.length).to.equal(0);
            expect(spinner._clientCallQueue.length).to.equal(1);
            expect(spinner._clientQueueMap.has(client.id)).to.be.true;
            handleList = [];

            // things that got in after we locked the spinner
            spinner.once('tick', () => {
              expect(handleList.length).to.equal(1);
              expect(handleList[0].queue.length).to.equal(2);
              expect(spinner._lockedOpCache.length).to.equal(0);
              expect(spinner._clientCallQueue.length).to.equal(0);
              expect(spinner._clientQueueMap.has(client.id)).to.be.true;

              spinner.disconnect(client.id);

              expect(spinner._clientQueueMap.has(client.id)).to.be.false;

              done();
            });
          });
        });
      });
    });
  });

  it('Throttling', (done) => {
    const client = new DummyClient();

    const throttleMs = 100;
    spinner.addClient(client, client.id, 1, throttleMs);

    spinner.ping(client.id, "junk");
    spinner.once('tick', () => {
      let firstTick = Date.now();
      spinner.ping(client.id, "junk");

      spinner.on('tick', () => {
        if (spinner._clientCallQueue.length === 0) {
          const lastTick = Date.now();
          const tDiff = lastTick - firstTick + 1;
          expect(tDiff).to.be.at.least(throttleMs);

          done();
        }
      });
    });
  });
});
