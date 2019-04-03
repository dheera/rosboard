'use strict'

const chai = require('chai');
const expect = chai.expect;
const serUtils = require('../src/utils/serialization_utils.js');
const DeserializeStream = serUtils.DeserializeStream;

describe('DeserializeStream', () => {
  const deserializeStream = new DeserializeStream();

  it('basic', (done) => {
    const len = 20;
    deserializeStream.on('message', (message) => {
      expect(message.length).to.equal(len);

      done();
    });

    const buf = new Buffer(len).fill(0);
    const bufWithLen = serUtils.Serialize(buf);

    deserializeStream.write(bufWithLen);
  });

  it('2 chunks', (done) => {
    const len = 20;
    let iter = 0;
    deserializeStream.on('message', (message) => {
      ++iter;
      expect(message.length).to.equal(len);

      if (iter === 2) {
        done();
      }
    });

    const buf = new Buffer(len).fill(0);
    const bufWithLen = serUtils.Serialize(buf);

    deserializeStream.write(bufWithLen);
    deserializeStream.write(bufWithLen);
  });

  it('split length', (done) => {
    const len = 20;
    let iter = 0;
    deserializeStream.on('message', (message) => {
      ++iter;
      expect(message.length).to.equal(len);

      if (iter === 2) {
        done();
      }
    });

    const buf = new Buffer(len).fill(0);
    const bufWithLen = serUtils.Serialize(buf);

    const doubleBuf = Buffer.concat([bufWithLen, bufWithLen]);
    const bufA = doubleBuf.slice(0, len+5);
    const bufB = doubleBuf.slice(len+5);

    deserializeStream.write(bufA);
    deserializeStream.write(bufB);
  });

  describe('service response flag', () => {

    beforeEach(() => {
      deserializeStream.setServiceRespDeserialize();
    });

    it('normal', (done) => {
      const len = 20;
      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === 2) {
          done();
        }
      });

      const buf = new Buffer(len).fill(0);
      const bufWithLen = serUtils.Serialize(buf);
      const okBuf = new Buffer(1).fill(1);

      const doubleBuf = Buffer.concat([okBuf, bufWithLen, okBuf, bufWithLen]);
      const bufA = doubleBuf.slice(0, len+5);
      const bufB = doubleBuf.slice(len+5);

      deserializeStream.write(bufA);
      deserializeStream.write(bufB);
    });

    it('improper', (done) => {
      const len = 19;
      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === 2) {
          done();
        }
      });

      const buf = new Buffer(len + 5).fill(0);
      buf[0] = 1;
      buf.writeUInt32LE(19, 1);

      const doubleBuf = Buffer.concat([buf, buf]);
      const bufA = doubleBuf.slice(0, len+6);
      const bufB = doubleBuf.slice(len+6);

      deserializeStream.write(bufA);
      deserializeStream.write(bufB);
    });

    afterEach(() => {
      deserializeStream._deserializeServiceResp = false;
    });
  });

  describe('various splits', () => {
    const len = 20;
    const bufData = new Buffer(len).fill(0);
    const serBufData = serUtils.Serialize(bufData);

    const peelEmOff = (buff, sliceLens) => {
      let i = 0;
      while (buff.length > 0) {
        const sliceLen = sliceLens[i++];
        const slice = buff.slice(0, sliceLen);
        deserializeStream.write(slice);
        buff = buff.slice(sliceLen);
      }
    };

    it('1', (done) => {
      const sliceLens = [24, 23, 25];
      const numBufs = sliceLens.length;
      const bufArr = new Array(numBufs).fill(serBufData);
      let bigBuff = Buffer.concat(bufArr);

      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === numBufs) {
          done();
        }
      });

      peelEmOff(bigBuff, sliceLens);
    });

    it('2', (done) => {
      const sliceLens = [18, 26, 28];
      const numBufs = sliceLens.length;
      const bufArr = new Array(numBufs).fill(serBufData);
      let bigBuff = Buffer.concat(bufArr);

      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === numBufs) {
          done();
        }
      });

      peelEmOff(bigBuff, sliceLens);
    });

    it('3', (done) => {
      const sliceLens = [18, 28, 26, 25, 26, 24, 21, 23, 25];
      const numBufs = sliceLens.length;
      const bufArr = new Array(numBufs).fill(serBufData);
      let bigBuff = Buffer.concat(bufArr);

      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === numBufs) {
          done();
        }
      });

      peelEmOff(bigBuff, sliceLens);
    });

    it('4', (done) => {
      const sliceLens = [28, 26, 25, 26, 24, 21, 23, 26, 17];
      const numBufs = sliceLens.length;
      const bufArr = new Array(numBufs).fill(serBufData);
      let bigBuff = Buffer.concat(bufArr);

      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === numBufs) {
          done();
        }
      });

      peelEmOff(bigBuff, sliceLens);
    });

    it('services', (done) => {
      const sliceLens = [28, 26, 25, 26, 24, 21, 23, 26, 17];
      const numBufs = sliceLens.length;
      const len = 19;
      let iter = 0;
      deserializeStream.on('message', (message) => {
        ++iter;
        expect(message.length).to.equal(len);

        if (iter === numBufs) {
          deserializeStream._deserializeServiceResp = false;
          done();
        }
      });

      const buf = new Buffer(len + 5).fill(0);
      buf[0] = 1;
      buf.writeUInt32LE(19, 1);

      const bufArr = new Array(numBufs).fill(buf);
      let bigBuff = Buffer.concat(bufArr);

      deserializeStream.setServiceRespDeserialize();
      peelEmOff(bigBuff, sliceLens);
    });
  });

  // TODO: spend more time writing unit tests and get rid of randomness

  it('random split stream', (done) => {
    const len = 20;
    const bufData = new Buffer(len).fill(0);
    const serBufData = serUtils.Serialize(bufData);

    const numBufs = 10000;
    const bufArr = new Array(numBufs).fill(serBufData);
    let bigBuff = Buffer.concat(bufArr);

    let iter = 0;
    deserializeStream.on('message', (message) => {
      ++iter;
      expect(message.length).to.equal(len);

      if (iter === numBufs) {
        done();
      }
    });

    const rand = () => { return Math.round(Math.random() * 2 * (len + 4)); }

    while (bigBuff.length > 0) {
      const pos = Math.min(rand(), bigBuff.length);
      const slice = bigBuff.slice(0, pos);
      deserializeStream.write(slice);
      bigBuff = bigBuff.slice(pos);
    }
  });

  it('random split stream 2', (done) => {
    const len = 20;
    const bufData = new Buffer(len).fill(0);
    const serBufData = serUtils.Serialize(bufData);

    const numBufs = 10000;
    const bufArr = new Array(numBufs).fill(serBufData);
    let bigBuff = Buffer.concat(bufArr);

    let iter = 0;
    deserializeStream.on('message', (message) => {
      ++iter;
      expect(message.length).to.equal(len);

      if (iter === numBufs) {
        done();
      }
    });

    // return a random integer between 20 and 28
    const rand = () => { return Math.round(Math.random() * 8) + len; }

    while (bigBuff.length > 0) {
      const pos = Math.min(rand(), bigBuff.length);
      const slice = bigBuff.slice(0, pos);
      deserializeStream.write(slice);
      bigBuff = bigBuff.slice(pos);
    }
  });

  it('random split services', (done) => {
    const sliceLens = [28, 26, 25, 26, 24, 21, 23, 26, 17];
    const numBufs = 10000;

    const len = 19;
    let iter = 0;
    deserializeStream.on('message', (message) => {
      ++iter;
      expect(message.length).to.equal(len);

      if (iter === numBufs) {
        deserializeStream._deserializeServiceResp = false;
        done();
      }
    });

    const buf = new Buffer(len + 5).fill(0);
    buf[0] = 1;
    buf.writeUInt32LE(19, 1);

    const bufArr = new Array(numBufs).fill(buf);
    let bigBuff = Buffer.concat(bufArr);

    deserializeStream.setServiceRespDeserialize();

    const rand = () => { return Math.round(Math.random() * 8) + len; }

    while (bigBuff.length > 0) {
      const pos = Math.min(rand(), bigBuff.length);
      const slice = bigBuff.slice(0, pos);
      deserializeStream.write(slice);
      bigBuff = bigBuff.slice(pos);
    }
  });

  afterEach(() => {
    deserializeStream.removeAllListeners('message');
  });
});
