'use strict';

const chai = require('chai');
const expect = chai.expect;
const serialize = require('../lib/base_serialize.js');
const BN = require('bn.js');

function toBuf(val) {
  return Buffer.from(val, 'hex');
}

describe('Serialization Tests', () => {
  it('Uint8', () => {
    const buffer = Buffer.alloc(4);
    const compBuf = Buffer.alloc(4);
    expect(serialize.uint8(0, buffer, 0)).to.equal(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint8(255, buffer, 1)).to.equal(2);
    compBuf.writeUInt8(255, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint8(20, buffer, 2)).to.equal(3);
    compBuf.writeUInt8(20, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint8(86, buffer, 3)).to.equal(4);
    compBuf.writeUInt8(86, 3);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint8 Array', () => {
    const buffer = Buffer.alloc(14);
    const compBuf = Buffer.alloc(14);
    expect(serialize.Array.uint8([0], buffer, 0)).to.equal(5);
    compBuf.writeUInt32LE(1);
    compBuf.writeUInt8(0, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint8([255, 20], buffer, 5, 2)).to.equal(7);
    compBuf.writeUInt8(255, 5);
    compBuf.writeUInt8(20, 6);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint8([126, 32, 192], buffer, 7)).to.equal(14);
    compBuf.writeUInt32LE(3, 7);
    compBuf.writeUInt8(126, 11);
    compBuf.writeUInt8(32, 12);
    compBuf.writeUInt8(192, 13);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint16', () => {
    const buffer = Buffer.alloc(8);
    const compBuf = Buffer.alloc(8);
    expect(serialize.uint16(0, buffer, 0)).to.equal(2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint16(65535, buffer, 2)).to.equal(4);
    compBuf.writeUInt16LE(65535, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint16(32455, buffer, 4)).to.equal(6);
    compBuf.writeUInt16LE(32455, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint16(86, buffer, 6)).to.equal(8);
    compBuf.writeUInt16LE(86, 6);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint16 Array', () => {
    const buffer = Buffer.alloc(18);
    const compBuf = Buffer.alloc(18);
    expect(serialize.Array.uint16([0, 65534], buffer, 0, -1)).to.equal(8);
    compBuf.writeUInt32LE(2);
    compBuf.writeUInt16LE(0, 4);
    compBuf.writeUInt16LE(65534, 6);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint16([255], buffer, 8, -1)).to.equal(14);
    compBuf.writeUInt32LE(1, 8);
    compBuf.writeUInt16LE(255, 12);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint16([268, 32], buffer, 14, 2)).to.equal(18);
    compBuf.writeUInt16LE(268, 14);
    compBuf.writeUInt16LE(32, 16);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint32', () => {
    const buffer = Buffer.alloc(16);
    const compBuffer = Buffer.alloc(16);
    expect(serialize.uint32(0, buffer, 0)).to.equal(4);
    expect(buffer.equals(compBuffer)).to.be.true;
    expect(serialize.uint32(4294967295, buffer, 4)).to.equal(8);
    compBuffer.writeUInt32LE(4294967295, 4);
    expect(buffer.equals(compBuffer)).to.be.true;
    expect(serialize.uint32(1132256, buffer, 8)).to.equal(12);
    compBuffer.writeUInt32LE(1132256, 8);
    expect(buffer.equals(compBuffer)).to.be.true;
    expect(serialize.uint32(86, buffer, 12)).to.equal(16);
    compBuffer.writeUInt32LE(86, 12);
    expect(buffer.equals(compBuffer)).to.be.true;
  });

  it('Uint32 Array', () => {
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.Array.uint32([0, 65534], buffer, 0, -1)).to.equal(12);
    compBuf.writeUInt32LE(2);
    compBuf.writeUInt32LE(0, 4);
    compBuf.writeUInt32LE(65534, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint32([99], buffer, 12, 1)).to.equal(16);
    compBuf.writeUInt32LE(99, 12);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint64', () => {
    const buffer = Buffer.alloc(32);
    const compBuf = Buffer.alloc(32);
    const bignums = [new BN('9223372036854775807'), new BN('18446744073709551615'), new BN('123'), new BN('808080808080')];
    const buffers = [Buffer.from('ffffffffffffff7f', 'hex'), Buffer.from('ffffffffffffffff', 'hex'), Buffer.from('7b00000000000000', 'hex'), Buffer.from('90985e25bc000000', 'hex')];

    expect(serialize.uint64(bignums[0], buffer, 0)).to.equal(8);
    compBuf.set(buffers[0], 0);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint64(bignums[1], buffer, 8)).to.equal(16);
    compBuf.set(buffers[1], 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint64(bignums[2], buffer, 16)).to.equal(24);
    compBuf.set(buffers[2], 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.uint64(bignums[3], buffer, 16)).to.equal(24);
    compBuf.set(buffers[3], 16);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Uint64 Array', () => {
    const buffer = Buffer.alloc(36);
    const compBuf = Buffer.alloc(36);
    const bignums = [new BN('9223372036854775807'), new BN('18446744073709551615'), new BN('123'), new BN('808080808080')];
    const buffers = [Buffer.from('ffffffffffffff7f', 'hex'), Buffer.from('ffffffffffffffff', 'hex'), Buffer.from('7b00000000000000', 'hex'), Buffer.from('90985e25bc000000', 'hex')];

    expect(serialize.Array.uint64([bignums[0], bignums[1], bignums[2]], buffer, 0, 3)).to.equal(24);
    compBuf.set(buffers[0], 0);
    compBuf.set(buffers[1], 8);
    compBuf.set(buffers[2], 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.uint64([bignums[3]], buffer, 24, -1)).to.equal(36);
    compBuf.writeUInt32LE(1, 24);
    compBuf.set(buffers[3], 28);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int8', () => {
    const buffer = Buffer.alloc(4);
    const compBuf = Buffer.alloc(4);
    expect(serialize.int8(0, buffer, 0)).to.equal(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int8(-128, buffer, 1)).to.equal(2);
    compBuf.writeInt8(-128, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int8(127, buffer, 2)).to.equal(3);
    compBuf.writeInt8(127, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int8(16, buffer, 3)).to.equal(4);
    compBuf.writeInt8(16, 3);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int8 Array', () => {
    const buffer = Buffer.alloc(14);
    const compBuf = Buffer.alloc(14);
    expect(serialize.Array.int8([1], buffer, 0, -1)).to.equal(5);
    compBuf.writeUInt32LE(1);
    compBuf.writeInt8(1, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int8([127, 20], buffer, 5, -1)).to.equal(11);
    compBuf.writeUInt32LE(2, 5);
    compBuf.writeInt8(127, 9);
    compBuf.writeInt8(20, 10);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int8([126, 32, -124], buffer, 11, 3)).to.equal(14);
    compBuf.writeInt8(126, 11);
    compBuf.writeInt8(32, 12);
    compBuf.writeInt8(-124, 13);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int16', () => {
    const buffer = Buffer.alloc(8);
    const compBuf = Buffer.alloc(8);
    expect(serialize.int16(1, buffer, 0)).to.equal(2);
    compBuf.writeInt16LE(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int16(32767, buffer, 2)).to.equal(4);
    compBuf.writeInt16LE(32767, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int16(-32768, buffer, 4)).to.equal(6);
    compBuf.writeInt16LE(-32768, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int16(-22, buffer, 6)).to.equal(8);
    compBuf.writeInt16LE(-22, 6);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int16 Array', () => {
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.Array.int16([1], buffer, 0, -1)).to.equal(6);
    compBuf.writeUInt32LE(1);
    compBuf.writeInt16LE(1, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int16([32767, -32768], buffer, 6, 2)).to.equal(10);
    compBuf.writeInt16LE(32767, 6);
    compBuf.writeInt16LE(-32768, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int16([-22], buffer, 10, -1)).to.equal(16);
    compBuf.writeUInt32LE(1, 10);
    compBuf.writeInt16LE(-22, 14);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int32', () => {
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.int32(1, buffer, 0)).to.equal(4);
    compBuf.writeInt32LE(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int32(2147483647, buffer, 4)).to.equal(8);
    compBuf.writeInt32LE(2147483647, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int32(-2147483648, buffer, 8)).to.equal(12);
    compBuf.writeInt32LE(-2147483648, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int32(1235456848, buffer, 12)).to.equal(16);
    compBuf.writeInt32LE(1235456848, 12);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int32 Array', () => {
    const buffer = Buffer.alloc(24);
    const compBuf = Buffer.alloc(24);
    expect(serialize.Array.int32([1], buffer, 0, -1)).to.equal(8);
    compBuf.writeUInt32LE(1);
    compBuf.writeInt32LE(1, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int32([2147483647, -2147483648], buffer, 8, -1)).to.equal(20);
    compBuf.writeUInt32LE(2, 8);
    compBuf.writeInt32LE(2147483647, 12);
    compBuf.writeInt32LE(-2147483648, 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int32([1235456848], buffer, 20, 1)).to.equal(24);
    compBuf.writeInt32LE(1235456848, 20);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int64', () => {
    const buffer = Buffer.alloc(32);
    const compBuf = Buffer.alloc(32);
    const bignums = [new BN('9223372036854775807'), new BN('-9223372036854775807'), new BN('123456789'), new BN('-987654321')];
    const bufs = [Buffer.from('ffffffffffffff7f', 'hex'), Buffer.from('0100000000000080', 'hex'), Buffer.from('15cd5b0700000000', 'hex'), Buffer.from('4f9721c5ffffffff', 'hex')];

    expect(serialize.int64(bignums[0], buffer, 0)).to.equal(8);
    compBuf.set(bufs[0]);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int64(bignums[1], buffer, 8)).to.equal(16);
    compBuf.set(bufs[1], 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int64(bignums[2], buffer, 16)).to.equal(24);
    compBuf.set(bufs[2], 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int64(bignums[3], buffer, 24)).to.equal(32);
    compBuf.set(bufs[3], 24);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Int64 Array', () => {
    const buffer = Buffer.alloc(36);
    const compBuf = Buffer.alloc(36);
    const bignums = [new BN('9223372036854775807'), new BN('-9223372036854775807'), new BN('123456789'), new BN('-987654321')];
    const bufs = [Buffer.from('ffffffffffffff7f', 'hex'), Buffer.from('0100000000000080', 'hex'), Buffer.from('15cd5b0700000000', 'hex'), Buffer.from('4f9721c5ffffffff', 'hex')];

    expect(serialize.Array.int64([bignums[0], bignums[1]], buffer, 0, 1)).to.equal(16);
    compBuf.set(bufs[0], 0);
    compBuf.set(bufs[1], 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.int64([bignums[2], bignums[3]], buffer, 16, -1)).to.equal(36);
    compBuf.writeUInt32LE(2, 16);
    compBuf.set(bufs[2], 20);
    compBuf.set(bufs[3], 28);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Float32', () => {
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.float32(-0.11126, buffer, 0)).to.equal(4);
    compBuf.writeFloatLE(-0.11126, 0);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float32(-2456654, buffer, 4)).to.equal(8);
    compBuf.writeFloatLE(-2456654, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float32(19875664, buffer, 8)).to.equal(12);
    compBuf.writeFloatLE(19875664, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float32(0.00000012, buffer, 12)).to.equal(16);
    compBuf.writeFloatLE(0.00000012, 12);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Float32 Array', () => {
    const buffer = Buffer.alloc(32);
    const compBuf = Buffer.alloc(32);
    expect(serialize.Array.float32([-0.11126, -2456654], buffer, 0, -1)).to.equal(12);
    compBuf.writeUInt32LE(2);
    compBuf.writeFloatLE(-0.11126, 4);
    compBuf.writeFloatLE(-2456654, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.float32([19875664], buffer, 12, 1)).to.equal(16);
    compBuf.writeFloatLE(19875664, 12);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.float32([0.00000012, -499, 80], buffer, 16, -1)).to.equal(32);
    compBuf.writeUInt32LE(3, 16);
    compBuf.writeFloatLE(0.00000012, 20);
    compBuf.writeFloatLE(-499, 24);
    compBuf.writeFloatLE(80, 28);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Float64', () => {
    const buffer = Buffer.alloc(32);
    const compBuf = Buffer.alloc(32);
    expect(serialize.float64(-0.11126, buffer, 0)).to.equal(8);
    compBuf.writeDoubleLE(-0.11126, 0);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float64(-2456654, buffer, 8)).to.equal(16);
    compBuf.writeDoubleLE(-2456654, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float64(19875665, buffer, 16)).to.equal(24);
    compBuf.writeDoubleLE(19875665, 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.float64(2.23e-16, buffer, 24)).to.equal(32);
    compBuf.writeDoubleLE(2.23e-16, 24);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Float64 Array', () => {
    const buffer = Buffer.alloc(36);
    const compBuf = Buffer.alloc(36);
    expect(serialize.Array.float64([-0.11126, -2456654, 19875665], buffer, 0, 3)).to.equal(24);
    compBuf.writeDoubleLE(-0.11126, 0);
    compBuf.writeDoubleLE(-2456654, 8);
    compBuf.writeDoubleLE(19875665, 16);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.float64([2.23e-16], buffer, 24, -1)).to.equal(36);
    compBuf.writeUInt32LE(1, 24);
    compBuf.writeDoubleLE(2.23e-16, 28);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Time', () => {
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.time({ secs: 1234560, nsecs: 565600 }, buffer, 0)).to.equal(8);
    compBuf.writeInt32LE(1234560, 0);
    compBuf.writeInt32LE(565600, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.time({ secs: -89855022, nsecs: 14545 }, buffer, 8)).to.equal(16);
    compBuf.writeInt32LE(-89855022, 8);
    compBuf.writeInt32LE(14545, 12);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Time Array', () => {
    const buffer = Buffer.alloc(28);
    const compBuf = Buffer.alloc(28);
    expect(serialize.Array.time([{ secs: 1234560, nsecs: 565600 }, { secs: 0, nsecs: 0 }], buffer, 0, -1)).to.equal(20);
    compBuf.writeUInt32LE(2, 0);
    compBuf.writeInt32LE(1234560, 4);
    compBuf.writeInt32LE(565600, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.time([{ secs: -89855022, nsecs: 14545 }], buffer, 20, 1)).to.equal(28);
    compBuf.writeInt32LE(-89855022, 20);
    compBuf.writeInt32LE(14545, 24);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Duration', () => {
    // duration has the same representation as time
    const buffer = Buffer.alloc(16);
    const compBuf = Buffer.alloc(16);
    expect(serialize.duration({ secs: 1234560, nsecs: 565600 }, buffer, 0)).to.equal(8);
    compBuf.writeInt32LE(1234560, 0);
    compBuf.writeInt32LE(565600, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.duration({ secs: -89855022, nsecs: 14545 }, buffer, 8)).to.equal(16);
    compBuf.writeInt32LE(-89855022, 8);
    compBuf.writeInt32LE(14545, 12);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Duration Array', () => {
    const buffer = Buffer.alloc(28);
    const compBuf = Buffer.alloc(28);
    expect(serialize.Array.duration([{ secs: 1234560, nsecs: 565600 }, { secs: 0, nsecs: 0 }], buffer, 0, -1)).to.equal(20);
    compBuf.writeUInt32LE(2, 0);
    compBuf.writeInt32LE(1234560, 4);
    compBuf.writeInt32LE(565600, 8);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.duration([{ secs: -89855022, nsecs: 14545 }], buffer, 20, 1)).to.equal(28);
    compBuf.writeInt32LE(-89855022, 20);
    compBuf.writeInt32LE(14545, 24);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Char', () => {
    const buffer = Buffer.alloc(4);
    const compBuf = Buffer.alloc(4);
    expect(serialize.char(0, buffer, 0)).to.equal(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.char(255, buffer, 1)).to.equal(2);
    compBuf.writeUInt8(255, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.char(20, buffer, 2)).to.equal(3);
    compBuf.writeUInt8(20, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.char(86, buffer, 3)).to.equal(4);
    compBuf.writeUInt8(86, 3);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Char Array', () => {
    const buffer = Buffer.alloc(14);
    const compBuf = Buffer.alloc(14);
    expect(serialize.Array.char([0], buffer, 0, -1)).to.equal(5);
    compBuf.writeUInt32LE(1);
    compBuf.writeUInt8(0, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.char([255, 20], buffer, 5, 2)).to.equal(7);
    compBuf.writeUInt8(255, 5);
    compBuf.writeUInt8(20, 6);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.char([126, 32, 192], buffer, 7, -1)).to.equal(14);
    compBuf.writeUInt32LE(3, 7);
    compBuf.writeUInt8(126, 11);
    compBuf.writeUInt8(32, 12);
    compBuf.writeUInt8(192, 13);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Byte', () => {
    const buffer = Buffer.alloc(4);
    const compBuf = Buffer.alloc(4);
    expect(serialize.byte(0, buffer, 0)).to.equal(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.int8(-128, buffer, 1)).to.equal(2);
    compBuf.writeInt8(-128, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.byte(127, buffer, 2)).to.equal(3);
    compBuf.writeInt8(127, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.byte(16, buffer, 3)).to.equal(4);
    compBuf.writeInt8(16, 3);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Byte Array', () => {
    const buffer = Buffer.alloc(14);
    const compBuf = Buffer.alloc(14);
    expect(serialize.Array.byte([1], buffer, 0, -1)).to.equal(5);
    compBuf.writeUInt32LE(1);
    compBuf.writeInt8(1, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.byte([127, 20], buffer, 5, -1)).to.equal(11);
    compBuf.writeUInt32LE(2, 5);
    compBuf.writeInt8(127, 9);
    compBuf.writeInt8(20, 10);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.byte([126, 32, -124], buffer, 11, 3)).to.equal(14);
    compBuf.writeInt8(126, 11);
    compBuf.writeInt8(32, 12);
    compBuf.writeInt8(-124, 13);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Bool', () => {
    const buffer = Buffer.alloc(4);
    const compBuf = Buffer.alloc(4);
    expect(serialize.bool(true, buffer, 0)).to.equal(1);
    compBuf.writeInt8(1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.bool(false, buffer, 1)).to.equal(2);
    compBuf.writeInt8(0, 1);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.bool(true, buffer, 2)).to.equal(3);
    compBuf.writeInt8(1, 2);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.bool(true, buffer, 3)).to.equal(4);
    compBuf.writeInt8(1, 3);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('Bool Array', () => {
    const buffer = Buffer.alloc(14);
    const compBuf = Buffer.alloc(14);
    expect(serialize.Array.bool([true], buffer, 0, -1)).to.equal(5);
    compBuf.writeUInt32LE(1);
    compBuf.writeInt8(1, 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.bool([false, true], buffer, 5, 2)).to.equal(7);
    compBuf.writeInt8(0, 5);
    compBuf.writeInt8(1, 6);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.bool([false, true, true], buffer, 7, -1)).to.equal(14);
    compBuf.writeUInt32LE(3, 7);
    compBuf.writeInt8(0, 11);
    compBuf.writeInt8(1, 12);
    compBuf.writeInt8(1, 13);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('String', () => {
    const buffer = Buffer.alloc(39);
    const compBuf = Buffer.alloc(39);
    expect(serialize.string('hi', buffer, 0)).to.equal(6);
    compBuf.writeUInt32LE(2);
    compBuf.write('hi', 4);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.string('there', buffer, 6)).to.equal(15);
    compBuf.writeUInt32LE(5, 6);
    compBuf.write('there', 10);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.string('1', buffer, 15)).to.equal(20);
    compBuf.writeUInt32LE(1, 15);
    compBuf.write('1', 19);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.string('こんにちは', buffer, 20)).to.equal(39);
    compBuf.writeUInt32LE(15, 20);
    compBuf.write('こんにちは', 24);
    expect(buffer.equals(compBuf)).to.be.true;
  });

  it('String Array', () => {
    const buffer = Buffer.alloc(47);
    const compBuf = Buffer.alloc(47);
    expect(serialize.Array.string(['hi', 'there', 'こんにちは'], buffer, 0, -1)).to.equal(38);
    compBuf.writeUInt32LE(3);
    compBuf.writeUInt32LE(2, 4);
    compBuf.write('hi', 8);
    compBuf.writeUInt32LE(5, 10);
    compBuf.write('there', 14);
    compBuf.writeUInt32LE(15, 19);
    compBuf.write('こんにちは', 23);
    expect(buffer.equals(compBuf)).to.be.true;
    expect(serialize.Array.string(['9'], buffer, 38, 1)).to.equal(43);
    compBuf.writeUInt32LE(1, 38);
    compBuf.write('9', 42);
    expect(buffer.equals(compBuf)).to.be.true;
  });
});