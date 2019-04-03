'use strict';

const chai = require('chai');
const expect = chai.expect;
const serialize = require('../lib/base_serialize.js');
const deserialize = require('../lib/base_deserialize.js');
const BN = require('bn.js');

describe('Deserialization Tests', () => {
  it('Uint8', () => {
    const buf = Buffer.alloc(5);
    serialize.Array.uint8([1, 2, 3, 255, 0], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.uint8(buf, bufferOffset)).to.equal(1);
    expect(bufferOffset[0]).to.equal(1);
    expect(deserialize.uint8(buf, bufferOffset)).to.equal(2);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.uint8(buf, bufferOffset)).to.equal(3);
    expect(bufferOffset[0]).to.equal(3);
    expect(deserialize.uint8(buf, bufferOffset)).to.equal(255);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.uint8(buf, bufferOffset)).to.equal(0);
    expect(bufferOffset[0]).to.equal(5);
  });

  it('Uint8 Array', () => {
    const buf = Buffer.alloc(14);
    serialize.Array.uint8([1, 2], buf, 0, -1);
    serialize.Array.uint8([3, 1, 2], buf, 6, 3);
    serialize.Array.uint8([3], buf, 9, -1);

    const bufferOffset = [0];
    expect(Array.from(deserialize.Array.uint8(buf, bufferOffset, -1))).to.deep.equal([1, 2]);
    expect(bufferOffset[0]).to.equal(6);
    expect(Array.from(deserialize.Array.uint8(buf, bufferOffset, 3))).to.deep.equal([3, 1, 2]);
    expect(bufferOffset[0]).to.equal(9);
    expect(Array.from(deserialize.Array.uint8(buf, bufferOffset, -1))).to.deep.equal([3]);
    expect(bufferOffset[0]).to.equal(14);
  });

  it('Uint16', () => {
    const buf = Buffer.alloc(14);
    serialize.Array.uint16([65024, 257, 254, 65535], buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.uint16(buf, bufferOffset)).to.equal(65024);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.uint16(buf, bufferOffset)).to.equal(257);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.uint16(buf, bufferOffset)).to.equal(254);
    expect(bufferOffset[0]).to.equal(6);
    expect(deserialize.uint16(buf, bufferOffset)).to.equal(65535);
    expect(bufferOffset[0]).to.equal(8);
  });

  it('Uint16 Array', () => {
    const buf = Buffer.alloc(20);
    serialize.Array.uint16([65024, 257], buf, 0, -1);
    serialize.Array.uint16([254], buf, 8, 1);
    serialize.Array.uint16([20, 16, 2], buf, 10, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.uint16(buf, bufferOffset, -1)).to.deep.equal([65024, 257]);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.Array.uint16(buf, bufferOffset, 1)).to.deep.equal([254]);
    expect(bufferOffset[0]).to.equal(10);
    expect(deserialize.Array.uint16(buf, bufferOffset, -1)).to.deep.equal([20, 16, 2]);
    expect(bufferOffset[0]).to.equal(20);
  });

  it('Uint32', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.uint32([4294967295, 0, 369131520, 168427616], buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.uint32(buf, bufferOffset)).to.equal(4294967295);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.uint32(buf, bufferOffset)).to.equal(0);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.uint32(buf, bufferOffset)).to.equal(369131520);
    expect(bufferOffset[0]).to.equal(12);
    expect(deserialize.uint32(buf, bufferOffset)).to.equal(168427616);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Uint32 Array', () => {
    const buf = Buffer.alloc(28);
    serialize.Array.uint32([4294967295, 0], buf, 0, -1);
    serialize.Array.uint32([369131520], buf, 12, -1);
    serialize.Array.uint32([168427616, 12], buf, 20, 2);

    const bufferOffset = [0];
    expect(deserialize.Array.uint32(buf, bufferOffset, -1)).to.deep.equal([4294967295, 0]);
    expect(bufferOffset[0]).to.equal(12);
    expect(deserialize.Array.uint32(buf, bufferOffset, -1)).to.deep.equal([369131520]);
    expect(bufferOffset[0]).to.equal(20);
    expect(deserialize.Array.uint32(buf, bufferOffset, 2)).to.deep.equal([168427616, 12]);
    expect(bufferOffset[0]).to.equal(28);
  });

  it('Uint64', () => {
    const buf = Buffer.alloc(32);
    const bignums = [new BN('9223372036854775807'), new BN('18446744073709551615'), new BN('123'), new BN('8080808080')];
    serialize.Array.uint64(bignums, buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.uint64(buf, bufferOffset).eq(bignums[0])).to.be.true;
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.uint64(buf, bufferOffset).eq(bignums[1])).to.be.true;
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.uint64(buf, bufferOffset).eq(bignums[2])).to.be.true;
    expect(bufferOffset[0]).to.equal(24);
    expect(deserialize.uint64(buf, bufferOffset).eq(bignums[3])).to.be.true;
    expect(bufferOffset[0]).to.equal(32);
  });

  it('Uint64 Array', () => {
    const buf = Buffer.alloc(36);

    const bignums = [new BN('9223372036854775807'), new BN('18446744073709551615'), new BN('123'), new BN('8080808080')];

    serialize.Array.uint64(bignums.slice(0, 3), buf, 0, -1);
    serialize.Array.uint64(bignums.slice(3, 4), buf, 28, 1);

    const bufferOffset = [0];
    let arr = deserialize.Array.uint64(buf, bufferOffset, -1);
    expect(arr.length).to.equal(3);
    expect(arr[0].eq(bignums[0])).to.be.true;
    expect(arr[1].eq(bignums[1])).to.be.true;
    expect(arr[2].eq(bignums[2])).to.be.true;
    expect(bufferOffset[0]).to.equal(28);

    arr = deserialize.Array.uint64(buf, bufferOffset, 1);
    expect(bufferOffset[0]).to.equal(36);
    expect(arr.length).to.equal(1);
    expect(arr[0].eq(bignums[3])).to.be.true;
  });

  it('Int8', () => {
    const buf = Buffer.alloc(5);
    serialize.Array.int8([1, -2, 3, -128, 127], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.int8(buf, bufferOffset)).to.equal(1);
    expect(bufferOffset[0]).to.equal(1);
    expect(deserialize.int8(buf, bufferOffset)).to.equal(-2);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.int8(buf, bufferOffset)).to.equal(3);
    expect(bufferOffset[0]).to.equal(3);
    expect(deserialize.int8(buf, bufferOffset)).to.equal(-128);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.int8(buf, bufferOffset)).to.equal(127);
    expect(bufferOffset[0]).to.equal(5);
  });

  it('Int8 Array', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.int8([1, 2], buf, 0, -1);
    serialize.Array.int8([3, 1, 2], buf, 6, 3);
    serialize.Array.int8([3, -126, 24], buf, 9, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.int8(buf, bufferOffset, -1)).to.deep.equal([1, 2]);
    expect(bufferOffset[0]).to.equal(6);
    expect(deserialize.Array.int8(buf, bufferOffset, 3)).to.deep.equal([3, 1, 2]);
    expect(bufferOffset[0]).to.equal(9);
    expect(deserialize.Array.int8(buf, bufferOffset, -1)).to.deep.equal([3, -126, 24]);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Int16', () => {
    const buf = Buffer.alloc(8);
    serialize.Array.int16([32767, 257, -32768, 124], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.int16(buf, bufferOffset)).to.equal(32767);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.int16(buf, bufferOffset)).to.equal(257);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.int16(buf, bufferOffset)).to.equal(-32768);
    expect(bufferOffset[0]).to.equal(6);
    expect(deserialize.int16(buf, bufferOffset)).to.equal(124);
    expect(bufferOffset[0]).to.equal(8);
  });

  it('Int16 Array', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.int16([1, 32767], buf, 0, 2);
    serialize.Array.int16([-32768], buf, 4, -1);
    serialize.Array.int16([-22], buf, 10, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.int16(buf, bufferOffset, 2)).to.deep.equal([1, 32767]);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.Array.int16(buf, bufferOffset, -1)).to.deep.equal([-32768]);
    expect(bufferOffset[0]).to.equal(10);
    expect(deserialize.Array.int16(buf, bufferOffset, -1)).to.deep.equal([-22]);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Int32', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.int32([1, 2147483647, -2147483648, 1235456848], buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.int32(buf, bufferOffset)).to.equal(1);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.int32(buf, bufferOffset)).to.equal(2147483647);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.int32(buf, bufferOffset)).to.equal(-2147483648);
    expect(bufferOffset[0]).to.equal(12);
    expect(deserialize.int32(buf, bufferOffset)).to.equal(1235456848);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Int32 Array', () => {
    const buf = Buffer.alloc(28);
    serialize.Array.int32([1, 2147483647], buf, 0, 2);
    serialize.Array.int32([-2147483648], buf, 8, -1);
    serialize.Array.int32([1235456848, 12], buf, 16, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.int32(buf, bufferOffset, 2)).to.deep.equal([1, 2147483647]);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.Array.int32(buf, bufferOffset, -1)).to.deep.equal([-2147483648]);
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.Array.int32(buf, bufferOffset, -1)).to.deep.equal([1235456848, 12]);
    expect(bufferOffset[0]).to.equal(28);
  });

  it('Int64', () => {
    const buf = Buffer.alloc(32);
    const numbers = ['9223372036854775807', '-9223372036854775807', '123456789', '-987654321'];
    const bignums = numbers.map(num => new BN(num));
    serialize.Array.int64(bignums, buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.int64(buf, bufferOffset).eq(bignums[0])).to.be.true;
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.int64(buf, bufferOffset).eq(bignums[1])).to.be.true;
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.int64(buf, bufferOffset).eq(bignums[2])).to.be.true;
    expect(bufferOffset[0]).to.equal(24);
    expect(deserialize.int64(buf, bufferOffset).eq(bignums[3])).to.be.true;
    expect(bufferOffset[0]).to.equal(32);
  });

  it('Int64 Array', () => {
    const buf = Buffer.alloc(36);
    const numbers = ['9223372036854775807', '-9223372036854775807', '123456789', '-987654321'];
    const bignums = numbers.map(num => new BN(num));
    serialize.Array.int64(bignums.slice(0, 2), buf, 0, 2);
    serialize.Array.int64(bignums.slice(2, 4), buf, 16, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.int64(buf, bufferOffset, 2);
    expect(arr[0].eq(bignums[0])).to.be.true;
    expect(arr[1].eq(bignums[1])).to.be.true;
    expect(arr.length).to.equal(2);
    expect(bufferOffset[0]).to.equal(16);
    arr = deserialize.Array.int64(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(36);
    expect(arr[0].eq(bignums[2])).to.be.true;
    expect(arr[1].eq(bignums[3])).to.be.true;
    expect(arr.length).to.equal(2);
  });

  it('Float32', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.float32([0.00000012, 19875664, -2456654, -0.011], buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.float32(buf, bufferOffset)).to.be.closeTo(0.00000012, 1e-7);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.float32(buf, bufferOffset)).to.be.closeTo(19875664, 1e-7);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.float32(buf, bufferOffset)).to.be.closeTo(-2456654, 1e-7);
    expect(bufferOffset[0]).to.equal(12);
    expect(deserialize.float32(buf, bufferOffset)).to.be.closeTo(-0.011, 1e-7);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Float32 Array', () => {
    const buf = Buffer.alloc(28);
    serialize.Array.float32([0.00000012, 19875664], buf, 0, 2);
    serialize.Array.float32([-2456654], buf, 8, -1);
    serialize.Array.float32([-0.011, 12], buf, 16, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.float32(buf, bufferOffset, 2);
    expect(bufferOffset[0]).to.equal(8);
    expect(arr.length).to.equal(2);
    expect(arr[0]).to.be.closeTo(0.00000012, 1e-7);
    expect(arr[1]).to.be.closeTo(19875664, 1e-7);

    arr = deserialize.Array.float32(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(16);
    expect(arr.length).to.equal(1);
    expect(arr[0]).to.be.closeTo(-2456654, 1e-7);

    arr = deserialize.Array.float32(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(28);
    expect(arr.length).to.equal(2);
    expect(arr[0]).to.be.closeTo(-0.011, 1e-7);
    expect(arr[1]).to.be.closeTo(12, 1e-7);
  });

  it('Float64', () => {
    const buf = Buffer.alloc(32);
    serialize.Array.float64([0.00000012, -2456654, 2.23e-16, 19875665], buf, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.float64(buf, bufferOffset)).to.be.closeTo(0.00000012, Number.EPSILON);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.float64(buf, bufferOffset)).to.be.closeTo(-2456654, Number.EPSILON);
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.float64(buf, bufferOffset)).to.be.closeTo(2.23e-16, Number.EPSILON);
    expect(bufferOffset[0]).to.equal(24);
    expect(deserialize.float64(buf, bufferOffset)).to.be.closeTo(19875665, Number.EPSILON);
    expect(bufferOffset[0]).to.equal(32);
  });

  it('Float64 Array', () => {
    const buf = Buffer.alloc(48);
    serialize.Array.float64([0.00000012, 19875664], buf, 0, 2);
    serialize.Array.float64([-2456654], buf, 16, -1);
    serialize.Array.float64([-0.011, 12], buf, 28, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.float64(buf, bufferOffset, 2);
    expect(bufferOffset[0]).to.equal(16);
    expect(arr.length).to.equal(2);
    expect(arr[0]).to.be.closeTo(0.00000012, Number.EPSILON);
    expect(arr[1]).to.be.closeTo(19875664, Number.EPSILON);

    arr = deserialize.Array.float64(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(28);
    expect(arr.length).to.equal(1);
    expect(arr[0]).to.be.closeTo(-2456654, Number.EPSILON);

    arr = deserialize.Array.float64(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(48);
    expect(arr.length).to.equal(2);
    expect(arr[0]).to.be.closeTo(-0.011, Number.EPSILON);
    expect(arr[1]).to.be.closeTo(12, 1e-7);
  });

  it('Time', () => {
    const buffer = Buffer.alloc(32);
    const times = [{ secs: 0, nsecs: 0 }, { secs: 32768, nsecs: 215888 }, { secs: 5489, nsecs: 5653 }, { secs: 81232, nsecs: 121 }];
    serialize.Array.time(times, buffer, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.time(buffer, bufferOffset)).to.deep.equal(times[0]);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.time(buffer, bufferOffset)).to.deep.equal(times[1]);
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.time(buffer, bufferOffset)).to.deep.equal(times[2]);
    expect(bufferOffset[0]).to.equal(24);
    expect(deserialize.time(buffer, bufferOffset)).to.deep.equal(times[3]);
    expect(bufferOffset[0]).to.equal(32);
  });

  it('Time Array', () => {
    const buffer = Buffer.alloc(36);
    const times = [{ secs: 0, nsecs: 0 }, { secs: 32768, nsecs: 215888 }, { secs: 5489, nsecs: 5653 }, { secs: 81232, nsecs: 121 }];
    serialize.Array.time(times.slice(0, 3), buffer, 0, 3);
    serialize.Array.time(times.slice(3, 4), buffer, 24, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.time(buffer, bufferOffset, 3);
    expect(bufferOffset[0]).to.equal(24);
    expect(arr.length).to.equal(3);
    expect(arr[0]).to.deep.equal(times[0]);
    expect(arr[1]).to.deep.equal(times[1]);
    expect(arr[2]).to.deep.equal(times[2]);

    arr = deserialize.Array.time(buffer, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(36);
    expect(arr.length).to.equal(1);
    expect(arr[0]).to.deep.equal(times[3]);
  });

  it('Duration', () => {
    const buffer = Buffer.alloc(32);
    const times = [{ secs: 0, nsecs: 0 }, { secs: -32768, nsecs: -215888 }, { secs: 5489, nsecs: 5653 }, { secs: 81232, nsecs: 121 }];
    serialize.Array.duration(times, buffer, 0, 4);

    const bufferOffset = [0];
    expect(deserialize.duration(buffer, bufferOffset)).to.deep.equal(times[0]);
    expect(bufferOffset[0]).to.equal(8);
    expect(deserialize.duration(buffer, bufferOffset)).to.deep.equal(times[1]);
    expect(bufferOffset[0]).to.equal(16);
    expect(deserialize.duration(buffer, bufferOffset)).to.deep.equal(times[2]);
    expect(bufferOffset[0]).to.equal(24);
    expect(deserialize.duration(buffer, bufferOffset)).to.deep.equal(times[3]);
    expect(bufferOffset[0]).to.equal(32);
  });

  it('Duration Array', () => {
    const buffer = Buffer.alloc(36);
    const times = [{ secs: 0, nsecs: 0 }, { secs: 32768, nsecs: 215888 }, { secs: -5489, nsecs: -5653 }, { secs: 81232, nsecs: 121 }];
    serialize.Array.duration(times.slice(0, 3), buffer, 0, 3);
    serialize.Array.duration(times.slice(3, 4), buffer, 24, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.duration(buffer, bufferOffset, 3);
    expect(bufferOffset[0]).to.equal(24);
    expect(arr.length).to.equal(3);
    expect(arr[0]).to.deep.equal(times[0]);
    expect(arr[1]).to.deep.equal(times[1]);
    expect(arr[2]).to.deep.equal(times[2]);

    arr = deserialize.Array.duration(buffer, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(36);
    expect(arr.length).to.equal(1);
    expect(arr[0]).to.deep.equal(times[3]);
  });

  it('Char', () => {
    const buf = Buffer.alloc(5);
    serialize.Array.char([1, 2, 3, 255, 0], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.char(buf, bufferOffset)).to.equal(1);
    expect(bufferOffset[0]).to.equal(1);
    expect(deserialize.char(buf, bufferOffset)).to.equal(2);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.char(buf, bufferOffset)).to.equal(3);
    expect(bufferOffset[0]).to.equal(3);
    expect(deserialize.char(buf, bufferOffset)).to.equal(255);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.char(buf, bufferOffset)).to.equal(0);
    expect(bufferOffset[0]).to.equal(5);
  });

  it('Char Array', () => {
    const buf = Buffer.alloc(14);
    serialize.Array.char([1, 2], buf, 0, -1);
    serialize.Array.char([3, 1, 2], buf, 6, 3);
    serialize.Array.char([3], buf, 9, -1);

    const bufferOffset = [0];
    expect(Array.from(deserialize.Array.char(buf, bufferOffset, -1))).to.deep.equal([1, 2]);
    expect(bufferOffset[0]).to.equal(6);
    expect(Array.from(deserialize.Array.char(buf, bufferOffset, 3))).to.deep.equal([3, 1, 2]);
    expect(bufferOffset[0]).to.equal(9);
    expect(Array.from(deserialize.Array.char(buf, bufferOffset, -1))).to.deep.equal([3]);
    expect(bufferOffset[0]).to.equal(14);
  });

  it('Byte', () => {
    const buf = Buffer.alloc(5);
    serialize.Array.byte([1, -2, 3, -128, 127], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.byte(buf, bufferOffset)).to.equal(1);
    expect(bufferOffset[0]).to.equal(1);
    expect(deserialize.byte(buf, bufferOffset)).to.equal(-2);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.byte(buf, bufferOffset)).to.equal(3);
    expect(bufferOffset[0]).to.equal(3);
    expect(deserialize.byte(buf, bufferOffset)).to.equal(-128);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.byte(buf, bufferOffset)).to.equal(127);
    expect(bufferOffset[0]).to.equal(5);
  });

  it('Byte Array', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.byte([1, 2], buf, 0, -1);
    serialize.Array.byte([3, 1, 2], buf, 6, 3);
    serialize.Array.byte([3, -126, 24], buf, 9, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.byte(buf, bufferOffset, -1)).to.deep.equal([1, 2]);
    expect(bufferOffset[0]).to.equal(6);
    expect(deserialize.Array.byte(buf, bufferOffset, 3)).to.deep.equal([3, 1, 2]);
    expect(bufferOffset[0]).to.equal(9);
    expect(deserialize.Array.byte(buf, bufferOffset, -1)).to.deep.equal([3, -126, 24]);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('Bool', () => {
    const buf = Buffer.alloc(5);
    serialize.Array.bool([true, false, true, false, false], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.bool(buf, bufferOffset)).to.equal(true);
    expect(bufferOffset[0]).to.equal(1);
    expect(deserialize.bool(buf, bufferOffset)).to.equal(false);
    expect(bufferOffset[0]).to.equal(2);
    expect(deserialize.bool(buf, bufferOffset)).to.equal(true);
    expect(bufferOffset[0]).to.equal(3);
    expect(deserialize.bool(buf, bufferOffset)).to.equal(false);
    expect(bufferOffset[0]).to.equal(4);
    expect(deserialize.bool(buf, bufferOffset)).to.equal(false);
    expect(bufferOffset[0]).to.equal(5);
  });

  it('Bool Array', () => {
    const buf = Buffer.alloc(16);
    serialize.Array.bool([false, false], buf, 0, -1);
    serialize.Array.bool([true, true, false], buf, 6, 3);
    serialize.Array.bool([true, false, true], buf, 9, -1);

    const bufferOffset = [0];
    expect(deserialize.Array.bool(buf, bufferOffset, -1)).to.deep.equal([false, false]);
    expect(bufferOffset[0]).to.equal(6);
    expect(deserialize.Array.bool(buf, bufferOffset, 3)).to.deep.equal([true, true, false]);
    expect(bufferOffset[0]).to.equal(9);
    expect(deserialize.Array.bool(buf, bufferOffset, -1)).to.deep.equal([true, false, true]);
    expect(bufferOffset[0]).to.equal(16);
  });

  it('String', () => {
    const buf = Buffer.alloc(47);
    serialize.Array.string(["Hello", "World", "一番", "Strings", "here"], buf, 0, 5);

    const bufferOffset = [0];
    expect(deserialize.string(buf, bufferOffset)).to.equal("Hello");
    expect(bufferOffset[0]).to.equal(9);
    expect(deserialize.string(buf, bufferOffset)).to.equal("World");
    expect(bufferOffset[0]).to.equal(18);
    expect(deserialize.string(buf, bufferOffset)).to.equal("一番");
    expect(bufferOffset[0]).to.equal(28);
    expect(deserialize.string(buf, bufferOffset)).to.equal("Strings");
    expect(bufferOffset[0]).to.equal(39);
    expect(deserialize.string(buf, bufferOffset)).to.equal("here");
    expect(bufferOffset[0]).to.equal(47);
  });

  it('String Array', () => {
    const buf = Buffer.alloc(72);
    const strings = ["Hello", "World", "一番", "Strings", "here", "019283dlakjsd"];
    serialize.Array.string(strings.slice(0, 2), buf, 0, 2);
    serialize.Array.string(strings.slice(2, 5), buf, 18, -1);
    serialize.Array.string(strings.slice(5, 6), buf, 51, -1);

    const bufferOffset = [0];
    let arr = deserialize.Array.string(buf, bufferOffset, 2);
    expect(bufferOffset[0]).to.equal(18);
    expect(arr.length).to.equal(2);
    expect(arr[0]).to.equal(strings[0]);
    expect(arr[1]).to.equal(strings[1]);

    arr = deserialize.Array.string(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(51);
    expect(arr.length).to.equal(3);
    expect(arr[0]).to.equal(strings[2]);
    expect(arr[1]).to.equal(strings[3]);
    expect(arr[2]).to.equal(strings[4]);

    arr = deserialize.Array.string(buf, bufferOffset, -1);
    expect(bufferOffset[0]).to.equal(72);
    expect(arr.length).to.equal(1);
    expect(arr[0]).to.equal(strings[5]);
  });
});