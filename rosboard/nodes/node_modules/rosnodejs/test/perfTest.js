'use strict';

/**
 * Quick test for serialization, deserialization Performance
 */

const expect = require('chai').expect;
const rosnodejs = require('../src/index.js');
const TfStamped = rosnodejs.require('geometry_msgs').msg.TransformStamped;
const TfMessage = rosnodejs.require('tf2_msgs').msg.TFMessage;
const Image = rosnodejs.require('sensor_msgs').msg.Image;
const Header = rosnodejs.require('std_msgs').msg.Header;

const header = new Header({seq: 100, stamp: {secs: 20, nsecs: 100010}, frame_id: 'test_cam'});

function getSeconds(hrTime) {
  return hrTime[0] + hrTime[1] / 1e9;
}

function getMB(bytes) {
  return bytes / 1e6;
}

function getBandwidth(bytes, hrTime) {
  return (getMB(bytes) / getSeconds(hrTime)).toFixed(3);
}

const NUM_CYCLES = 100;

let hrTime;
let deltaT;

console.log('=== Serialization Performance Test ===');
console.log(' ==');
console.log(' == Image Test');
console.log(' == Cycles: %d', NUM_CYCLES);
console.log(' ==');


let image;
console.time('Create Image');
let width = 1280,
  height = 800,
  step = width * 3;
for (let i = 0; i < NUM_CYCLES; ++i ) {
  image = new Image({
    width: width,
    height: height,
    encoding: 'bgr8',
    step: step,
    data: Buffer.allocUnsafe(step * height),
    header
  });
}
console.timeEnd('Create Image');

let bytesPerCycle;
let bufsize;
console.time('Determine Message Size');
for (let i = 0; i < NUM_CYCLES; ++i) {
  bufsize = Image.getMessageSize(image);
}
console.timeEnd('Determine Message Size');
bytesPerCycle = bufsize * NUM_CYCLES;

console.log('Buffer size: %d', bufsize);

console.time('allocate buffer');
let buffer;
for (let i = 0; i < NUM_CYCLES; ++i) {
  buffer = new Buffer(bufsize);
}
console.timeEnd('allocate buffer');

console.time('Serialize');
hrTime = process.hrtime();
for (let i = 0; i < NUM_CYCLES; ++i) {
  Image.serialize(image, buffer, 0);
}
deltaT = process.hrtime(hrTime);
console.timeEnd('Serialize');
console.log(`Serialized BW: ${getBandwidth(bytesPerCycle, deltaT)}MB`);

console.time('Deserialize');
let deserialized;
hrTime = process.hrtime();
for (let i = 0; i < NUM_CYCLES; ++i) {
  deserialized = Image.deserialize(buffer, [0]);
}
deltaT = process.hrtime(hrTime);
console.timeEnd('Deserialize');
console.log(`Deserialized BW: ${getBandwidth(bytesPerCycle, deltaT)}MB`);

// verify equality!
expect(deserialized).to.deep.equal(image);

const NUM_TFS = 1000;

console.log(' ==');
console.log(' == TF Test');
console.log(' == Cycles: %d', NUM_CYCLES);
console.log(' == # of Transforms: %d', NUM_TFS);
console.log(' ==');

let tfStamped = new TfStamped();
tfStamped.header.frame_id = 'test_parent_frame';
tfStamped.child_frame_id = 'test_frame';

console.time('Create TfMessage');
let tfMessage;
for (let i = 0; i < NUM_CYCLES; ++i) {
  tfMessage = new TfMessage();
  for (let j = 0; j < NUM_TFS; ++j) {
    let tf = new TfStamped(tfStamped);
    tfMessage.transforms.push(tf);
  }
}
console.timeEnd('Create TfMessage');

console.time('Determine Message Size');
for (let i = 0; i < NUM_CYCLES; ++i) {
  bufsize = TfMessage.getMessageSize(tfMessage);
}
console.timeEnd('Determine Message Size');
bytesPerCycle = bufsize * NUM_CYCLES;

console.log('Buffer size: %d', bufsize);

console.time('Allocate buffer');
for (let i = 0; i < NUM_CYCLES; ++i) {
  buffer = new Buffer(bufsize);
}
console.timeEnd('Allocate buffer');

console.time('Serialize');
hrTime = process.hrtime();
for (let i = 0; i < NUM_CYCLES; ++i) {
  TfMessage.serialize(tfMessage, buffer, 0);
}
deltaT = process.hrtime(hrTime);
console.timeEnd('Serialize');
console.log(`Serialized BW: ${getBandwidth(bytesPerCycle, deltaT)}MB`);

console.time('Deserialize');
hrTime = process.hrtime();
for (let i = 0; i < NUM_CYCLES; ++i) {
  deserialized = TfMessage.deserialize(buffer, [0]);
}
deltaT = process.hrtime(hrTime);
console.timeEnd('Deserialize');
console.log(`Deserialized BW: ${getBandwidth(bytesPerCycle, deltaT)}MB`);

// verify equality!
expect(deserialized).to.deep.equal(tfMessage);