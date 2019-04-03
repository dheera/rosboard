'use strict'

const chai = require('chai');
const expect = chai.expect;
const msgUtils = require('../src/utils/message_utils.js');
const BN = require('bn.js');

describe('gennodejsTests', () => {
  msgUtils.loadMessagePackage('std_msgs');
  msgUtils.loadMessagePackage('test_msgs');
  msgUtils.loadMessagePackage('actionlib_msgs');

  it('basic', (done) => {
    let stringMsg;
    const loadStrFunc = () => {
      stringMsg = msgUtils.getHandlerForMsgType('std_msgs/String');
    };
    expect(loadStrFunc).to.not.throw(/good function/);
    expect(stringMsg).to.be.a('function');
    expect(stringMsg).to.have.property('serialize');
    expect(stringMsg.serialize).to.be.a('function');
    expect(stringMsg).to.have.property('deserialize');
    expect(stringMsg.deserialize).to.be.a('function');
    expect(stringMsg).to.have.property('datatype');
    expect(stringMsg.datatype).to.be.a('function');
    expect(stringMsg).to.have.property('md5sum');
    expect(stringMsg.md5sum).to.be.a('function');
    expect(stringMsg).to.have.property('messageDefinition');
    expect(stringMsg.messageDefinition).to.be.a('function');

    done();
  });

  it('json or instance', (done) => {
    const msgData = 'chatter';
    const stdMsgString = msgUtils.getHandlerForMsgType('std_msgs/String');
    const msgInstance = new stdMsgString();
    msgInstance.data = msgData;

    let instanceBuffer = new Buffer(stdMsgString.getMessageSize(msgInstance));
    stdMsgString.serialize(msgInstance, instanceBuffer, 0);

    let jsonMsg = {data: msgData};
    let jsonBuffer = new Buffer(stdMsgString.getMessageSize(jsonMsg));
    stdMsgString.serialize(jsonMsg, jsonBuffer, 0);

    expect(instanceBuffer.equals(jsonBuffer)).to.be.true;

    done();
  });

  describe('builtins', () => {
    it('string', (done) => {
      const stdMsgString = msgUtils.getHandlerForMsgType('std_msgs/String');
      const msgData = 'chatter';
      const msgSize = 4 + msgData.length;
      // manually serialize string msg
      const fullMsg = new Buffer(msgSize);
      fullMsg.writeUInt32LE(msgData.length);
      fullMsg.write(msgData, 4);

      // auto serialize
      const msg = new stdMsgString();
      msg.data = msgData;

      const fullMsg2 = new Buffer(stdMsgString.getMessageSize(msg));
      stdMsgString.serialize(msg, fullMsg2, 0);

      // expect equality
      expect(fullMsg.equals(fullMsg2)).to.be.true;

      // deserialize msg buffer - should equal original msgData
      expect(stdMsgString.deserialize(fullMsg2, [0]).data).to.equal(msgData);

      expect((new stdMsgString()).data).to.equal('');

      done();
    });

    it('bool', (done) => {
      const data = true;

      const msgBuffer = new Buffer(1);
      msgBuffer.writeInt8(true);

      const msgBuffer2 =  new Buffer(1);
      const Bool = msgUtils.getHandlerForMsgType('std_msgs/Bool');
      Bool.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Bool.deserialize(msgBuffer2, [0]).data).to.equal(data);

      expect((new Bool()).data).to.equal(false);

      done();
    });

    it('int8', (done) => {
      const intData = -33;

      const msgBuffer = new Buffer(1);
      msgBuffer.writeInt8(intData);

      const msgBuffer2 =  new Buffer(1);
      const Int8 = msgUtils.getHandlerForMsgType('std_msgs/Int8');
      Int8.serialize({data: intData}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Int8.deserialize(msgBuffer2, [0]).data).to.equal(intData);

      expect((new Int8()).data).to.equal(0);

      done();
    });

    it('uint8', (done) => {
      const data = 32;

      const msgBuffer = new Buffer(1);
      msgBuffer.writeInt8(data);

      const msgBuffer2 =  new Buffer(1);
      const UInt8 = msgUtils.getHandlerForMsgType('std_msgs/UInt8');
      UInt8.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(UInt8.deserialize(msgBuffer2, [0]).data).to.equal(data);

      expect((new UInt8()).data).to.equal(0);

      done();
    });

    it('int16', (done) => {
      const intData = -3345;

      const msgBuffer = new Buffer(2);
      msgBuffer.writeInt16LE(intData);

      const msgBuffer2 =  new Buffer(2);
      const Int16 = msgUtils.getHandlerForMsgType('std_msgs/Int16');
      Int16.serialize({data: intData}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Int16.deserialize(msgBuffer2, [0]).data).to.equal(intData);

      expect((new Int16()).data).to.equal(0);

      done();
    });

    it('uint16', (done) => {
      const data = 65530;

      const msgBuffer = new Buffer(2);
      msgBuffer.writeUInt16LE(data);

      const msgBuffer2 =  new Buffer(2);
      const UInt16 = msgUtils.getHandlerForMsgType('std_msgs/UInt16');
      UInt16.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(UInt16.deserialize(msgBuffer2, [0]).data).to.equal(data);

      expect((new UInt16()).data).to.equal(0);

      done();
    });

    it('int32', (done) => {
      const intData = -3345;

      const msgBuffer = new Buffer(4);
      msgBuffer.writeInt32LE(intData);

      const msgBuffer2 =  new Buffer(4);
      const Int32 = msgUtils.getHandlerForMsgType('std_msgs/Int32');
      Int32.serialize({data: intData}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Int32.deserialize(msgBuffer2, [0]).data).to.equal(intData);

      expect((new Int32()).data).to.equal(0);

      done();
    });

    it('uint32', (done) => {
      const data = 65530;

      const msgBuffer = new Buffer(4);
      msgBuffer.writeUInt32LE(data);

      const msgBuffer2 =  new Buffer(4);
      const UInt32 = msgUtils.getHandlerForMsgType('std_msgs/UInt32');
      UInt32.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(UInt32.deserialize(msgBuffer2, [0]).data).to.equal(data);

      expect((new UInt32()).data).to.equal(0);

      done();
    });

    it('int64', (done) => {
      const intData = new BN('9223372036854765802');

      const msgBuffer = intData.toBuffer('le', 8);

      const msgBuffer2 =  new Buffer(8);
      const Int64 = msgUtils.getHandlerForMsgType('std_msgs/Int64');
      Int64.serialize({data: intData}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(intData.eq(Int64.deserialize(msgBuffer2, [0]).data)).to.be.true;

      expect((new Int64()).data).to.equal(0);

      done();
    });

    it('uint64', (done) => {
      const intData = new BN('9223372036854765802');

      const msgBuffer = intData.toBuffer('le', 8);

      const msgBuffer2 =  new Buffer(8);
      const UInt64 = msgUtils.getHandlerForMsgType('std_msgs/UInt64');
      UInt64.serialize({data: intData}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(intData.eq(UInt64.deserialize(msgBuffer2, [0]).data)).to.be.true;

      expect((new UInt64()).data).to.equal(0);

      done();
    });

    it('float32', (done) => {
      const data = -3345.123;

      const msgBuffer = new Buffer(4);
      msgBuffer.writeFloatLE(data);

      const msgBuffer2 =  new Buffer(4);
      const Float32 = msgUtils.getHandlerForMsgType('std_msgs/Float32');
      Float32.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Float32.deserialize(msgBuffer2, [0]).data).to.be.closeTo(data, 0.0005);

      expect((new Float32()).data).to.be.closeTo(0.0, 0.0005);

      done();
    });

    it('float64', (done) => {
      const data = -3345.123576;

      const msgBuffer = new Buffer(8);
      msgBuffer.writeDoubleLE(data);

      const msgBuffer2 =  new Buffer(8);
      const Float32 = msgUtils.getHandlerForMsgType('std_msgs/Float64');
      Float32.serialize({data: data}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      expect(Float32.deserialize(msgBuffer2, [0]).data).to.be.closeTo(data, 0.0000005);

      expect((new Float32()).data).to.be.closeTo(0.0, 0.0000005);

      done();
    });

    it('time', (done) => {
      const time = {secs: 0, nsecs: 0};

      const msgBuffer = new Buffer(8);
      msgBuffer.writeInt32LE(time.secs)
      msgBuffer.writeInt32LE(time.nsecs, 4);

      const msgBuffer2 =  new Buffer(8);
      const Time = msgUtils.getHandlerForMsgType('std_msgs/Time');
      Time.serialize({data: time}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      const deserializedTime = Time.deserialize(msgBuffer2, [0]).data;
      expect(deserializedTime.secs).to.equal(time.secs);
      expect(deserializedTime.nsecs).to.equal(time.nsecs);

      expect((new Time()).data).to.deep.equal({secs: 0, nsecs: 0});

      done();
    });

    it('duration', (done) => {
      const duration = {secs: 0, nsecs: 0};

      const msgBuffer = new Buffer(8);
      msgBuffer.writeInt32LE(duration.secs)
      msgBuffer.writeInt32LE(duration.nsecs, 4);

      const msgBuffer2 =  new Buffer(8);
      const Duration = msgUtils.getHandlerForMsgType('std_msgs/Duration');
      Duration.serialize({data: duration}, msgBuffer2, 0);

      expect(msgBuffer.equals(msgBuffer2)).to.be.true;
      const deserializedDuration = Duration.deserialize(msgBuffer2, [0]).data;
      expect(deserializedDuration.secs).to.equal(duration.secs);
      expect(deserializedDuration.nsecs).to.equal(duration.nsecs);

      expect((new Duration()).data).to.deep.equal({secs: 0, nsecs: 0});

      done();
    });
  });

  describe('complex_msgs', () => {
    it('messages and constants', (done) => {
      const BaseType = msgUtils.getHandlerForMsgType('test_msgs/BaseType');

      expect(BaseType).to.be.a('function');
      expect(BaseType).to.have.property('Constants');

      expect(BaseType.Constants.NUMERIC_CONSTANT_A).to.be.a('number');
      expect(BaseType.Constants.NUMERIC_CONSTANT_B).to.be.a('number');
      expect(BaseType.Constants.STRING_CONSTANT).to.be.a('string');

      expect(BaseType.Constants.NUMERIC_CONSTANT_A).to.equal(1);
      expect(BaseType.Constants.NUMERIC_CONSTANT_B).to.equal(2);
      expect(BaseType.Constants.STRING_CONSTANT).to.equal('hello');

      const baseType = new BaseType();
      expect(BaseType.getMessageSize(baseType)).to.equal(5);
      baseType.string_field = BaseType.Constants.STRING_CONSTANT;
      baseType.num_field = BaseType.Constants.NUMERIC_CONSTANT_A;
      let newlen = 5 + BaseType.Constants.STRING_CONSTANT.length;
      expect(BaseType.getMessageSize(baseType)).to.equal(newlen);

      const msgBuffer = new Buffer(BaseType.getMessageSize(baseType));
      BaseType.serialize(baseType, msgBuffer, 0);

      const deserializedMsg = BaseType.deserialize(msgBuffer, [0]);

      expect(deserializedMsg.string_field).to.equal(baseType.string_field);
      expect(deserializedMsg.num_field).to.equal(baseType.num_field);

      done();
    });

    it('check constant types', (done) => {
      const AllTypes = msgUtils.getHandlerForMsgType('test_msgs/AllTypes');

      expect(AllTypes).to.be.a('function');
      expect(AllTypes).to.have.property('Constants');

      expect(AllTypes.Constants.STRING_CONST).to.equal('1');
      expect(AllTypes.Constants.UINT8_CONST).to.equal(1);
      expect(AllTypes.Constants.UINT16_CONST).to.equal(1);
      expect(AllTypes.Constants.UINT32_CONST).to.equal(1);
      expect(AllTypes.Constants.UINT64_CONST).to.equal(1);
      expect(AllTypes.Constants.INT8_CONST).to.equal(1);
      expect(AllTypes.Constants.INT16_CONST).to.equal(1);
      expect(AllTypes.Constants.INT32_CONST).to.equal(1);
      expect(AllTypes.Constants.INT64_CONST).to.equal(1);
      expect(AllTypes.Constants.FLOAT32_CONST).to.equal(1);
      expect(AllTypes.Constants.FLOAT64_CONST).to.equal(1);
      expect(AllTypes.Constants.BYTE_CONST).to.equal(1);
      expect(AllTypes.Constants.CHAR_CONST).to.equal(1);
      expect(AllTypes.Constants.BOOL_CONST).to.equal(true);

      expect(AllTypes.md5sum()).to.equal('95bdfe2fab3bda07e308f9640f25904c');

      const allTypeMsg = new AllTypes();

      expect(allTypeMsg.string_field).to.equal('');
      expect(allTypeMsg.uint8_field).to.equal(0);
      expect(allTypeMsg.uint16_field).to.equal(0);
      expect(allTypeMsg.uint32_field).to.equal(0);
      expect(allTypeMsg.uint64_field).to.equal(0);
      expect(allTypeMsg.int8_field).to.equal(0);
      expect(allTypeMsg.int16_field).to.equal(0);
      expect(allTypeMsg.int32_field).to.equal(0);
      expect(allTypeMsg.int64_field).to.equal(0);
      expect(allTypeMsg.float32_field).to.equal(0);
      expect(allTypeMsg.float64_field).to.equal(0);
      expect(allTypeMsg.byte_field).to.equal(0);
      expect(allTypeMsg.char_field).to.equal(0);
      expect(allTypeMsg.bool_field).to.equal(false);
      expect(allTypeMsg.time_field).to.deep.equal({secs: 0, nsecs: 0});
      expect(allTypeMsg.duration_field).to.deep.equal({secs: 0, nsecs: 0});

      done();
    });

    it('initialization', () => {
      const BaseType = msgUtils.getHandlerForMsgType('test_msgs/BaseType');

      let baseType = new BaseType();
      expect(baseType.string_field).to.be.empty;
      expect(baseType.num_field).to.equal(0);

      const stringField = 'AnInTeReStInGVaLuE';
      const numField = 12345;

      baseType = new BaseType({
        string_field: stringField,
        num_field: numField
      });

      expect(baseType.string_field).to.equal(stringField);
      expect(baseType.num_field).to.equal(numField);

      const stringField2 = 'Woahh, they changed it';
      baseType.string_field = stringField2;

      baseType = new BaseType(baseType);
      expect(baseType.string_field).to.equal(stringField2);
      expect(baseType.num_field).to.equal(numField);

      const BaseTypeVariableLengthArray = msgUtils.getHandlerForMsgType('test_msgs/BaseTypeVariableLengthArray');
      let instance1 = new BaseTypeVariableLengthArray();

      expect(instance1.array_field).to.deep.equal([]);

      instance1.array_field.push(baseType);
      expect(instance1.array_field[0]).to.equal(baseType);

      let instance2 = new BaseTypeVariableLengthArray(instance1);
      expect(instance1.array_field).to.deep.equal(instance2.array_field);

      const BasicService = msgUtils.getHandlerForSrvType('test_msgs/BasicService');
      let basicServiceReq = new BasicService.Request();

      expect(basicServiceReq.data).to.equal('');
      expect(basicServiceReq.op).to.equal('');

      basicServiceReq = new BasicService.Request({data: stringField, op: stringField2});

      expect(basicServiceReq.data).to.equal(stringField);
      expect(basicServiceReq.op).to.equal(stringField2);

      let basicServiceResp = new BasicService.Response();

      expect(basicServiceResp.result).to.equal('');

      basicServiceResp = new BasicService.Response({result: stringField});

      expect(basicServiceResp.result).to.equal(stringField);

      // expect full message definition (including test_msgs/BaseType)
      expect(BaseTypeVariableLengthArray.messageDefinition().includes(BaseType.messageDefinition().trim())).to.be.true;
    });

    it('constant length arrays', (done) => {
      const CLA = msgUtils.getHandlerForMsgType('test_msgs/ConstantLengthArray');

      const cla = new CLA();
      const claArrLen = 10;
      expect(cla.array_field).to.be.a('Array');
      expect(cla.array_field.length).to.equal(claArrLen);
      expect(CLA.getMessageSize(cla)).to.equal(claArrLen);

      cla.array_field.forEach((item) => {
        expect(item).to.be.a('number');
        expect(item).to.equal(0);
      });

      const msgBuffer = new Buffer(CLA.getMessageSize(cla));
      CLA.serialize(cla, msgBuffer, 0);
      expect(msgBuffer.length).to.equal(claArrLen);

      const deserializedMsg = CLA.deserialize(msgBuffer, [0]);
      expect(deserializedMsg.array_field.length).to.equal(claArrLen);

      const BTCLA = msgUtils.getHandlerForMsgType('test_msgs/BaseTypeConstantLengthArray');
      const BaseType = msgUtils.getHandlerForMsgType('test_msgs/BaseType');

      const btcla = new BTCLA();
      const btclaArrLen = 5;
      expect(btcla.array_field).to.be.a('Array');
      expect(btcla.array_field.length).to.equal(btclaArrLen);
      expect(BTCLA.getMessageSize(btcla)).to.equal(25);

      btcla.array_field.forEach((item) => {
        expect(item).to.be.an.instanceof(BaseType);
      });

      const msgBuffer2 = new Buffer(BTCLA.getMessageSize(btcla));
      BTCLA.serialize(btcla, msgBuffer2, 0);
      expect(msgBuffer2.length).to.equal(25);

      const deserializedMsg2 = BTCLA.deserialize(msgBuffer2, [0]);
      expect(deserializedMsg2.array_field.length).to.equal(btclaArrLen);
      deserializedMsg2.array_field.forEach((item) => {
        expect(item).to.be.an.instanceof(BaseType);
      });

      done();
    });

    it('variable length arrays', (done) => {
      const VLA = msgUtils.getHandlerForMsgType('test_msgs/VariableLengthArray');

      const vla = new VLA();
      expect(vla.array_field).to.be.a('Array');
      expect(vla.array_field.length).to.equal(0);
      expect(VLA.getMessageSize(vla)).to.equal(4);

      const msgBuffer = new Buffer(4);
      VLA.serialize(vla, msgBuffer, 0);

      const val = 12;
      const arrLen = 7;
      vla.array_field = new Array(arrLen).fill(val);
      vla.array_field.forEach((item) => {
        expect(item).to.be.a('number');
        expect(item).to.equal(val);
      });

      expect(VLA.getMessageSize(vla)).to.equal(arrLen + 4);
      const msgBuffer2 = new Buffer(VLA.getMessageSize(vla));
      VLA.serialize(vla, msgBuffer2, 0);

      const deserializedMsg = VLA.deserialize(msgBuffer2, [0]);
      expect(deserializedMsg.array_field.length).to.equal(arrLen);
      deserializedMsg.array_field.forEach((item) => {
        expect(item).to.be.a('number');
        expect(item).to.equal(val);
      });

      const BTVLA = msgUtils.getHandlerForMsgType('test_msgs/BaseTypeVariableLengthArray');
      const BaseType = msgUtils.getHandlerForMsgType('test_msgs/BaseType');

      const btvla = new BTVLA();
      expect(btvla.array_field).to.be.a('Array');
      expect(btvla.array_field.length).to.equal(0);
      expect(BTVLA.getMessageSize(btvla)).to.equal(4);

      const msgBuffer3 = new Buffer(VLA.getMessageSize(btvla));
      VLA.serialize(btvla, msgBuffer3, 0);
      expect(msgBuffer3.length).to.equal(4);

      const arrLen2 = 4;
      btvla.array_field = new Array(arrLen2).fill(new BaseType());

      expect(BTVLA.getMessageSize(btvla)).to.equal(24);

      const msgBuffer4 = new Buffer(BTVLA.getMessageSize(btvla));
      BTVLA.serialize(btvla, msgBuffer4, 0);

      const deserializedMsg2 = BTVLA.deserialize(msgBuffer4, [0]);
      expect(deserializedMsg2.array_field.length).to.equal(arrLen2);
      deserializedMsg2.array_field.forEach((item) => {
        expect(item).to.be.an.instanceof(BaseType);
      });

      done();
    });

    it('services and constants', (done) => {
      const BasicService = msgUtils.getHandlerForSrvType('test_msgs/BasicService');

      expect(BasicService).to.have.property('Request');
      expect(BasicService).to.have.property('Response');
      expect(BasicService).to.have.property('md5sum');
      expect(BasicService.md5sum).to.be.a('function');

      expect(BasicService.md5sum()).to.not.equal(BasicService.Request.md5sum());
      expect(BasicService.md5sum()).to.not.equal(BasicService.Response.md5sum());

      const BSRequest = BasicService.Request;
      expect(BSRequest.Constants.OP_REVERSE).to.equal('reverse');
      expect(BSRequest.Constants.OP_LEFT_PAD).to.equal('left_pad');
      const bsRequest = new BSRequest();
      expect(bsRequest.data).to.be.a('string');
      expect(bsRequest.op).to.be.a('string');
      expect(BSRequest.getMessageSize(bsRequest)).to.equal(8);

      const dataField = 'JUNK';
      bsRequest.data = dataField;
      bsRequest.op = BSRequest.Constants.OP_LEFT_PAD;
      let newlen = 8 + bsRequest.data.length + bsRequest.op.length;
      expect(BSRequest.getMessageSize(bsRequest)).to.equal(newlen);

      const msgBuffer = new Buffer(BSRequest.getMessageSize(bsRequest));
      BSRequest.serialize(bsRequest, msgBuffer, 0);

      const deserializedRequest = BSRequest.deserialize(msgBuffer, [0]);
      expect(deserializedRequest).to.be.an.instanceof(BSRequest);
      expect(deserializedRequest.data).to.equal(dataField);
      expect(deserializedRequest.op).to.equal(BSRequest.Constants.OP_LEFT_PAD);

      const BSResponse = BasicService.Response;
      expect(BSResponse.Constants.RES_NULL).to.equal('null');
      const bsResponse = new BSResponse();
      expect(bsResponse.result).to.be.a('string');

      bsResponse.result = BSResponse.Constants.RES_NULL;
      const msgBuffer2 = new Buffer(BSResponse.getMessageSize(bsResponse));
      BSResponse.serialize(bsResponse, msgBuffer2, 0);
      expect(msgBuffer2.length).to.equal(8);

      const deserializedResponse = BSResponse.deserialize(msgBuffer2, [0]);
      expect(deserializedResponse).to.be.an.instanceof(BSResponse);
      expect(deserializedResponse.result).to.equal(BSResponse.Constants.RES_NULL);

      done();
    });

    it('service depending on this package', (done) => {
      const TestService = msgUtils.getHandlerForSrvType('test_msgs/TestService');
      const BaseType = msgUtils.getHandlerForMsgType('test_msgs/BaseType');

      const TSRequest = TestService.Request;
      const TSResponse = TestService.Response;
      expect(TSRequest).to.be.a('function');
      expect(TSResponse).to.be.a('function');

      const tsRequest = new TSRequest();
      expect(TSRequest.getMessageSize(tsRequest)).to.equal(5);
      expect(tsRequest.input).to.be.an.instanceof(BaseType);
      tsRequest.input.string_field = BaseType.Constants.STRING_CONSTANT;

      let newlen = 5 + BaseType.Constants.STRING_CONSTANT.length;
      expect(TSRequest.getMessageSize(tsRequest)).to.equal(newlen);
      const msgBuffer = new Buffer(TSRequest.getMessageSize(tsRequest));
      TSRequest.serialize(tsRequest, msgBuffer, 0);
      expect(msgBuffer.length).to.equal(10);

      const deserializedRequest = TSRequest.deserialize(msgBuffer, [0]);
      expect(deserializedRequest).to.be.an.instanceof(TSRequest);
      expect(deserializedRequest.input).to.be.an.instanceof(BaseType);

      const tsResponse = new TSResponse();
      expect(tsResponse).to.be.empty;
      expect(TSResponse.getMessageSize(tsResponse)).to.equal(0);
      expect(TSResponse.getMessageSize()).to.equal(0);

      const msgBuffer2 = new Buffer(TSResponse.getMessageSize(tsResponse));
      TSResponse.serialize(tsResponse, msgBuffer2, 0);
      expect(msgBuffer2.length).to.equal(0);

      const deserializedRequest2 = TSResponse.deserialize(msgBuffer2, [0]);
      expect(deserializedRequest2).to.be.an.instanceof(TSResponse);
      expect(deserializedRequest2).to.be.empty;

      done();
    });

    it('message depending on another package', (done) => {
      const StdMsg = msgUtils.getHandlerForMsgType('test_msgs/StdMsg');
      const Header = msgUtils.getHandlerForMsgType('std_msgs/Header');

      const frameId = 'base';
      const time = {secs: 100, nsecs: 1000};
      const seq = 123;

      const header = new Header();
      expect(header.frame_id).to.be.a('string');
      header.seq = seq;
      header.stamp = time;
      header.frame_id = frameId;

      const stdMsg = new StdMsg();
      expect(StdMsg.getMessageSize(stdMsg)).to.equal(24);
      expect(stdMsg.header).to.be.an.instanceof(Header);
      stdMsg.header = header;
      stdMsg.time_field = time;

      let newlen = 24 + stdMsg.header.frame_id.length;
      expect(StdMsg.getMessageSize(stdMsg)).to.equal(newlen);
      const msgBuffer = new Buffer(StdMsg.getMessageSize(stdMsg));
      StdMsg.serialize(stdMsg, msgBuffer, 0);

      const deserializedMsg = StdMsg.deserialize(msgBuffer, [0]);
      expect(deserializedMsg.header.seq).to.equal(seq);
      expect(deserializedMsg.header.stamp.secs).to.equal(time.secs);
      expect(deserializedMsg.header.stamp.nsecs).to.equal(time.nsecs);
      expect(deserializedMsg.header.frame_id).to.equal(frameId);
      expect(deserializedMsg.time_field.secs).to.equal(time.secs);
      expect(deserializedMsg.time_field.nsecs).to.equal(time.nsecs);

      // check message definition is full message definition (including message it depends on)
      expect(StdMsg.messageDefinition().includes(Header.messageDefinition().trim())).to.be.true;

      done();
    });

    it('service depending on another package', (done) => {
      const HeaderService = msgUtils.getHandlerForSrvType('test_msgs/HeaderService');
      const Header = msgUtils.getHandlerForMsgType('std_msgs/Header');

      const HRequest = HeaderService.Request;
      const HResponse = HeaderService.Response;
      expect(HRequest).to.be.a('function');
      expect(HResponse).to.be.a('function');

      const hRequest = new HRequest();
      expect(hRequest).to.be.empty;
      expect(HRequest.getMessageSize(hRequest)).to.equal(0);
      expect(HRequest.getMessageSize()).to.equal(0);

      const msgBuffer = new Buffer(HRequest.getMessageSize(hRequest));
      HRequest.serialize(hRequest, msgBuffer, 0);
      expect(msgBuffer.length).to.equal(0);

      const deserializedRequest = HRequest.deserialize(msgBuffer, [0]);
      expect(deserializedRequest).to.be.an.instanceof(HRequest);
      expect(hRequest).to.be.empty;

      const hResponse =  new HResponse();
      expect(HResponse.getMessageSize(hResponse)).to.equal(16);
      expect(hResponse.header_response).to.be.an.instanceof(Header);
      const seq = 123;
      const frameId = 'base';
      hResponse.header_response.seq = seq;
      hResponse.header_response.frame_id = frameId;

      let newlen = 16 + frameId.length;
      expect(HResponse.getMessageSize(hResponse)).to.equal(newlen);

      const msgBuffer2 = new Buffer(HResponse.getMessageSize(hResponse));
      HResponse.serialize(hResponse, msgBuffer2, 0);
      expect(msgBuffer2.length).to.equal(20);

      const deserializedRequest2 = HResponse.deserialize(msgBuffer2, [0]);
      expect(deserializedRequest2).to.be.an.instanceof(HResponse);
      expect(deserializedRequest2.header_response).to.be.an.instanceof(Header);
      expect(deserializedRequest2.header_response.seq).to.equal(seq);
      expect(deserializedRequest2.header_response.frame_id).to.equal(frameId);
      expect(deserializedRequest2.header_response.stamp.secs).to.equal(0);
      expect(deserializedRequest2.header_response.stamp.nsecs).to.equal(0);

      done();
    });

    it('Service with improper divider', (done) => {
      // correct message divider is '---'
      // we're allowing any line starting with '---' though, parroting genmsg
      const HeaderService = msgUtils.getHandlerForSrvType('test_msgs/NonSpecServiceDivider');

      const requestMsg = HeaderService.Request.messageDefinition().trim();
      expect(requestMsg).to.equal('string data');

      const responseMsg = HeaderService.Response.messageDefinition().trim();
      expect(responseMsg).to.equal('uint8 response');

      done();
    });
  });

  describe('actions', () => {
    it('Action Messages Exist', () => {
      expect(msgUtils.getHandlerForMsgType('actionlib_msgs/GoalID')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('actionlib_msgs/GoalStatus')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('actionlib_msgs/GoalStatusArray')).to.not.throw;

      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionActionResult')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionActionFeedback')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionActionGoal')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionResult')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionAction')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionFeedback')).to.not.throw;
      expect(msgUtils.getHandlerForMsgType('test_msgs/TestActionGoal')).to.not.throw;
    });

    it('Message Definition', () => {
      const TestActionActionGoal = msgUtils.getHandlerForMsgType('test_msgs/TestActionActionGoal');
      const TestActionActionFeedback = msgUtils.getHandlerForMsgType('test_msgs/TestActionActionFeedback');
      const TestActionActionResult = msgUtils.getHandlerForMsgType('test_msgs/TestActionActionResult');
      const TestActionGoal = msgUtils.getHandlerForMsgType('test_msgs/TestActionGoal');
      const TestActionFeedback = msgUtils.getHandlerForMsgType('test_msgs/TestActionFeedback');
      const TestActionResult = msgUtils.getHandlerForMsgType('test_msgs/TestActionResult');

      const actionGoalMsgDefinition = TestActionActionGoal.messageDefinition().trim();
      const actionFeedbackMsgDefinition = TestActionActionFeedback.messageDefinition().trim();
      const actionResultMsgDefinition = TestActionActionResult.messageDefinition().trim();
      const goalMsgDefinition = TestActionGoal.messageDefinition().trim();
      const feedbackMsgDefinition = TestActionFeedback.messageDefinition().trim();
      const resultMsgDefinition = TestActionResult.messageDefinition().trim();

      expect(goalMsgDefinition.includes('---')).to.be.false;
      expect(goalMsgDefinition.includes(feedbackMsgDefinition)).to.be.false;
      expect(goalMsgDefinition.includes(resultMsgDefinition)).to.be.false;

      expect(feedbackMsgDefinition.includes('---')).to.be.false;
      expect(feedbackMsgDefinition.includes(goalMsgDefinition)).to.be.false;
      expect(feedbackMsgDefinition.includes(resultMsgDefinition)).to.be.false;

      expect(resultMsgDefinition.includes('---')).to.be.false;
      expect(resultMsgDefinition.includes(feedbackMsgDefinition)).to.be.false;
      expect(resultMsgDefinition.includes(goalMsgDefinition)).to.be.false;

      expect(actionGoalMsgDefinition.includes('---')).to.be.false;
      expect(actionGoalMsgDefinition.includes(goalMsgDefinition)).to.be.true;
      expect(actionGoalMsgDefinition.includes(feedbackMsgDefinition)).to.be.false;
      expect(actionGoalMsgDefinition.includes(resultMsgDefinition)).to.be.false;

      expect(actionFeedbackMsgDefinition.includes('---')).to.be.false;
      expect(actionFeedbackMsgDefinition.includes(goalMsgDefinition)).to.be.false;
      expect(actionFeedbackMsgDefinition.includes(feedbackMsgDefinition)).to.be.true;
      expect(actionFeedbackMsgDefinition.includes(resultMsgDefinition)).to.be.false;

      expect(actionResultMsgDefinition.includes('---')).to.be.false;
      expect(actionResultMsgDefinition.includes(goalMsgDefinition)).to.be.false;
      expect(actionResultMsgDefinition.includes(feedbackMsgDefinition)).to.be.false;
      expect(actionResultMsgDefinition.includes(resultMsgDefinition)).to.be.true;
    });
  });

  describe('Message Resolving', () => {
    // Test partially filling in a message and then being able to serialize it

    const test_msgs = msgUtils.getPackage('test_msgs');
    const Header = msgUtils.getHandlerForMsgType('std_msgs/Header');
    const BaseType = test_msgs.msg.BaseType;
    const StdMsg = test_msgs.msg.StdMsg;

    // just assuming this will be enough
    const buf = Buffer.alloc(500);

    it('Base Type', () => {
      let msg = {num_field: 1001};
      let fullMsg = BaseType.Resolve(msg);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(1001);

      expect(() => BaseType.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      msg = {num_field: undefined};
      fullMsg = BaseType.Resolve(msg);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      msg = undefined;
      fullMsg = BaseType.Resolve(msg);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      fullMsg = BaseType.Resolve(null);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      fullMsg = BaseType.Resolve('hi');

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      fullMsg = BaseType.Resolve(false);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      fullMsg = BaseType.Resolve(true);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;

      fullMsg = BaseType.Resolve(1);

      expect(fullMsg.string_field).to.equal('');
      expect(fullMsg.num_field).to.equal(0);

      expect(() => BaseType.serialize(fullMsg, buf, 0)).to.not.throw;
    });

    it('Nested Type', () => {
      let msg = {header: {seq: 102}};
      let fullMsg = StdMsg.Resolve(msg);

      expect(fullMsg.header instanceof Header).to.be.true;
      expect(() => StdMsg.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => StdMsg.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {header: undefined};
      fullMsg = StdMsg.Resolve(msg);

      expect(fullMsg.header instanceof Header).to.be.true;
      expect(() => StdMsg.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => StdMsg.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = undefined;
      fullMsg = StdMsg.Resolve(msg);

      expect(fullMsg.header instanceof Header).to.be.true;
      expect(() => StdMsg.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => StdMsg.serialize(fullMsg, buf, 0)).to.not.throw(Error);
    });

    it('Simple Constant Length Array', () => {
      const ConstantLengthArray = test_msgs.msg.ConstantLengthArray;

      let msg = {array_field: [1,2]};
      let fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.throw(Error);

      msg = {array_field: undefined};
      fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = undefined;
      fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);
    });

    it('Simple Variable Length Array', () => {
      const VariableLengthArray = test_msgs.msg.VariableLengthArray;

      let msg = {};
      let fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: undefined};
      fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: [1,2]};
      fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.not.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);
    });

    it('Complex Constant Length Array', () => {
      const ConstantLengthArray = test_msgs.msg.BaseTypeConstantLengthArray;

      // BaseTypeConstantLengthArray has an array_field field of length 5

      let msg = {array_field: [new BaseType(), 2]};
      let fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: undefined};
      fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: [new BaseType(), 2, 5, 10, 12, 15, 20]};
      fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = 'hi';
      fullMsg = ConstantLengthArray.Resolve(msg);

      expect(() => ConstantLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => ConstantLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);
    });

    it('Complex Variable Length Array', () => {
      const VariableLengthArray = test_msgs.msg.BaseTypeVariableLengthArray;

      let msg = {};
      let fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: undefined};
      fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: [1,2]};
      fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);

      msg = {array_field: []};
      fullMsg = VariableLengthArray.Resolve(msg);

      expect(() => VariableLengthArray.serialize(msg, buf, 0)).to.not.throw(Error);
      expect(() => VariableLengthArray.serialize(fullMsg, buf, 0)).to.not.throw(Error);
    });
  });
});
