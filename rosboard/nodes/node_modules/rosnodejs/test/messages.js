'use strict';

const chai = require('chai');
const expect = chai.expect;
const rosnodejs = require('../src/index.js');

describe('messages', function () {

  it('real messages', function() {
    expect(rosnodejs.require('geometry_msgs')).to.not.be.empty;
    expect(rosnodejs.require('geometry_msgs').msg).to.not.be.empty;
    expect(rosnodejs.require('geometry_msgs').msg).to.have.keys(
      'Accel','AccelStamped','AccelWithCovariance','AccelWithCovarianceStamped',
      'Inertia','InertiaStamped','Point','Point32','PointStamped',
      'Polygon','PolygonStamped','Pose','Pose2D','PoseArray','PoseStamped',
      'PoseWithCovariance','PoseWithCovarianceStamped','Quaternion',
      'QuaternionStamped','Transform','TransformStamped','Twist','TwistStamped',
      'TwistWithCovariance','TwistWithCovarianceStamped','Vector3',
      'Vector3Stamped','Wrench','WrenchStamped'
    );
    expect(rosnodejs.require('geometry_msgs').srv).to.be.empty;

    expect(rosnodejs.require('std_msgs')).to.not.be.empty;
    expect(rosnodejs.require('std_msgs').msg).to.not.be.empty;
    expect(rosnodejs.require('std_msgs').msg).to.have.keys(
      'Bool','Duration','Float64MultiArray','Int32MultiArray',
      'MultiArrayDimension','UInt16MultiArray','UInt8','Byte','Empty','Header',
      'Int64','MultiArrayLayout','UInt32','UInt8MultiArray','ByteMultiArray',
      'Float32','Int16','Int64MultiArray','String','UInt32MultiArray','Char',
      'Float32MultiArray','Int16MultiArray','Int8','Time','UInt64','ColorRGBA',
      'Float64','Int32','Int8MultiArray','UInt16','UInt64MultiArray'
    )
    expect(rosnodejs.require('std_msgs').srv).to.be.empty;

    expect(rosnodejs.require('std_srvs')).to.not.be.empty;
    expect(rosnodejs.require('std_srvs').msg).to.be.empty;
    expect(rosnodejs.require('std_srvs').srv).to.not.be.empty;
    expect(rosnodejs.require('std_srvs').srv).to.have.keys(
      'Empty','SetBool','Trigger'
    );

    expect(rosnodejs.require('test_msgs')).to.not.be.empty;
    expect(rosnodejs.require('test_msgs').msg).to.not.be.empty;
    expect(rosnodejs.require('test_msgs').msg).to.have.keys(
      'AllTypes','BaseTypeConstantLengthArray','ConstantLengthArray',
      'VariableLengthArray','BaseType','BaseTypeVariableLengthArray','StdMsg',
      "TestActionAction","TestActionActionFeedback","TestActionActionGoal",
      "TestActionActionResult","TestActionFeedback","TestActionGoal",
      "TestActionResult"
    );
    expect(rosnodejs.require('test_msgs').srv).to.not.be.empty;
    expect(rosnodejs.require('test_msgs').srv).to.have.keys(
      'BasicService','HeaderService','NonSpecServiceDivider','TestService'
    );
  });

  it('non-existant messages', function() {
    const nonExistantPkg = 'made_up_message_package';
    expect(rosnodejs.require.bind(rosnodejs, nonExistantPkg)).to.throw(
      Error, `Unable to find message package ${nonExistantPkg} from CMAKE_PREFIX_PATH`
    );
  });
});
