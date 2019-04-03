# rosnodejs [![Build Status](https://travis-ci.org/RethinkRobotics-opensource/rosnodejs.svg)](https://travis-ci.org/RethinkRobotics-opensource/rosnodejs)

## Install
`npm install rosnodejs`

## Start a node
```
const rosnodejs = require('rosnodejs');
rosnodejs.initNode('/my_node')
.then(() => {
  // do stuff
});
```

## Publish/Subscribe
```
const nh = rosnodejs.nh;
const sub = nh.subscribe('/chatter', 'std_msgs/String', (msg) => {
  console.log('Got msg on chatter: %j', msg);
});

const pub = nh.advertise('/chatter', 'std_msgs/String');
pub.publish({ data: "hi" });
```

## Services
```
const service = nh.advertiseService('/add_two_ints', 'beginner_tutorials/AddTwoInts', (req, resp) => {
  res.sum = req.a + req.b;
  return true;
});

const client = nh.serviceClient('/add_two_ints', 'beginner_tutorials/AddTwoInts');
client.call({a: 1, b: 2});
```

## Params
```
nh.setParam('val', 2);
nh.getParam('val')
.then((val) => {
  // do stuff
});
```
## Generating Messages

Messages can be generated in a number of ways depending on the versions of ROS and Node.js you're using.
- catkin - works in ROS Kinetic and later for Node.js v6+.
```
$ catkin_make
OR
$ catkin build
```
- `loadAllPackages()` - One-time "build" call available through `rosnodejs` for versions of ROS before Kinetic and Node.js v6+. Should be called separately and in advance of processes attempting to use messages.
```
const rosnodejs = require('rosnodejs');
rosnodejs.loadAllPackages();
```
- On the fly - all versions of ROS and Node.js 4.5+. When generating on the fly, messages can not be required until the node has initialized.
```
const rosnodejs = require('rosnodejs');
await rosnodejs.initNode('my_node', { onTheFly: true })
const stdMsgs = rosnodejs.require('std_msgs');
```

| |Pre-Kinetic|Kinetic & Later|
|:---:|:---:|:---:|
|Node.js  >= v6|`loadAllPackages()`, on the fly|catkin, `loadAllPackages()`, on the fly|
|Node.js < v6|on the fly|on the fly|

## Using Messages
```
const sensorMsgs = rosnodejs.require('sensor_msgs');

const image = new sensorMsgs.msg.Image();
const temperature = new sensorMsgs.msg.Temperature({ temperature: 32 });

const SetCameraInfo = sensorMsgs.srv.SetCameraInfo;
const setRequest = new SetCameraInfo.Request();

// messages can be used when advertising/subscribing
const StringMsg = rosnodejs.require('std_msgs').msg.String;
const sub = nh.subscribe('/chatter', StringMsg, (msg) => { ... });
const pub = nh.advertise('/chatter', StringMsg);

const AddTwoInts = rosnodejs.require('beginner_tutorials').srv.AddTwoInts;
const service = nh.advertiseService('/add_two_ints', AddTwoInts, (req, resp) => { ... });
const client = nh.serviceClient('/add_two_ints', AddTwoInts);
```
## Actions (Experimental)
```
const nh = rosnodejs.nh;
const as = new rosnodejs.ActionServer({
  nh,
  type: 'turtle_actionlib/Shape',
  actionServer: '/turtle_shape'
});

as.on('goal', function (goal) {
  goal.setAccepted();
});

as.start();

const ac = new rosnodejs.ActionClient({
  nh,
  type: 'turtle_actionlib/Shape',
  actionServer:'/turtle_shape'
});

ac.sendGoal({edges: 3, radius: 1});
```
## Run the turtlesim example

Start:

```
roscore
rosrun turtlesim turtlesim_node
rosrun turtle_actionlib shape_server
```

Then run
```
node src/examples/turtle.js
```

or, if you are running an older version of node:

```
npm run compile
node dist/examples/turtle.js
```

## Catkin-Like
Checkout [`rosnodejs_examples`](https://github.com/RethinkRobotics-opensource/rosnodejs_examples) for a more catkin-inspired take on using `rosnodejs`.

## Inspired By
`rosnodejs` was inspired by other work that you can learn more about here
- [roscpp & rospy](https://github.com/ros/ros_comm)
- [Robots as web services](http://ieeexplore.ieee.org/document/5980464/?tp=&arnumber=5980464&url=http:%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D5980464)
- [ROSBridge](https://github.com/RobotWebTools/rosbridge_suite)
- [roslibjs](https://github.com/RobotWebTools/roslibjs)

