var socket = io.of('/ros');

class Subscriber {
  constructor(topicName, topicType, callback) {
    let that = this;
    socket.emit('subscribe', topicName, topicType, callback,
        (subscriberId) => { that.subscriberId = subscriberId; });
  }
}
