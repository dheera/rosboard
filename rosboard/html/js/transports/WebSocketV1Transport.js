class WebSocketV1Transport {
    constructor({path, onOpen, onClose, onRosMsg, onTopics}) {
      this.path = path;
      this.onOpen = onOpen ? onOpen.bind(this) : null;
      this.onClose = onClose ? onClose.bind(this) : null;
      this.onRosMsg = onRosMsg ? onRosMsg.bind(this) : null;
      this.onTopics = onTopics ? onTopics.bind(this) : null;
      this.ws = null;
    }
  
    connect() {
      var protocolPrefix = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
      let abspath = protocolPrefix + '//' + location.host + this.path;
  
      let that = this;
  
      this.ws = new WebSocket(abspath);
  
      this.ws.onopen = function(){
        console.log("connected");
        if(that.onopen) that.onOpen(that);
      }
      
      this.ws.onclose = function(){
        console.log("disconnected");
        if(that.onclose) that.onClose(that);
      }
  
      this.ws.onmessage = function(wsmsg) {
        let data = JSON.parse(wsmsg.data);
        let wsMsgType = data[0];
  
        if(wsMsgType === "ping") this.send(JSON.stringify(["pong", Date.now()]));
        else if(wsMsgType === "ros_msg" && that.onRosMsg) that.onRosMsg(data[1]);
        else if(wsMsgType === "topics" && that.onTopics) that.onTopics(data[1]);
        else console.log("received unknown message: " + wsmsg);
      }
    }
  
    isConnected() {
      return (this.ws && this.ws.readyState === this.ws.OPEN);
    }
  
    subscribe(topic_name) {
      this.ws.send(JSON.stringify(["sub", topic_name]));
    }
  }
  