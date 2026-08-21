class WebSocketV1Transport {
    constructor({path, onOpen, onClose, onMsg, onTopics, onServices, onServiceResponse, onSystem}) {
      this.path = path;
      this.onOpen = onOpen ? onOpen.bind(this) : null;
      this.onClose = onClose ? onClose.bind(this) : null;
      this.onMsg = onMsg ? onMsg.bind(this) : null;
      this.onTopics = onTopics ? onTopics.bind(this) : null;
      this.onServices = onServices ? onServices.bind(this) : null;
      this.onServiceResponse = onServiceResponse ? onServiceResponse.bind(this) : null;
      this.onSystem = onSystem ? onSystem.bind(this) : null;
      this.ws = null;
    }
  
    connect() {
      var protocolPrefix = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
      let abspath = protocolPrefix + '//' + location.host + this.path;
  
      let that = this;
  
      this.ws = new WebSocket(abspath);
  
      this.ws.onopen = function(){
        console.log("connected");
        if(that.onOpen) that.onOpen(that);
      }
      
      this.ws.onclose = function(){
        console.log("disconnected");
        if(that.onClose) that.onClose(that);
      }
  
      this.ws.onmessage = function(wsmsg) {
        let data = [];
        try {
          // try fast native parser
          data = JSON.parse(wsmsg.data);
        } catch(e) {
          // Python may have included Infinity, -Infinity, NaN
          // fall back to a JSON5 parsers which will deal with these well but is almost 50X slower in Chrome
          data = JSON5.parse(wsmsg.data);
        }

        let wsMsgType = data[0];
  
        if(wsMsgType === WebSocketV1Transport.MSG_PING) {
          this.send(JSON.stringify([WebSocketV1Transport.MSG_PONG, {
            [WebSocketV1Transport.PONG_SEQ]: data[1][WebSocketV1Transport.PING_SEQ],
            [WebSocketV1Transport.PONG_TIME]: Date.now(),
          }]));
        }
        else if(wsMsgType === WebSocketV1Transport.MSG_MSG && that.onMsg) that.onMsg(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_TOPICS && that.onTopics) that.onTopics(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_SERVICES && that.onServices) that.onServices(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_SERVICE_RESPONSE && that.onServiceResponse) that.onServiceResponse(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_SYSTEM && that.onSystem) that.onSystem(data[1]);
        else console.log("received unknown message: " + wsmsg.data);
      }
    }
  
    isConnected() {
      return (this.ws && this.ws.readyState === this.ws.OPEN);
    }
  
    subscribe({topicName, maxUpdateRate = 24.0}) {
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_SUB, {topicName: topicName, maxUpdateRate: maxUpdateRate}]));
    }

    unsubscribe({topicName}) {
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_UNSUB, {topicName: topicName}]));
    }

    callService({serviceName, serviceType, request, requestId}) {
      console.log('WebSocketV1Transport.callService called with:', {
        serviceName, serviceType, request, requestId
      });
      
      if (!this.ws || this.ws.readyState !== this.ws.OPEN) {
        console.error('WebSocket is not connected');
        return;
      }
      
      const message = [WebSocketV1Transport.MSG_SERVICE_CALL, {
        serviceName: serviceName, 
        serviceType: serviceType, 
        request: request,
        requestId: requestId
      }];
      
      console.log('Sending service call message:', JSON.stringify(message));
      this.ws.send(JSON.stringify(message));
    }
  }
  
  WebSocketV1Transport.MSG_PING = "p";
  WebSocketV1Transport.MSG_PONG = "q";
  WebSocketV1Transport.MSG_MSG = "m";
  WebSocketV1Transport.MSG_TOPICS = "t";
  WebSocketV1Transport.MSG_SERVICES = "v";
  WebSocketV1Transport.MSG_SUB = "s";
  WebSocketV1Transport.MSG_SYSTEM = "y";
  WebSocketV1Transport.MSG_UNSUB = "u";
  WebSocketV1Transport.MSG_SERVICE_CALL = "c";
  WebSocketV1Transport.MSG_SERVICE_RESPONSE = "r";

  WebSocketV1Transport.PING_SEQ= "s";
  WebSocketV1Transport.PONG_SEQ = "s";
  WebSocketV1Transport.PONG_TIME = "t";