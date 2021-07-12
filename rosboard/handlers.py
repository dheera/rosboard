import json
import time
import tornado
import tornado.web
import tornado.websocket
import traceback
import uuid

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class ROSBoardSocketHandler(tornado.websocket.WebSocketHandler):
    sockets = set()
    cache = []
    cache_size = 200

    def initialize(self, node):
        self.node = node

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        self.id = uuid.uuid4()
        self.latency = 0
        self.clock_diff = 0
        self.last_ping_time = 0
        self.last_pong_time = 0
        self.update_intervals_by_topic = {}
        self.last_data_times_by_topic = {}
        ROSBoardSocketHandler.sockets.add(self)

    def on_close(self):
        ROSBoardSocketHandler.sockets.remove(self)
        for topic_name in self.node.subscriptions:
            if self.id in self.node.subscriptions[topic_name]:
                self.node.subscriptions[topic_name].remove(self.id)

    @classmethod
    def update_cache(cls, chat):
        cls.cache.append(chat)
        if len(cls.cache) > cls.cache_size:
            cls.cache = cls.cache[-cls.cache_size :]


    @classmethod
    def send_pings(cls):
        for socket in cls.sockets:
            try:
                socket.last_ping_time = time.time() * 1000
                socket.write_message(json.dumps([ROSBoardSocketHandler.MSG_PING]))
            except Exception as e:
                print("Error sending message: %s", str(e))

    @classmethod
    def broadcast(cls, message):
        for socket in cls.sockets:
            try:
                if message[0] == ROSBoardSocketHandler.MSG_TOPICS:
                    socket.write_message(json.dumps(message))
                elif message[0] == ROSBoardSocketHandler.MSG_MSG:
                    topic_name = message[1]["_topic_name"]
                    if topic_name not in socket.node.subscriptions:
                        continue
                    if socket.id not in socket.node.subscriptions[topic_name]:
                        continue

                    t = time.time()
                    if t - socket.last_data_times_by_topic.get(topic_name, 0.0) < \
                            socket.update_intervals_by_topic.get(topic_name) - 2e-4:
                        continue

                    ros_msg_dict = message[1]

                    socket.write_message(json.dumps([ROSBoardSocketHandler.MSG_MSG, ros_msg_dict]))
                    socket.last_data_times_by_topic[topic_name] = t
            except Exception as e:
                print("Error sending message: %s" % str(e))
                traceback.print_exc()

    def on_message(self, message):
        try:
            cmd = json.loads(message)
        except (ValueError, TypeError):
            print("error: bad: %s" % message)
            return

        if type(cmd) is not list or len(cmd) < 1 or type(cmd[0]) is not str:
            print("error: bad: %s" % message)
            return

        if cmd[0] == ROSBoardSocketHandler.MSG_PING:
            if len(cmd) != 1:
                print("error: ping: bad: %s" % message)
                return
            self.write_message(json.dumps([ROSBoardSocketHandler.MSG_PONG, time.time()]))

        elif cmd[0] == ROSBoardSocketHandler.MSG_PONG:
            if len(cmd) != 2 or type(cmd[1]) is not dict:
                print("error: pong: bad: %s" % message)
                return

            remote_clock_time = cmd[1].get(ROSBoardSocketHandler.PONG_TIME, 0)
            self.last_pong_time = time.time() * 1000
            self.latency = (self.last_pong_time - self.last_ping_time) / 2
            self.clock_diff = 0.95 * self.clock_diff + 0.05 * ((self.last_pong_time + self.last_ping_time) / 2 - remote_clock_time)

        elif cmd[0] == ROSBoardSocketHandler.MSG_SUB:
            if len(cmd) != 2 or type(cmd[1]) is not dict:
                print("error: sub: bad: %s" % message)
                return

            topic_name = cmd[1].get("topicName")
            max_update_rate = float(cmd[1].get("maxUpdateRate", 24.0))

            self.update_intervals_by_topic[topic_name] = 1.0 / max_update_rate
            self.node.update_intervals_by_topic[topic_name] = min(
                self.node.update_intervals_by_topic.get(topic_name, 1.),
                self.update_intervals_by_topic[topic_name]
            )

            if topic_name is None:
                print("error: no topic specified")
                return

            if topic_name not in self.node.subscriptions:
                self.node.subscriptions[topic_name] = set()

            self.node.subscriptions[topic_name].add(self.id)
            self.node.update_subscriptions()

        elif cmd[0] == ROSBoardSocketHandler.MSG_UNSUB:
            if len(cmd) != 2 or type(cmd[1]) is not dict:
                print("error: unsub: bad: %s" % message)
                return
            topic_name = cmd[1].get("topicName")

            if topic_name not in self.node.subscriptions:
                self.node.subscriptions[topic_name] = set()

            try:
                self.node.subscriptions[topic_name].remove(self.id)
            except KeyError:
                print("KeyError trying to remove sub")

ROSBoardSocketHandler.MSG_PING = "p";
ROSBoardSocketHandler.MSG_PONG = "q";
ROSBoardSocketHandler.MSG_MSG = "m";
ROSBoardSocketHandler.MSG_TOPICS = "t";
ROSBoardSocketHandler.MSG_SUB = "s";
ROSBoardSocketHandler.MSG_UNSUB = "u";

ROSBoardSocketHandler.PONG_TIME = "t";
