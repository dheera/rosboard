#!/usr/bin/env python3

try:
    import rospy # ROS1
except ImportError:
    try:
        import rosboard.rospy2 as rospy # ROS2, run as module
    except ImportError:
        import rospy2 as rospy # ROS2, run directly
except ModuleNotFoundError as e:
    print(str(e))
    exit(1)

try:
    import tornado, tornado.web, tornado.websocket
except ImportError:
    print("Please install tornado (sudo pip3 install tornado)")
    exit(2)

import asyncio
from collections import namedtuple
from functools import partial
import importlib
import json
import socket
import os
from pprint import pprint
import re
import sys
import time
import subprocess
import threading
import traceback
import uuid

from rosgraph_msgs.msg import Log

import pdb_attach
pdb_attach.listen(50000)

try:
    # module imports
    from .message_helper import ros2dict
    from .dmesg_subscriber import DMesgSubscriber
    from .dummy_subscriber import DummySubscriber
except:
    # try harder goddamnit stupid python3
    from message_helper import ros2dict
    from dmesg_subscriber import DMesgSubscriber
    from dummy_subscriber import DummySubscriber

TopicDescription = namedtuple(field_names = ["name", "msg_class", "type"], typename = "TopicDescription")

def mean(list):
    return sum(list)/len(list)

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class ROSBoardSocketHandler(tornado.websocket.WebSocketHandler):
    sockets = set()
    cache = []
    cache_size = 200

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        self.id = uuid.uuid4()
        self.latency = 0
        self.clock_diff = 0
        self.last_ping_time = 0
        self.last_pong_time = 0
        ROSBoardSocketHandler.sockets.add(self)

    def on_close(self):
        ROSBoardSocketHandler.sockets.remove(self)
        for topic_name in ROSBoardNode.instance.subscriptions:
            if self.id in ROSBoardNode.instance.subscriptions[topic_name]:
                ROSBoardNode.instance.subscriptions[topic_name].remove(self.id)

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
                socket.write_message(json.dumps(["ping"]))
            except Exception as e:
                print("Error sending message: %s", str(e))

    @classmethod
    def broadcast(cls, message):
        for socket in cls.sockets:
            try:
                if message[0] == "topics":
                    socket.write_message(json.dumps(message))
                elif message[0] == "ros_msg":
                    topic_name = message[1]["_topic_name"]
                    if topic_name not in ROSBoardNode.instance.subscriptions:
                        continue
                    if socket.id not in ROSBoardNode.instance.subscriptions[topic_name]:
                        continue
                    ros_msg_dict = message[1]
                    socket.write_message(json.dumps(message))
            except Exception as e:
                print("Error sending message: %s", str(e))

    def on_message(self, message):
        try:
            cmd = json.loads(message)
        except (ValueError, TypeError):
            print("error: bad: %s" % message)
            return

        if type(cmd) is not list or len(cmd) < 1 or type(cmd[0]) is not str:
            print("error: bad: %s" % message)
            return

        if cmd[0] == "ping":
            if len(cmd) != 1:
                print("error: ping: bad: %s" % message)
                return
            self.write_message(json.dumps(["pong", time.time()]))

        elif cmd[0] == "pong":
            if len(cmd) != 2 or type(cmd[1]) not in (int, float):
                print("error: pong: bad: %s" % message)
                return

            remote_clock_time = cmd[1]
            self.last_pong_time = time.time() * 1000
            self.latency = (self.last_pong_time - self.last_ping_time) / 2
            self.clock_diff = 0.95 * self.clock_diff + 0.05 * ((self.last_pong_time + self.last_ping_time) / 2 - remote_clock_time)

        elif cmd[0] == "sub":
            if len(cmd) != 2:
                print("error: sub: bad: %s" % message)
                return
            topic_name = cmd[1]

            if topic_name not in ROSBoardNode.instance.subscriptions:
                ROSBoardNode.instance.subscriptions[topic_name] = set()

            ROSBoardNode.instance.subscriptions[topic_name].add(self.id)
            ROSBoardNode.instance.update_subscriptions()

        elif cmd[0] == "unsub":
            if len(cmd) != 2:
                print("error: unsub: bad: %s" % message)
                return
            topic_name = cmd[1]

            if topic_name not in ROSBoardNode.instance.subscriptions:
                ROSBoardNode.instance.subscriptions[topic_name] = set()

            try:
                ROSBoardNode.instance.subscriptions[topic_name].remove(self.id)
            except KeyError:
                print("KeyError trying to remove sub")


class ROSBoardNode(object):
    instance = None
    def __init__(self, node_name = "rosboard_node"):
        self.__class__.instance = self
        rospy.init_node(node_name)
        self.port = rospy.get_param("~port", 8888)

        self.subscriptions = {}

        if rospy.__name__ == "rospy2":
            # ros2 hack: need to subscribe to at least 1 topic
            # before dynamic subscribing will work later.
            # ros2 docs don't explain why but we need this magic
            self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x:x)

        self.subs = {}

        tornado_settings = {
            'debug': True, 
            'static_path': os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html')
        }

        tornado_handlers = [
                (r"/rosboard/v1", ROSBoardSocketHandler),
                (r"/(.*)", NoCacheStaticFileHandler, {
                    "path": tornado_settings.get("static_path"),
                    "default_filename": "index.html"
                }),
        ]

        self.event_loop = None
        self.tornado_application = tornado.web.Application(tornado_handlers, **tornado_settings)
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.event_loop = tornado.ioloop.IOLoop()
        self.tornado_application.listen(self.port)
        threading.Thread(target = self.event_loop.start, daemon = True).start()
        threading.Thread(target = self.update_subscriptions_loop, daemon = True).start()
        threading.Thread(target = self.pingpong_loop, daemon = True).start()

    def start(self):
        rospy.spin()

    def get_msg_class(self, msg_type):
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            rospy.logerr("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            return getattr(importlib.import_module(msg_module), msg_class_name)
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def pingpong_loop(self):
        while True:
            time.sleep(5)

            if self.event_loop is None:
                continue
            try:
                self.event_loop.add_callback(ROSBoardSocketHandler.send_pings)
            except Exception as e:
                rospy.logwarn(str(e))
                traceback.print_exc()

    def update_subscriptions_loop(self):
        """
        Subscribes to robot topics and keeps track of them in self.sub_devices
        """
        while True:
            time.sleep(1)
            self.update_subscriptions()

    def update_subscriptions(self):
        try:
            # all topics and their types as strings e.g. {"/foo": "std_msgs/String", "/bar": "std_msgs/Int32"}
            self.all_topics = {}

            for topic_tuple in rospy.get_published_topics():
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]
                if type(topic_type) is list:
                    topic_type = topic_type[0] # ROS2
                self.all_topics[topic_name] = topic_type

            self.event_loop.add_callback(
                ROSBoardSocketHandler.broadcast,
                ["topics", self.all_topics]
            )

            for topic_name in self.subscriptions:
                if len(self.subscriptions[topic_name]) == 0:
                    continue

                if topic_name == "_dmesg":
                    if topic_name not in self.subs:
                        rospy.loginfo("Subscribing to dmesg [non-ros]")
                        self.subs[topic_name] = DMesgSubscriber(self.on_dmesg)
                    continue

                if topic_name not in self.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                if topic_name not in self.subs:
                    topic_type = self.all_topics[topic_name]
                    rospy.loginfo("Subscribing to %s" % topic_name)
                    msg_class = self.get_msg_class(topic_type)
                    if msg_class is None:
                        self.subs[topic_name] = DummySubscriber()
                        continue

                    self.subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args = (topic_name, topic_type),
                    )

            for topic_name in list(self.subs.keys()):
                if topic_name not in self.subscriptions or \
                    len(self.subscriptions[topic_name]) == 0:
                        rospy.loginfo("Unsubscribing from %s" % topic_name)
                        self.subs[topic_name].unregister()
                        del(self.subs[topic_name])

        except Exception as e:
            rospy.logwarn(str(e))
            traceback.print_exc()

    def on_dmesg(self, text):
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                "ros_msg",
                {
                    "_topic_name": "_dmesg",
                    "_topic_type": "rcl_interfaces/msg/Log",
                    "msg": text,
                },
            ]
        )

    def on_ros_msg(self, msg, topic_info):
        """
        Callback for a robot state topic.
        """
        topic_name, topic_type = topic_info

        if self.event_loop is None:
            return

        ros_msg_dict = ros2dict(msg)
        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = time.time() * 1000

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            ["ros_msg", ros_msg_dict]
        )

def main(args=None):
    ROSBoardNode().start()

if __name__ == '__main__':
    main()

