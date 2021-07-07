#!/usr/bin/env python3

try:
    import rospy # ROS1
except ImportError:
    import rospy2 as rospy # ROS2

import asyncio
import importlib
import json
import socket
import os
import tornado, tornado.web, tornado.websocket
import re
import sys
import time
import subprocess
import threading
import traceback
import uuid

from pprint import pprint

from collections import namedtuple
from functools import partial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String, Int32, Int64, Float32
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Temperature

try:
    from .message_helper import ros2dict
except: # try harder stupid python3
    from message_helper import ros2dict

TopicDescription = namedtuple(field_names = ["name", "msg_class", "type"], typename = "TopicDescription")

def mean(list):
    return sum(list)/len(list)

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class ROSBoardSocketHandler(tornado.websocket.WebSocketHandler):
    waiters = set()
    msg_by_topic_by_waiters = {}
    cache = []
    cache_size = 200

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        self.id = uuid.uuid4()
        ROSBoardSocketHandler.waiters.add(self)

    def on_close(self):
        ROSBoardSocketHandler.waiters.remove(self)

    @classmethod
    def update_cache(cls, chat):
        cls.cache.append(chat)
        if len(cls.cache) > cls.cache_size:
            cls.cache = cls.cache[-cls.cache_size :]

    @classmethod
    def send_message(cls, message):
        for waiter in cls.waiters:
            try:
                if message[0] == "topics":
                    waiter.write_message(json.dumps(["topics", message[1]]))
                elif message[0] == "ros_msg":
                    ros_msg_dict = message[1]
                    diff_dict = {}
                    if waiter.id not in cls.msg_by_topic_by_waiters:
                        cls.msg_by_topic_by_waiters[waiter.id] = {}
                    if ros_msg_dict["_topic_name"] not in cls.msg_by_topic_by_waiters[waiter.id]:
                        cls.msg_by_topic_by_waiters[waiter.id][ros_msg_dict["_topic_name"]] = {}
                    for key, value in ros_msg_dict.items():
                        if value != cls.msg_by_topic_by_waiters[waiter.id][ros_msg_dict["_topic_name"]].get(key):
                            cls.msg_by_topic_by_waiters[waiter.id][ros_msg_dict["_topic_name"]][key] = value
                            diff_dict[key] = value

                    diff_dict["_topic_name"] = ros_msg_dict["_topic_name"]
                    diff_dict["_topic_type"] = ros_msg_dict["_topic_type"]
                    waiter.write_message(json.dumps(["ros_msg", diff_dict]))
            except:
                print("Error sending message", traceback.format_exc())

    def on_message(self, message):
        print(message)

        try:
            cmd = json.loads(message)
        except (ValueError, TypeError):
            print("error: bad command: %s" % message)
            return

        if type(cmd) is not list or len(cmd) < 2:
            print("error: bad command: %s" % message)
            return

        if cmd[0] == "sub":
            topic_name = cmd[1]

            if topic_name not in ROSBoardNode.subscriptions:
                ROSBoardNode.subscriptions[topic_name] = set()

            ROSBoardNode.subscriptions[topic_name].add(self.id)

        elif cmd[0] == "unsub":
            topic_name = cmd[1]

            if topic_name not in ROSBoardNode.subscriptions:
                ROSBoardNode.subscriptions[topic_name] = set()

            try:
                ROSBoardNode.subscriptions[topic_name].remove(self.id)
            except KeyError:
                print("KeyError trying to remove sub")


class ROSBoardNode(object):
    def __init__(self, node_name = "rosboard_node"):
        rospy.init_node(node_name)
        self.port = rospy.get_param("~port", 8888)
        self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x:x)

        self.subs = {}

        tornado_settings = {
            'debug': True, 
            'static_path': os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html')
        }

        tornado_handlers = [
                (r"/socket", ROSBoardSocketHandler),
                (r"/(.*)", NoCacheStaticFileHandler, {
                    "path": tornado_settings.get("static_path"),
                    "default_filename": "index.html"
                }),
        ]

        self.event_loop = None
        tornado_application = tornado.web.Application(tornado_handlers, **tornado_settings)
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.event_loop = tornado.ioloop.IOLoop()
        tornado_application.listen(self.port)
        threading.Thread(target = self.event_loop.start, daemon = True).start()
        threading.Thread(target = self.update_topics_loop, daemon = True).start()

    def start(self):
        rospy.spin()

    def get_msg_class(self, msg_type):
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            self.log.error("invalid type %s" % msg_type)
            return None
        return getattr(importlib.import_module(msg_module), msg_class_name)

    def update_topics_loop(self):
        """
        Subscribes to robot topics and keeps track of them in self.sub_devices
        """
        while True:
            time.sleep(1)
            
            # all topics and their types as strings e.g. {"/foo": "std_msgs/String", "/bar": "std_msgs/Int32"}
            ROSBoardNode.all_topics = {}

            for topic_tuple in rospy.get_published_topics():
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]
                if type(topic_type) is list:
                    topic_type = topic_type[0] # ROS2
                ROSBoardNode.all_topics[topic_name] = topic_type

            self.event_loop.add_callback(
                ROSBoardSocketHandler.send_message,
                ["topics", ROSBoardNode.all_topics]
            )

            for topic_name in ROSBoardNode.subscriptions:
                if topic_name not in ROSBoardNode.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                if topic_name not in self.subs:
                    topic_type = ROSBoardNode.all_topics[topic_name]
                    rospy.loginfo("Subscribing to %s" % topic_name)
                    print(topic_name,
                        self.get_msg_class(topic_type))
                    self.subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        (topic_name, topic_type),
                    )

    def on_ros_msg(self, msg, topic_name, topic_type):
        """
        Callback for a robot state topic.
        """
        print(msg, topic_name, topic_type)
        if self.event_loop is not None:
            ros_msg_dict = ros2dict(msg)
            ros_msg_dict["_topic_name"] = topic_name
            ros_msg_dict["_topic_type"] = topic_type
            self.event_loop.add_callback(
                ROSBoardSocketHandler.send_message,
                ["ros_msg", ros_msg_dict]
            )

ROSBoardNode.subscriptions = {}

def main(args=None):
    ROSBoardNode().start()

if __name__ == '__main__':
    main()

