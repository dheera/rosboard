#!/usr/bin/env python3

import asyncio
import importlib
import os
import socket
import threading
import time
import tornado, tornado.web, tornado.websocket
import traceback

if os.environ.get("ROS_VERSION") == "1":
    import rospy # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import rosboard.rospy2 as rospy # ROS2
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)

from rosgraph_msgs.msg import Log

from rosboard.serialization import ros2dict
from rosboard.serialization import dict2ros
from rosboard.subscribers.dmesg_subscriber import DMesgSubscriber
from rosboard.subscribers.processes_subscriber import ProcessesSubscriber
from rosboard.subscribers.system_stats_subscriber import SystemStatsSubscriber
from rosboard.subscribers.dummy_subscriber import DummySubscriber
from rosboard.handlers import ROSBoardSocketHandler, NoCacheStaticFileHandler

class ROSBoardNode(object):
    instance = None
    def __init__(self, node_name = "rosboard_node"):
        self.__class__.instance = self
        rospy.init_node(node_name)
        self.port = rospy.get_param("~port", 8888)
        self.title = rospy.get_param("~title", socket.gethostname())

        # desired subscriptions of all the websockets connecting to this instance.
        # these remote subs are updated directly by "friend" class ROSBoardSocketHandler.
        # this class will read them and create actual ROS subscribers accordingly.
        # dict of topic_name -> set of sockets
        self.remote_subs = {}

        # actual ROS subscribers.
        # dict of topic_name -> ROS Subscriber
        self.local_subs = {}

        # minimum update interval per topic (throttle rate) amang all subscribers to a particular topic.
        # we can throw data away if it arrives faster than this
        # dict of topic_name -> float (interval in seconds)
        self.update_intervals_by_topic = {}

        # last time data arrived for a particular topic
        # dict of topic_name -> float (time in seconds)
        self.last_data_times_by_topic = {}

        if rospy.__name__ == "rospy2":
            # ros2 hack: need to subscribe to at least 1 topic
            # before dynamic subscribing will work later.
            # ros2 docs don't explain why but we need this magic.
            self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x:x)

        tornado_settings = {
            'debug': True,
            'static_path': os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html')
        }

        tornado_handlers = [
                (r"/rosboard/v1", ROSBoardSocketHandler, {
                    "node": self,
                }),
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

        # allows tornado to log errors to ROS
        self.logwarn = rospy.logwarn
        self.logerr = rospy.logerr

        # tornado event loop. all the web server and web socket stuff happens here
        threading.Thread(target = self.event_loop.start, daemon = True).start()

        # loop to sync remote (websocket) subs with local (ROS) subs
        threading.Thread(target = self.sync_subs_loop, daemon = True).start()

        # loop to keep track of latencies and clock differences for each socket
        threading.Thread(target = self.pingpong_loop, daemon = True).start()

        self.lock = threading.Lock()

        rospy.loginfo("ROSboard listening on :%d" % self.port)

    def start(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def call_service(self, service_name, service_type, request_data):
        """
        Calls a ROS service with the given request data.
        """
        rospy.loginfo(f"call_service called: {service_name} [{service_type}] with {request_data}")
        
        try:
            service_class = self.get_service_class(service_type)
            if service_class is None:
                error_msg = f"Could not load service type '{service_type}' for calling"
                rospy.logerr(error_msg)
                return {"success": False, "error": error_msg}

            # Create service proxy
            rospy.loginfo("Calling service %s [%s]" % (service_name, service_type))
            
            if rospy.__name__ == "rospy2":
                # ROS2
                service_proxy = rospy._node.create_client(service_class, service_name)
                
                # Wait for service to become available
                if not service_proxy.wait_for_service(timeout_sec=5.0):
                    error_msg = f"Service '{service_name}' not available"
                    rospy.logerr(error_msg)
                    return {"success": False, "error": error_msg}
                
                # Create request
                request = dict2ros(request_data, service_class.Request)
                
                # Call service
                future = service_proxy.call_async(request)
                
                # Use busy waiting instead of spin_until_future_complete to avoid executor conflicts
                import time
                start_time = time.time()
                timeout_duration = 10.0
                
                while not future.done():
                    if time.time() - start_time > timeout_duration:
                        error_msg = "Service call timed out"
                        rospy.logerr(error_msg)
                        return {"success": False, "error": error_msg}
                    time.sleep(0.001)  # Small sleep to avoid busy loop
                
                if future.result() is not None:
                    response_dict = ros2dict(future.result())
                    rospy.loginfo(f"Service call successful: {response_dict}")
                    return {"success": True, "response": response_dict}
                else:
                    error_msg = "Service call failed"
                    rospy.logerr(error_msg)
                    return {"success": False, "error": error_msg}
            else:
                # ROS1
                rospy.wait_for_service(service_name, timeout=5.0)
                service_proxy = rospy.ServiceProxy(service_name, service_class)
                
                # Create request from dict
                request = dict2ros(request_data, service_class._request_class)
                
                # Call service
                response = service_proxy(request)
                response_dict = ros2dict(response)
                rospy.loginfo(f"Service call successful: {response_dict}")
                return {"success": True, "response": response_dict}

        except Exception as e:
            error_msg = f"Error calling service {service_name}: {str(e)}"
            rospy.logerr(error_msg)
            import traceback
            traceback.print_exc()
            return {"success": False, "error": str(e)}

    def get_service_class(self, service_type):
        """
        Given a ROS service type specified as a string, e.g.
            "example_interfaces/AddTwoInts"
        or
            "example_interfaces/srv/AddTwoInts"
        it imports the service class into Python and returns the class.
        """
        try:
            service_module, dummy, service_class_name = service_type.replace("/", ".").rpartition(".")
        except ValueError:
            rospy.logerr("invalid service type %s" % service_type)
            return None

        try:
            if not service_module.endswith(".srv"):
                service_module = service_module + ".srv"
            return getattr(importlib.import_module(service_module), service_class_name)
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def populate_message_from_dict(self, msg, data_dict):
        """Populate a ROS message from a dictionary."""
        for key, value in data_dict.items():
            if hasattr(msg, key):
                setattr(msg, key, value)
    
    def message_to_dict(self, msg):
        """Convert a ROS message to a dictionary."""
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            if hasattr(value, '__slots__'):  # nested message
                result[slot] = self.message_to_dict(value)
            else:
                result[slot] = value
        return result

    def get_msg_class(self, msg_type):
        """
        Given a ROS message type specified as a string, e.g.
            "std_msgs/Int32"
        or
            "std_msgs/msg/Int32"
        it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

        Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
        """
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

    if os.environ.get("ROS_VERSION") == "2":
        def get_topic_qos(self, topic_name: str) -> QoSProfile:
            """!
            Given a topic name, get the QoS profile with which it is being published
            @param topic_name (str) the topic name
            @return QosProfile the qos profile with which the topic is published. If no publishers exist
            for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
            """
            if rospy.__name__ == "rospy2":
                topic_info = rospy._node.get_publishers_info_by_topic(topic_name=topic_name)
                if len(topic_info):
                    if topic_info[0].qos_profile.history == HistoryPolicy.UNKNOWN:
                        topic_info[0].qos_profile.history = HistoryPolicy.KEEP_LAST
                    return topic_info[0].qos_profile
                else:
                    rospy.logwarn(f"No publishers available for topic {topic_name}. Returning sensor data QoS")
                    return QoSProfile(
                            depth=10,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            # reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        )
            else:
                rospy.logwarn("QoS profiles are only used in ROS2")
                return None

    def pingpong_loop(self):
        """
        Loop to send pings to all active sockets every 5 seconds.
        """
        while True:
            time.sleep(5)

            if self.event_loop is None:
                continue
            try:
                self.event_loop.add_callback(ROSBoardSocketHandler.send_pings)
            except Exception as e:
                rospy.logwarn(str(e))
                traceback.print_exc()

    def sync_subs_loop(self):
        """
        Periodically calls self.sync_subs(). Intended to be run in a thread.
        """
        while True:
            time.sleep(1)
            self.sync_subs()

    def sync_subs(self):
        """
        Looks at self.remote_subs and makes sure local subscribers exist to match them.
        Also cleans up unused local subscribers for which there are no remote subs interested in them.
        """

        # Acquire lock since either sync_subs_loop or websocket may call this function (from different threads)
        self.lock.acquire()

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
                [ROSBoardSocketHandler.MSG_TOPICS, self.all_topics ]
            )
            
            # all services and their types
            self.all_services = {}
            
            try:
                if rospy.__name__ == "rospy2":
                    # ROS2
                    for service_tuple in rospy._node.get_service_names_and_types():
                        service_name = service_tuple[0]
                        service_type = service_tuple[1]
                        if type(service_type) is list and len(service_type) > 0:
                            service_type = service_type[0]
                        self.all_services[service_name] = service_type
                else:
                    # ROS1 - using rosservice list
                    try:
                        import subprocess
                        result = subprocess.run(['rosservice', 'list'], capture_output=True, text=True, timeout=2)
                        if result.returncode == 0:
                            service_names = [s.strip() for s in result.stdout.split('\n') if s.strip()]
                            for service_name in service_names:
                                try:
                                    type_result = subprocess.run(['rosservice', 'type', service_name], capture_output=True, text=True, timeout=1)
                                    if type_result.returncode == 0:
                                        service_type = type_result.stdout.strip()
                                        if service_type:
                                            self.all_services[service_name] = service_type
                                except:
                                    pass  # Skip services that can't be queried
                    except:
                        pass  # If rosservice commands fail, just leave services empty
                        
                self.event_loop.add_callback(
                    ROSBoardSocketHandler.broadcast,
                    [ROSBoardSocketHandler.MSG_SERVICES, self.all_services ]
                )
            except Exception as e:
                rospy.logwarn("Failed to get service list: %s" % str(e))

            for topic_name in self.remote_subs:
                if len(self.remote_subs[topic_name]) == 0:
                    continue

                # remote sub special (non-ros) topic: _dmesg
                # handle it separately here
                if topic_name == "_dmesg":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to dmesg [non-ros]")
                        self.local_subs[topic_name] = DMesgSubscriber(self.on_dmesg)
                    continue

                if topic_name == "_system_stats":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _system_stats [non-ros]")
                        self.local_subs[topic_name] = SystemStatsSubscriber(self.on_system_stats)
                    continue

                if topic_name == "_top":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _top [non-ros]")
                        self.local_subs[topic_name] = ProcessesSubscriber(self.on_top)
                    continue

                # check if remote sub request is not actually a ROS topic before proceeding
                if topic_name not in self.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                # if the local subscriber doesn't exist for the remote sub, create it
                if topic_name not in self.local_subs:
                    topic_type = self.all_topics[topic_name]
                    msg_class = self.get_msg_class(topic_type)

                    if msg_class is None:
                        # invalid message type or custom message package not source-bashed
                        # put a dummy subscriber in to avoid returning to this again.
                        # user needs to re-run rosboard with the custom message files sourced.
                        self.local_subs[topic_name] = DummySubscriber()
                        self.event_loop.add_callback(
                            ROSBoardSocketHandler.broadcast,
                            [
                                ROSBoardSocketHandler.MSG_MSG,
                                {
                                    "_topic_name": topic_name, # special non-ros topics start with _
                                    "_topic_type": topic_type,
                                    "_error": "Could not load message type '%s'. Are the .msg files for it source-bashed?" % topic_type,
                                },
                            ]
                        )
                        continue

                    self.last_data_times_by_topic[topic_name] = 0.0

                    rospy.loginfo("Subscribing to %s" % topic_name)

                    kwargs = {}
                    if rospy.__name__ == "rospy2":
                        # In ros2 we also can pass QoS parameters to the subscriber.
                        # To avoid incompatibilities we subscribe using the same Qos
                        # of the topic's publishers
                        kwargs = {"qos": self.get_topic_qos(topic_name)}
                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args = (topic_name, topic_type),
                        **kwargs
                    )

            # clean up local subscribers for which remote clients have lost interest
            for topic_name in list(self.local_subs.keys()):
                if topic_name not in self.remote_subs or \
                    len(self.remote_subs[topic_name]) == 0:
                        rospy.loginfo("Unsubscribing from %s" % topic_name)
                        self.local_subs[topic_name].unregister()
                        del(self.local_subs[topic_name])

        except Exception as e:
            rospy.logwarn(str(e))
            traceback.print_exc()

        self.lock.release()

    def on_system_stats(self, system_stats):
        """
        system stats received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        msg_dict = {
            "_topic_name": "_system_stats", # special non-ros topics start with _
            "_topic_type": "rosboard_msgs/msg/SystemStats",
        }

        for key, value in system_stats.items():
            msg_dict[key] = value

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                msg_dict
            ]
        )

    def on_top(self, processes):
        """
        processes list received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_top", # special non-ros topics start with _
                    "_topic_type": "rosboard_msgs/msg/ProcessList",
                    "processes": processes,
                },
            ]
        )

    def on_dmesg(self, text):
        """
        dmesg log received. make it look like a rcl_interfaces/msg/Log and send it off
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_dmesg", # special non-ros topics start with _
                    "_topic_type": "rcl_interfaces/msg/Log",
                    "msg": text,
                },
            ]
        )

    def on_ros_msg(self, msg, topic_info):
        """
        ROS messaged received (any topic or type).
        """
        topic_name, topic_type = topic_info
        t = time.time()
        if t - self.last_data_times_by_topic.get(topic_name, 0) < self.update_intervals_by_topic[topic_name] - 1e-4:
            return

        if self.event_loop is None:
            return

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(msg)

        # add metadata
        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = time.time() * 1000

        # log last time we received data on this topic
        self.last_data_times_by_topic[topic_name] = t

        # broadcast it to the listeners that care
        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [ROSBoardSocketHandler.MSG_MSG, ros_msg_dict]
        )

def main(args=None):
    ROSBoardNode().start()

if __name__ == '__main__':
    main()
