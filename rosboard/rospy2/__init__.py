#!/usr/bin/env python3

# rospy2
# Copyright (c) 2021, Dheera Venkatraman (https://dheera.net/)
# https://github.com/dheera/rospy2
# BSD 3-Clause License

import hashlib
import importlib
import os
import random
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.qos_event
import sys
import time
import types
import threading
from .constants import *
import builtin_interfaces.msg

__version__ = "1.0.2"
__name__ = "rospy2"

numpy = None # may be imported later
array = None # may be imported later

ARRAY_TO_LIST = False

rclpy.init(args = sys.argv)

# Variables starting with an underscore are private / not intended to be used directly.
# All normally-named functions/classes/methods are designed to function exactly
# as per their ROS1 namesake.

_node = None
_logger = None
_clock = None
_thread_spin = None
_wait_for_message_release = False
_on_shutdown = None

def get_param(param_name, default_value = None):
    global _node
    if param_name.startswith("/"):
        logerror("Getting parameters of other nodes is not yet supported")
        return 0

    param_name = param_name.strip("~")
    if not _node.has_parameter(param_name):
        _node.declare_parameter(param_name, default_value)
    return _node.get_parameter(param_name)._value

def init_node(node_name, anonymous=False, log_level=INFO, disable_signals=False):
    global _node, _logger, _clock, _thread_spin
    if anonymous:
        node_name += "_" + str(random.randint(10000,99999))

    _node = rclpy.create_node(
        node_name,
        allow_undeclared_parameters = True,
        automatically_declare_parameters_from_overrides = True,
    )
    _logger = _node.get_logger()
    _clock = _node.get_clock()

    ros2_log_level = {
        DEBUG: rclpy.logging.LoggingSeverity.DEBUG,
        INFO: rclpy.logging.LoggingSeverity.INFO,
        WARN: rclpy.logging.LoggingSeverity.WARN,
        ERROR: rclpy.logging.LoggingSeverity.ERROR,
        FATAL: rclpy.logging.LoggingSeverity.FATAL,
    }.get(log_level, rclpy.logging.LoggingSeverity.UNSET)
    rclpy.logging.set_logger_level(_logger.name, ros2_log_level)

    _thread_spin = threading.Thread(target=_thread_spin_target, daemon=True)
    _thread_spin.start()

def _thread_spin_target():
    global _on_shutdown, _node
    rclpy.spin(_node)
    if _on_shutdown:
        _on_shutdown()

is_shutdown = lambda: not rclpy.ok()

logdebug = lambda text: _logger.debug(text)
logdebug_once = lambda text: _logger.debug(text, once = True)
logdebug_throttle = lambda interval, text: _logger.debug(text, throttle_duration_sec = interval)

loginfo = lambda text: _logger.info(text)
loginfo_once = lambda text: _logger.info(text, once = True)
loginfo_throttle = lambda interval, text: _logger.info(text, throttle_duration_sec = interval)

logwarn = lambda text: _logger.warn(text)
logwarn_once = lambda text: _logger.warn(text, once = True)
logwarn_throttle = lambda interval, text: _logger.warn(text, throttle_duration_sec = interval)

logerr = lambda text: _logger.error(text)
logerr_once = lambda text: _logger.error(text, once = True)
logerr_throttle = lambda interval, text: _logger.error(text, throttle_duration_sec = interval)

logfatal = lambda text: _logger.fatal(text)
logfatal_once = lambda text: _logger.fatal(text, once = True)
logfatal_throttle = lambda interval, text: _logger.fatal(text, throttle_duration_sec = interval)

def on_shutdown(handler):
    global _on_shutdown
    _on_shutdown = handler

def set_param(parameter_name, parameter_value):
    global _node
    if type(parameter_value) is str:
        parameter_type = rclpy.Parameter.Type.STRING
    elif type(parameter_value) is float:
        parameter_type = rclpy.Parameter.Type.DOUBLE
    elif type(parameter_value) is int:
        parameter_type = rclpy.Parameter.Type.INT
    elif type(parameter_value) is bool:
        parameter_type = rclpy.Parameter.Type.BOOL
    else:
        raise Exception("Invalid parameter value type: %s" % str(type(parameter_value)))

    param = rclpy.parameter.Parameter(
        parameter_name,
        parameter_type,
        parameter_value,
    )
    _node.set_parameters([param])

def signal_shutdown(reason):
    rclpy.shutdown()

def sleep(duration):
    # TODO: replace with version that respects ROS time
    if type(duration) is rclpy.duration.Duration:
        time.sleep(duration.nanoseconds / 1e9)
    else:
        time.sleep(duration)

def spin():
    global _thread_spin
    _thread_spin.join()

def get_published_topics(namespace = '/'):
    return [[el[0], el[1][0]] for el in _node.get_topic_names_and_types()]

def get_caller_id():
    import inspect # import here since this function is highly unlikely to be used
    logwarn_once("get_caller_id() not supported in ROS2. taking a guess by selecting the first publisher. highly recommended to not do this!")
    topic_name = inspect.currentframe().f_back.f_back.f_locals["self"].resolved_name
    x = _node.get_publishers_info_by_topic(topic_name)
    return os.path.join(x[0].node_namespace, x[0].node_name)

def get_time():
    global _clock
    if _clock is None:
        raise ROSInitException("time is not initialized. Have you called init_node()?")
    return _clock.now().nanoseconds/1e9

def wait_for_message(topic_name, topic_type):
    global _node, _wait_for_message_release
    _wait_for_message_release = False
    sub = _node.create_subscriber(topic_name, topic_type, _release_wait_for_message)
    while not _wait_for_message_release:
        time.sleep(0.1)
    _node.destroy_subscriber(sub)

def _release_wait_for_message(self, msg):
    global _wait_for_message_release
    _wait_for_message_release = True

def wait_for_service(service_name):
    global _node
    abs_service_name = os.path.join(_node.get_namespace(), service_name)
    while True:
        for service in _node.get_service_names_and_types():
            if service[0] == abs_service_name:
                break
        time.sleep(0.5)

class Publisher(object):
    def __init__(self, topic_name, topic_type, queue_size = 1):
        global _node
        self.reg_type = "pub"
        self.data_class = topic_type
        self.name = topic_name
        self.resolved_name = topic_name
        self.type = _ros2_type_to_type_name(topic_type)
        self._pub = _node.create_publisher(
            topic_type,
            topic_name,
            rclpy.qos.QoSProfile(depth = queue_size, history = rclpy.qos.HistoryPolicy.KEEP_LAST)
        )
        self.get_num_connections = self._pub.get_subscription_count

    def __del__(self):
        global _node
        _node.destroy_publisher(self._pub)

    @property
    def md5sum(self): # No good ROS2 equivalent, fake it reasonably
        return hashlib.md5(str(self.type.get_fields_and_field_types()).encode("utf-8")).hexdigest()

    def publish(self, msg):
        if type(msg) in (str, int, float, bool):
            msg = self.data_class(data = msg)
        self._pub.publish(msg)

    def unregister(self):
        global _node
        _node.destroy_publisher(self._pub)

class Subscriber(object):
    def __init__(self, topic_name, topic_type, callback, callback_args = None, qos=10):
        global _node
        self.reg_type = "sub"
        self.data_class = topic_type
        self.name = topic_name
        self.resolved_name = topic_name
        self.type = _ros2_type_to_type_name(topic_type)
        self.callback = callback
        self.callback_args = callback_args
        self._sub = _node.create_subscription(topic_type, topic_name, self._ros2_callback, qos, event_callbacks = rclpy.qos_event.SubscriptionEventCallbacks())
        _node.guards
        self.get_num_connections = lambda: 1 # No good ROS2 equivalent

    def __del__(self):
        global _node
        _node.destroy_subscription(self._sub)

    def _ros2_callback(self, msg):
        global numpy, array, ARRAY_TO_LIST
        if ARRAY_TO_LIST:
            if numpy is None:
                numpy = importlib.import_module("numpy")
            if array is None:
                array = importlib.import_module("array")
            for field_name in msg.get_fields_and_field_types():
                value = getattr(msg, field_name)
                if type(value) in (array.array, numpy.ndarray):
                    setattr(msg, "_" + field_name, value.tolist())
        if self.callback_args:
            self.callback(msg, self.callback_args)
        else:
            self.callback(msg)

    @property
    def md5sum(self): # No good ROS2 equivalent, fake it reasonably
        return hashlib.md5(str(self.type.get_fields_and_field_types()).encode("utf-8")).hexdigest()

    def unregister(self):
        global _node
        _node.destroy_subscription(self._sub)

class Service(object):
    def __init__(self, service_name, service_type, callback):
        global _node
        self._srv = _node.create_service(service_type, service_name, callback)

    def __del__(self):
        global _node
        _node.destroy_service(self._srv)

class ServiceProxy(object):
    def __init__(self, service_name, service_type):
        global _node
        self._client = _node.create_client(service_type, service_name)

    def __del__(self):
        global _node
        _node.destroy_client(self._client)
    
    def __call__(self, req):
        global _node
        resp = self._client.call_async(req)
        rclpy.spin_until_future_complete(_node, resp)
        return resp

class Duration(object):
    def __new__(cls, secs, nsecs = 0):
        global _node
        d = rclpy.duration.Duration(nanoseconds = secs * 1000000000 + nsecs)
        d.to_nsec = types.MethodType(lambda self: self.nanoseconds, d)
        d.to_sec = types.MethodType(lambda self: self.nanoseconds / 1e9, d)
        d.is_zero = types.MethodType(lambda self: self.nanoseconds == 0, d)
        d.secs = secs
        d.nsecs = nsecs
        return d

    @classmethod
    def from_sec(cls, secs):
        return rclpy.duration.Duration(nanosecods = secs * 1000000000)

    @classmethod
    def from_seconds(cls, secs):
        return rclpy.duration.Duration(nanosecods = secs * 1000000000)

class Time(object):
    def __new__(cls, secs = 0, nsecs = 0):
        return builtin_interfaces.msg.Time(sec = secs, nanosec = nsecs)

    @classmethod
    def from_sec(cls, secs):
        return builtin_interfaces.msg.Time(sec = secs)

    @classmethod
    def from_seconds(cls, secs):
        return builtin_interfaces.msg.Time(sec = secs)

    @classmethod
    def now(cls):
        global _clock
        if _clock is None:
            raise ROSInitException("time is not initialized. Have you called init_node()?")
        secs, nsecs = _clock.now().seconds_nanoseconds()
        return builtin_interfaces.msg.Time(sec = secs, nanosec = nsecs)

class Rate(object):
    def __init__(self, hz):
        global _node
        self._rate = _node.create_rate(hz)

    def __del__(self):
        global _node
        _node.destroy_rate(self._rate)

    def sleep(self):
        self._rate.sleep()

class Timer(object):
    def __init__(self, timer_period, callback):
        global _node
        self.callback = callback
        self._timer = _node.create_timer(timer_period, self._ros2_callback)

    def __del__(self):
        global _node
        _node.destroy_timer(self._timer)

    def _ros2_callback(self):
        self.callback(TimerEvent(0, 0, 0, 0, 0)) # TODO: fill in these values

class TimerEvent(object):
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected = last_expected
        self.last_real = last_real
        self.current_expected = current_expected
        self.current_real = current_real
        self.last_duration = last_duration

class ROSException(Exception):
    pass

class ROSInitException(ROSException):
    pass

class ROSInternalException(ROSException):
    pass

class ROSInterruptException(ROSException):
    pass

class ROSSerializationException(ROSException):
    pass

class ROSTimeMovedBackwardsException(ROSException):
    pass

class ServiceException(Exception):
    pass

class TransportException(Exception):
    pass

class TransportInitError(Exception):
    pass

class TransportTerminated(Exception):
    pass

def _ros2_type_to_type_name(ros2_type):
    """
    from std_msgs.msg import String # ros2
    _ros2_type_to_type_name(String) # --> "std_msgs/String"
    """
    try:
        first_dot = ros2_type.__module__.find(".")
        return ros2_type[0:first_dot] + "/" + ros2_type.__name__
    except:
        # this shouldn't happen but try harder, don't crash the robot for something silly like this
        return str(ros2_type).replace("<class '", "").replace("'>", "")

class _Module: pass

rostime = _Module()
rostime.Time = Time
rostime.Duration = Duration

exceptions = _Module()
exceptions.ROSException = ROSException
exceptions.ROSInitException = ROSInitException

builtin_interfaces.msg.Time.to_nsec = lambda self: self.sec * 1000000000 + self.nanosec
builtin_interfaces.msg.Time.to_sec = lambda self: self.sec + self.nanosec / 1e9
builtin_interfaces.msg.Time.is_zero = lambda self: self.sec == 0 and self.nanosec == 0
def secs_setter(self, value): self.sec = value
builtin_interfaces.msg.Time.secs = property(lambda self: self.sec, secs_setter)
def nsecs_setter(self, value): self.nanosec = value
builtin_interfaces.msg.Time.nsecs = property(lambda self: self.nanosec, nsecs_setter)

# Allow initializing messages with positional arguments which ROS1 allows but ROS2 doesn't, e.g.
# KeyValue("key", "value")
# Int32(100)

try:
    import diagnostic_msgs.msg
    diagnostic_msgs.msg.KeyValue.__oldinit__ = diagnostic_msgs.msg.KeyValue.__init__
    diagnostic_msgs.msg.KeyValue.__init__ = lambda self, key="", value="": \
        diagnostic_msgs.msg.KeyValue.__oldinit__(self, key = key, value = value)
except ImportError:
    pass #user hasn't installed diagnostic_msgs, we don't need to do anything

try:
    import geometry_msgs.msg
    geometry_msgs.msg.Quaternion.__oldinit__ = geometry_msgs.msg.Quaternion.__init__
    geometry_msgs.msg.Quaternion.__init__ = lambda self, x=0.0, y=0.0, z=0.0, w=0.0: \
        geometry_msgs.msg.Quaternion.__oldinit__(self, x = float(x), y = float(y), z = float(z), w = float(w))
    geometry_msgs.msg.Point.__oldinit__ = geometry_msgs.msg.Point.__init__
    geometry_msgs.msg.Point.__init__ = lambda self, x=0.0, y=0.0, z=0.0: \
        geometry_msgs.msg.Point.__oldinit__(self, x = float(x), y = float(y), z = float(z))
    geometry_msgs.msg.Point32.__oldinit__ = geometry_msgs.msg.Point32.__init__
    geometry_msgs.msg.Point32.__init__ = lambda self, x=0.0, y=0.0, z=0.0: \
        geometry_msgs.msg.Point32.__oldinit__(self, x = float(x), y = float(y), z = float(z))
except ImportError:
    pass #user hasn't installed geometry_msgs, we don't need to do anything

try:
    import rosgraph_msgs.msg
    import rcl_interfaces.msg
    rosgraph_msgs.msg.Log = rcl_interfaces.msg.Log
except:
    pass

import std_msgs.msg
std_msgs.msg.Bool.__oldinit__ = std_msgs.msg.Bool.__init__
std_msgs.msg.Bool.__init__ = lambda self, data = False: std_msgs.msg.Bool.__oldinit__(self, data = data)
std_msgs.msg.Byte.__oldinit__ = std_msgs.msg.Byte.__init__
std_msgs.msg.Byte.__init__ = lambda self, data = 0: std_msgs.msg.Byte.__oldinit__(self, data = data)
std_msgs.msg.Char.__oldinit__ = std_msgs.msg.Char.__init__
std_msgs.msg.Char.__init__ = lambda self, data = 0: std_msgs.msg.Char.__oldinit__(self, data = data)
std_msgs.msg.Int8.__oldinit__ = std_msgs.msg.Int8.__init__
std_msgs.msg.Int8.__init__ = lambda self, data = 0: std_msgs.msg.Int8.__oldinit__(self, data = data)
std_msgs.msg.Int16.__oldinit__ = std_msgs.msg.Int16.__init__
std_msgs.msg.Int16.__init__ = lambda self, data = 0: std_msgs.msg.Int16.__oldinit__(self, data = data)
std_msgs.msg.Int32.__oldinit__ = std_msgs.msg.Int32.__init__
std_msgs.msg.Int32.__init__ = lambda self, data = 0: std_msgs.msg.Int32.__oldinit__(self, data = data)
std_msgs.msg.Int64.__oldinit__ = std_msgs.msg.Int64.__init__
std_msgs.msg.Int64.__init__ = lambda self, data = 0: std_msgs.msg.Int64.__oldinit__(self, data = data)
std_msgs.msg.UInt8.__oldinit__ = std_msgs.msg.UInt8.__init__
std_msgs.msg.UInt8.__init__ = lambda self, data = 0: std_msgs.msg.UInt8.__oldinit__(self, data = data)
std_msgs.msg.UInt16.__oldinit__ = std_msgs.msg.UInt16.__init__
std_msgs.msg.UInt16.__init__ = lambda self, data = 0: std_msgs.msg.UInt16.__oldinit__(self, data = data)
std_msgs.msg.UInt32.__oldinit__ = std_msgs.msg.UInt32.__init__
std_msgs.msg.UInt32.__init__ = lambda self, data = 0: std_msgs.msg.UInt32.__oldinit__(self, data = data)
std_msgs.msg.UInt64.__oldinit__ = std_msgs.msg.UInt64.__init__
std_msgs.msg.UInt64.__init__ = lambda self, data = 0: std_msgs.msg.UInt64.__oldinit__(self, data = data)
std_msgs.msg.Float32.__oldinit__ = std_msgs.msg.Float32.__init__
std_msgs.msg.Float32.__init__ = lambda self, data = 0.0: std_msgs.msg.Float32.__oldinit__(self, data = float(data))
std_msgs.msg.Float64.__oldinit__ = std_msgs.msg.Float64.__init__
std_msgs.msg.Float64.__init__ = lambda self, data = 0.0: std_msgs.msg.Float64.__oldinit__(self, data = float(data))
std_msgs.msg.String.__oldinit__ = std_msgs.msg.String.__init__
std_msgs.msg.String.__init__ = lambda self, data = "": std_msgs.msg.String.__oldinit__(self, data = data)
std_msgs.msg.ColorRGBA.__oldinit__ = std_msgs.msg.ColorRGBA.__init__
std_msgs.msg.ColorRGBA.__init__ = lambda self, r=0.0, g=0.0, b=0.0, a=0.0: \
    std_msgs.msg.ColorRGBA.__oldinit__(self, r = float(r), g = float(g), b = float(b), a = float(a))
std_msgs.msg.Header.__oldinit__ = std_msgs.msg.Header.__init__
std_msgs.msg.Header.seq = property(lambda self: 0, lambda self: None) # seq is deprecated/nonexistent in ROS2
std_msgs.msg.Header.__init__ = lambda self, seq = 0, stamp = builtin_interfaces.msg.Time(), frame_id = "": \
    std_msgs.msg.Header.__oldinit__(self, stamp = stamp, frame_id = frame_id)

