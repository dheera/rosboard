#!/usr/bin/env python3

import numpy as np
import array

def ros2dict(msg):
    """
    Converts an arbitrary ROS message into a JSON-serializable dict.
    """
    if type(msg) in (str, bool, int, float):
        return msg

    output = {}

    try:
        fields_and_field_types = msg.get_fields_and_field_types()
    except AttributeError:
        raise ValueError("ros2dict: Does not appear to be a simple type or a ROS message: %s" % str(msg))

    for field in fields_and_field_types:
        value = getattr(msg, field)
        if type(value) in (str, bool, int, float):
            output[field] = value

        elif type(value) is list:
            output[field] = [ros2dict(el) for el in value]

        elif type(value) in (np.ndarray, array.array):
            output[field] = value.tolist()

        else:
            output[field] = ros2dict(value)

    return output

if __name__ == "__main__":
    # Run unit tests
    print("str")
    print(ros2dict("test"))
    print("Path")
    from nav_msgs.msg import Path
    print(ros2dict(Path()))
    print("NavSatFix")
    from sensor_msgs.msg import NavSatFix
    print(ros2dict(NavSatFix()))
    print("Int32MultiArray")
    from std_msgs.msg import Int32MultiArray
    print(ros2dict(Int32MultiArray()))
    print("object (this should not work)")
    try:
        print(ros2dict(object()))
    except ValueError:
        print("exception successfully caught")
    print("all tests completed successfully")
