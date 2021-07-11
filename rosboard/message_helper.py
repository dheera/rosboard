#!/usr/bin/env python3

import numpy as np
import array

import io

try:
    import PIL
    from PIL import Image
except ImportError:
    PIL = None

import base64

try:
    from .cv_bridge import imgmsg_to_cv2
except: # try harder stupid python3
    from cv_bridge import imgmsg_to_cv2

def ros2dict(msg):
    """
    Converts an arbitrary ROS1/ROS2 message into a JSON-serializable dict.
    """
    if type(msg) in (str, bool, int, float):
        return msg

    if type(msg) is tuple:
        return list(msg)

    output = {}

    if hasattr(msg, "get_fields_and_field_types"): # ROS2
        fields_and_field_types = msg.get_fields_and_field_types()
    elif hasattr(msg, "__slots__"): # ROS1
        fields_and_field_types = msg.__slots__
    else:
        raise ValueError("ros2dict: Does not appear to be a simple type or a ROS message: %s" % str(msg))

    for field in fields_and_field_types:

        if msg.__module__ == "sensor_msgs.msg._CompressedImage" and field == "data":
            if PIL is None:
                output["_error"] = "Please install PIL for image support."
                continue
            img = Image.open(io.BytesIO(bytearray(msg.data)))
            buffered = io.BytesIO()
            img.save(buffered, format="JPEG", quality = 50)
            output["data"] = []
            output["_img_jpeg"] = base64.b64encode(buffered.getvalue()).decode()
            continue

        if msg.__module__ == "sensor_msgs.msg._Image" and field == "data":
            if PIL is None:
                output["_error"] = "Please install PIL for image support."
                continue
            cv2_img = imgmsg_to_cv2(msg, flip_channels = True)
            while cv2_img.shape[0] > 800 or cv2_img.shape[1] > 800:
                cv2_img = cv2_img[::2,::2]
            if cv2_img.dtype == np.uint32:
                cv2_img = (cv2_img >> 24).astype(np.uint8)
            elif cv2_img.dtype == np.uint16:
                cv2_img = (cv2_img >> 8).astype(np.uint8)

            output["data"] = []
            try:
                img = Image.fromarray(cv2_img)
                buffered = io.BytesIO()
                img.save(buffered, format="JPEG", quality = 50)
                output["_img_jpeg"] = base64.b64encode(buffered.getvalue()).decode()
            except OSError as e:
                output["_error"] = str(e)
            continue

        value = getattr(msg, field)
        if type(value) in (str, bool, int, float):
            output[field] = value

        if type(value) is tuple:
            output[field] = list(value)

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
