import array
import base64
import numpy as np
import rosboard.compression

def ros2dict(msg):
    """
    Converts an arbitrary ROS1/ROS2 message into a JSON-serializable dict.
    """
    if type(msg) in (str, bool, int, float):
        return msg

    if type(msg) is tuple:
        return list(msg)

    if type(msg) is bytes:
        return base64.b64encode(msg).decode()

    output = {}

    if hasattr(msg, "get_fields_and_field_types"): # ROS2
        fields_and_field_types = msg.get_fields_and_field_types()
    elif hasattr(msg, "__slots__"): # ROS1
        fields_and_field_types = msg.__slots__
    else:
        raise ValueError("ros2dict: Does not appear to be a simple type or a ROS message: %s" % str(msg))

    for field in fields_and_field_types:

        # CompressedImage: compress to jpeg
        if (msg.__module__ == "sensor_msgs.msg._CompressedImage" or \
            msg.__module__ == "sensor_msgs.msg._compressed_image") \
            and field == "data":
            rosboard.compression.compress_compressed_image(msg, output)
            continue

        # Image: compress to jpeg
        if (msg.__module__ == "sensor_msgs.msg._Image" or \
            msg.__module__ == "sensor_msgs.msg._image") \
            and field == "data":
            rosboard.compression.compress_image(msg, output)
            continue

        # OccupancyGrid: render and compress to jpeg
        if (msg.__module__ == "nav_msgs.msg._OccupancyGrid" or \
            msg.__module__ == "nav_msgs.msg._occupancy_grid") \
            and field == "data":
            rosboard.compression.compress_occupancy_grid(msg, output)
            continue

        # LaserScan: reduce precision
        if (msg.__module__ == "sensor_msgs.msg._LaserScan" or \
            msg.__module__ == "sensor_msgs.msg._laser_scan") \
            and field == "ranges":
            rosboard.compression.compress_laser_scan(msg, output)
            continue
        if (msg.__module__ == "sensor_msgs.msg._LaserScan" or \
            msg.__module__ == "sensor_msgs.msg._laser_scan") \
            and field == "intensities":
            continue

        # PointCloud2: extract only necessary fields, reduce precision
        if (msg.__module__ == "sensor_msgs.msg._PointCloud2" or \
            msg.__module__ == "sensor_msgs.msg._point_cloud2") \
            and field == "data" and msg.data:
            rosboard.compression.compress_point_cloud2(msg, output)
            continue

        value = getattr(msg, field)
        if type(value) in (str, bool, int, float):
            output[field] = value

        elif type(value) is bytes:
            output[field] = base64.b64encode(value).decode()

        elif type(value) is tuple:
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
