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

def dict2ros(data, ros_message_class):
    """
    Converts a dictionary representation back to a ROS message object.
    This is the inverse function of ros2dict().
    """
    import array
    import base64
    import numpy as np
    
    if ros_message_class in (str, bool, int, float):
        return ros_message_class(data) if data is not None else ros_message_class()
    
    # For primitive types, just return the data directly
    if type(data) in (str, bool, int, float):
        return data
    
    if not isinstance(data, dict):
        return data
        
    # Create an instance of the ROS message
    msg = ros_message_class()
    
    # Get field names and types from the message
    if hasattr(msg, "get_fields_and_field_types"):  # ROS2
        fields_and_field_types = msg.get_fields_and_field_types()
    elif hasattr(msg, "__slots__"):  # ROS1
        fields_and_field_types = msg.__slots__
    else:
        # If we can't get field info, try to set attributes directly
        for key, value in data.items():
            if not key.startswith('_'):  # Skip metadata fields
                try:
                    setattr(msg, key, value)
                except:
                    pass
        return msg
    
    for field_name in fields_and_field_types:
        if field_name not in data:
            continue
            
        field_value = data[field_name]
        
        try:
            # Get the field type if possible
            if hasattr(msg, "get_fields_and_field_types"):  # ROS2
                field_type_str = fields_and_field_types[field_name]
                
                # Handle array types
                if field_type_str.endswith(']'):
                    # This is an array field
                    if isinstance(field_value, list):
                        # Convert list elements if needed
                        base_type = field_type_str.split('[')[0]
                        if base_type in ('float32', 'float64'):
                            field_value = [float(x) for x in field_value]
                        elif base_type in ('int8', 'int16', 'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64'):
                            field_value = [int(x) for x in field_value]
                        elif base_type == 'bool':
                            field_value = [bool(x) for x in field_value]
                        elif base_type == 'string':
                            field_value = [str(x) for x in field_value]
                
                # Handle basic types
                elif field_type_str in ('float32', 'float64'):
                    field_value = float(field_value) if field_value is not None else 0.0
                elif field_type_str in ('int8', 'int16', 'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64'):
                    field_value = int(field_value) if field_value is not None else 0
                elif field_type_str == 'bool':
                    field_value = bool(field_value) if field_value is not None else False
                elif field_type_str == 'string':
                    field_value = str(field_value) if field_value is not None else ""
                    
            # Handle bytes fields (base64 encoded)
            if isinstance(field_value, str) and field_name == 'data':
                current_value = getattr(msg, field_name)
                # Only try base64 decoding if the current field is actually bytes type
                # and the string looks like base64
                if isinstance(current_value, bytes) or (hasattr(current_value, '__len__') and len(current_value) == 0 and hasattr(msg, '__module__')):
                    # Check if this is likely a sensor message with binary data
                    if (hasattr(msg, '__module__') and 
                        ('sensor_msgs' in msg.__module__ or 'nav_msgs' in msg.__module__) and
                        len(field_value) > 10 and 
                        all(c.isalnum() or c in '+/=' for c in field_value)):
                        try:
                            field_value = base64.b64decode(field_value)
                        except:
                            pass  # Not base64, keep as string
                    # For other cases, keep as string unless we know it should be bytes
            
            # Handle nested message fields
            elif isinstance(field_value, dict):
                # This is a nested message, need to get the field type and recursively convert
                try:
                    current_field = getattr(msg, field_name)
                    field_type = type(current_field)
                    field_value = dict2ros(field_value, field_type)
                except:
                    # If we can't get the field type, try setting it directly
                    pass
            
            # Handle lists of nested messages
            elif isinstance(field_value, list) and field_value and isinstance(field_value[0], dict):
                try:
                    current_field = getattr(msg, field_name)
                    if hasattr(current_field, '__len__') and len(current_field) > 0:
                        # Get the type from the first element
                        element_type = type(current_field[0])
                        field_value = [dict2ros(item, element_type) for item in field_value]
                    else:
                        # Try to infer the type from the ROS2 field type annotation
                        if hasattr(msg, "get_fields_and_field_types"):
                            field_type_str = fields_and_field_types[field_name]
                            # This would need more sophisticated type resolution
                            # For now, just set the value directly
                            pass
                except:
                    pass
            
            # Set the field value
            setattr(msg, field_name, field_value)
            
        except Exception as e:
            # If there's an error setting a field, continue with others
            import rospy
            rospy.logwarn(f"Failed to set field {field_name}: {e}")
            continue
    
    return msg

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
