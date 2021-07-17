import array
import base64
import io
import numpy as np

from rosboard.cv_bridge import imgmsg_to_cv2

try:
    import simplejpeg
except ImportError:
    simplejpeg = None
    try:
        import cv2
        print("simplejpeg not found. Falling back to cv2 for JPEG encoding.")
    except ImportError:
        cv2 = None
        try:
            import PIL
            from PIL import Image
            print("simplejpeg not found. Falling back to PIL for JPEG encoding.")
        except ImportError:
            PIL = None

def decode_jpeg(input_bytes):
    if simplejpeg:
        return simplejpeg.decode_jpeg(input_bytes)
    elif cv2:
        return cv2.imdecode(np.frombuffer(input_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)[:,:,::-1]
    elif PIL:
        return np.asarray(Image.open(io.BytesIO(input_bytes)))

def encode_jpeg(img):
    if simplejpeg:
        if len(img.shape) == 2:
            img = np.expand_dims(img, axis=2)
            if not img.flags['C_CONTIGUOUS']:
                img = img.copy(order='C')
            return simplejpeg.encode_jpeg(img, colorspace = "GRAY", quality = 50)
        elif len(img.shape) == 3:
            if not img.flags['C_CONTIGUOUS']:
                img = img.copy(order='C')
            if img.shape[2] == 1:
                return simplejpeg.encode_jpeg(img, colorspace = "GRAY", quality = 50)
            elif img.shape[2] == 4:
                return simplejpeg.encode_jpeg(img, colorspace = "RGBA", quality = 50)
            elif img.shape[2] == 3:
                return simplejpeg.encode_jpeg(img, colorspace = "RGB", quality = 50)
        else:
            return b''
    elif cv2:
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = img[:,:,::-1]
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()
    elif PIL:
        pil_img = Image.fromarray(img)
        buffered = io.BytesIO()
        pil_img.save(buffered, format="JPEG", quality = 50)    
        return buffered.getvalue()


def compress_compressed_image(msg, output):
    output["data"] = []

    if simplejpeg is None and cv2 is None and PIL is None:
        output["_error"] = "Please install simplejpeg, cv2 (OpenCV), or PIL (pillow) for image support."
        return

    # if message is already in jpeg format and small enough just pass it through
    if len(msg.data) < 250000 and msg.format == "jpeg":
        output["_img_jpeg"] = base64.b64encode(bytearray(msg.data)).decode()
        return
    
    # else recompress it
    img = decode_jpeg(bytearray(msg.data))
    img_jpeg = encode_jpeg(img)
    output["_img_jpeg"] = base64.b64encode(img_jpeg).decode()
            

def compress_image(msg, output):
    output["data"] = []

    if simplejpeg is None and cv2 is None and PIL is None:
        output["_error"] = "Please install simplejpeg, cv2 (OpenCV), or PIL (pillow) for image support."
        return

    cv2_img = imgmsg_to_cv2(msg, flip_channels = True)    

    # if image has alpha channel, cut it out since we will ultimately compress as jpeg
    if len(cv2_img.shape) == 3 and cv2_img.shape[2] == 4:
        cv2_img = cv2_img[:,:,0:3]

    # if image has only 2 channels, expand it to 3 channels for visualization
    # channel 0 -> R, channel 1 -> G, zeros -> B
    if len(cv2_img.shape) == 3 and cv2_img.shape[2] == 2:
        cv2_img = np.stack((cv2_img[:,:,0], cv2_img[:,:,1], np.zeros(cv2_img[:,:,0].shape)), axis = -1)

    # enforce <800px max dimension, and do a stride-based resize
    if cv2_img.shape[0] > 800 or cv2_img.shape[1] > 800:
        stride = int(np.ceil(max(cv2_img.shape[0] / 800.0, cv2_img.shape[1] / 800.0)))
        cv2_img = cv2_img[::stride,::stride]
    
    # if image format isn't already uint8, make it uint8 for visualization purposes
    if cv2_img.dtype != np.uint8:
        if cv2_img.dtype == np.uint64:
            # keep only the most significant 8 bits (0 to 255)
            cv2_img = (cv2_img >> 56).astype(np.uint8)
        elif cv2_img.dtype == np.uint32:
            # keep only the most significant 8 bits (0 to 255)
            cv2_img = (cv2_img >> 24).astype(np.uint8)
        elif cv2_img.dtype == np.uint16:
            # keep only the most significant 8 bits (0 to 255)
            cv2_img = (cv2_img >> 8).astype(np.uint8)
        elif cv2_img.dtype == np.float16 or cv2_img.dtype == np.float32 or cv2_img.dtype == np.float64:
            # map the float range (0 to 1) to uint8 range (0 to 255)
            cv2_img = np.clip(cv2_img * 255, 0, 255).astype(np.uint8)

    try:
        img_jpeg = encode_jpeg(cv2_img)
        output["_img_jpeg"] = base64.b64encode(img_jpeg).decode()
    except OSError as e:
        output["_error"] = str(e)    

def compress_occupancy_grid(msg, output):
    output["_data"] = []

    if simplejpeg is None and cv2 is None and PIL is None:
        output["_error"] = "Please install simplejpeg, cv2 (OpenCV), or PIL (pillow) for image support."
        return
    
    try:
        occupancy_map = np.array(msg.data, dtype=np.uint16).reshape(msg.info.height, msg.info.width)[::-1,:]

        while occupancy_map.shape[0] > 800 or occupancy_map.shape[1] > 800:
            occupancy_map = occupancy_map[::2,::2]

        cv2_img = ((100 - occupancy_map) * 10 // 4).astype(np.uint8) # *10//4 is int approx to *255.0/100.0
        cv2_img = np.stack((cv2_img,)*3, axis = -1) # greyscale to rgb
        cv2_img[occupancy_map < 0] = [255, 127, 0]
        cv2_img[occupancy_map > 100] = [255, 0, 0]

    except Exception as e:
        output["_error"] = str(e)
    try:
        img_jpeg = encode_jpeg(cv2_img)
        output["_img_jpeg"] = base64.b64encode(img_jpeg).decode()
    except OSError as e:
        output["_error"] = str(e)

DATATYPE_MAPPING_PCL2_NUMPY = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

def compress_point_cloud2(msg, output):
    # Lossy compress the point cloud by fetching only x,y,z fields and lowering their
    # precision to uint16. For a LIDAR that can see upto 100 meters, that is 3mm resolution.
    # Good enough for visualization.

    output["data"] = []

    if msg.point_step * msg.width * msg.height != len(msg.data):
        output["_error"] = "PointCloud2 error: msg.point_step * msg.width * msg.height == len(msg.data)"
        return

    fields_as_dict = {}
    np_struct = []
    used_bytes = 0

    for field in msg.fields:
        fields_as_dict[field.name] = field
        if field.datatype not in DATATYPE_MAPPING_PCL2_NUMPY:
            output["_error"] = "Bad point field type %d" % field.datatype
        np_datatype = DATATYPE_MAPPING_PCL2_NUMPY[field.datatype]
        np_struct.append((field.name, np_datatype))
        used_bytes += np.nbytes[np_datatype]

    if "x" not in fields_as_dict or "y" not in fields_as_dict:
        output["_error"] = "PointCloud2 error: must contain at least 'x' and 'y' fields for visualization"
        return
    
    if msg.point_step < used_bytes:
        output["_error"] = "PointCloud2 error: total byte sizes of fields exceeds point_step"
        return

    if msg.point_step > used_bytes:
        np_struct.append(('unused_bytes', np.uint8, msg.point_step - used_bytes))

    points = np.frombuffer(msg.data, dtype = np.uint8).view(dtype = np_struct)

    if points.size > 65536:
        output["_warn"] = "Point cloud too large, randomly subsampling to 65536 points."
        idx = np.random.randint(points.size, size=65536)
        points = points[idx]

    xpoints = points['x'].astype(np.float32)
    if msg.is_bigendian == np.little_endian:
        xpoints = xpoints.byteswap()
    xmax = np.max(xpoints)
    xmin = np.min(xpoints)
    if xmax - xmin < 1.0:
        xmax = xmin + 1.0
    xpoints_uint16 = (65535 * (xpoints - xmin) / (xmax - xmin)).astype(np.uint16)

    ypoints = points['y'].astype(np.float32)
    if msg.is_bigendian == np.little_endian:
        ypoints = ypoints.byteswap()
    ymax = np.max(ypoints)
    ymin = np.min(ypoints)
    if ymax - ymin < 1.0:
        ymax = ymin + 1.0
    ypoints_uint16 = (65535 * (ypoints - ymin) / (ymax - ymin)).astype(np.uint16)
    
    if "z" in fields_as_dict:
        zpoints = points['z'].astype(np.float32)
        if msg.is_bigendian == np.little_endian:
            zpoints = zpoints.byteswap()
        zmax = np.max(zpoints)
        zmin = np.min(zpoints)
        if zmax - zmin < 1.0:
            zmax = zmin + 1.0
        zpoints_uint16 = (65535 * (zpoints - zmin) / (zmax - zmin)).astype(np.uint16)
    else:
        zmax = 1.0
        zmin = 0.0
        zpoints_uint16 = ypoints_uint16 * 0
    
    bounds_uint16 = [xmin, xmax, ymin, ymax, zmin, zmax]
    if np.little_endian:
        points_uint16 = np.stack((xpoints_uint16, ypoints_uint16, zpoints_uint16),1).ravel().view(dtype=np.uint8)
    else:
        points_uint16 = np.stack((xpoints_uint16, ypoints_uint16, zpoints_uint16),1).ravel().byteswap().view(dtype=np.uint8)

    output["_data_uint16"] = {
        "type": "xyz",
        "bounds": list(map(float, bounds_uint16)),
        "points": base64.b64encode(points_uint16).decode(),
    }

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
            compress_compressed_image(msg, output)
            continue

        # Image: compress to jpeg
        if (msg.__module__ == "sensor_msgs.msg._Image" or \
            msg.__module__ == "sensor_msgs.msg._image") \
            and field == "data":
            compress_image(msg, output)
            continue

        # OccupancyGrid: render and compress to jpeg
        if (msg.__module__ == "nav_msgs.msg._OccupancyGrid" or \
            msg.__module__ == "nav_msgs.msg._occupancy_grid") \
            and field == "data":
            compress_occupancy_grid(msg, output)
            continue

        # LaserScan: strip to 3 decimal places (1mm precision) to reduce JSON size
        if (msg.__module__ == "sensor_msgs.msg._LaserScan" or \
            msg.__module__ == "sensor_msgs.msg._laser_scan") \
            and field == "ranges":
            output["ranges"] = list(map(lambda x: round(x, 3), msg.ranges))
            continue

        # PointCloud2: extract only necessary fields, reduce precision
        if (msg.__module__ == "sensor_msgs.msg._PointCloud2" or \
            msg.__module__ == "sensor_msgs.msg._point_cloud2") \
            and field == "data":
            compress_point_cloud2(msg, output)
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
