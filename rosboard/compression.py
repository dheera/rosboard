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

_PCL2_DATATYPES_NUMPY_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

def decode_pcl2(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: empty list)
    :return: numpy.recarray with values for each point.
    """

    assert cloud.point_step * cloud.width * cloud.height == len(cloud.data), \
        'length of data does not match point_step * width * height'

    all_field_names = []
    np_struct = []
    total_used_bytes = 0
    for field in cloud.fields:
        all_field_names.append(field.name)
        assert field.datatype in _PCL2_DATATYPES_NUMPY_MAP, \
            'invalid datatype %d specified for field %s' % (field.datatype, field.name)
        field_np_datatype = _PCL2_DATATYPES_NUMPY_MAP[field.datatype]
        np_struct.append((field.name, field_np_datatype))
        total_used_bytes += np.nbytes[field_np_datatype]

    assert cloud.point_step >= total_used_bytes, \
        'error: total byte sizes of fields exceeds point_step'

    if cloud.point_step > total_used_bytes:
        np_struct.append(('unused_bytes', np.uint8, cloud.point_step - total_used_bytes))

    points = np.frombuffer(cloud.data, dtype=np_struct).view(dtype=np.recarray)

    if skip_nans:
        nan_indexes = None
        for field_name in all_field_names:
            if nan_indexes is None:
                nan_indexes = np.isnan(points[field_name])
            else:
                nan_indexes = nan_indexes | np.isnan(points[field_name])

        points = points[~nan_indexes]

    if uvs:
        fetch_indexes = [(v * cloud.width + u) for u, v in uvs]
        points = points[fetch_indexes]

    # if endianness between cloud and system doesn't match then byteswap everything
    if cloud.is_bigendian == np.little_endian:
        points = points.byteswap()

    if field_names is None:
        return points
    else:
        return points[list(field_names)]

def compress_compressed_image(msg, output):
    output["data"] = []
    output["__comp"] = ["data"]

    if simplejpeg is None and cv2 is None and PIL is None:
        output["_error"] = "Please install simplejpeg, cv2 (OpenCV), or PIL (pillow) for image support."
        return

    # if message is already in jpeg format and small enough just pass it through
    if len(msg.data) < 250000 and "jpeg" in msg.format:
        output["_data_jpeg"] = base64.b64encode(bytearray(msg.data)).decode()
        return
    
    # else recompress it
    try:
        img = decode_jpeg(bytearray(msg.data))
        original_shape = img.shape
        if img.shape[0] > 800 or img.shape[1] > 800:
            stride = int(np.ceil(max(img.shape[0] / 800.0, img.shape[1] / 800.0)))
            img = img[::stride,::stride]
        img_jpeg = encode_jpeg(img)
    except Exception as e:
        output["_error"] = "Error: %s" % str(e)
        return
    output["_data_jpeg"] = base64.b64encode(img_jpeg).decode()
    output["_data_shape"] = list(original_shape)
            

def compress_image(msg, output):
    output["data"] = []
    output["__comp"] = ["data"]

    if simplejpeg is None and cv2 is None and PIL is None:
        output["_error"] = "Please install simplejpeg, cv2 (OpenCV), or PIL (pillow) for image support."
        return

    cv2_img = imgmsg_to_cv2(msg, flip_channels = True)    
    original_shape = cv2_img.shape

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
        output["_data_jpeg"] = base64.b64encode(img_jpeg).decode()
        output["_data_shape"] = original_shape
    except OSError as e:
        output["_error"] = str(e)    

def compress_occupancy_grid(msg, output):
    output["_data"] = []
    output["__comp"] = ["data"]

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
        output["_data_jpeg"] = base64.b64encode(img_jpeg).decode()
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
    # assuming fields are ('x', 'y', 'z', ...),
    # compression scheme is:
    # msg['_data_uint16'] = {
    #   bounds: [ xmin, xmax, ymin, ymax, zmin, zmax, ... ]
    #   points: string: base64 encoded bytearray of struct { uint16 x_frac; uint16 y_frac; uint16 z_frac;}
    # }
    # where x_frac = 0 maps to xmin and x_frac = 65535 maps to xmax
    # i.e. we are encoding all the floats as uint16 values where 0 represents the min value in the entire dataset and
    # 65535 represents the max value in the dataset, and bounds: [...] holds information on those bounds so the
    # client can decode back to a float

    output["data"] = []
    output["__comp"] = ["data"]

    field_names = [field.name for field in msg.fields]

    if "x" not in field_names or "y" not in field_names:
        output["_error"] = "PointCloud2 error: must contain at least 'x' and 'y' fields for visualization"
        return

    if "z" in field_names:
        decode_fields = ("x", "y", "z")
    else:
        decode_fields = ("x", "y")
    
    try:
        points = decode_pcl2(msg, field_names = decode_fields, skip_nans = True)
    except AssertionError as e:
        output["_error"] = "PointCloud2 error: %s" % str(e)
    
    if points.size > 65536:
        output["_warn"] = "Point cloud too large, randomly subsampling to 65536 points."
        idx = np.random.randint(points.size, size=65536)
        points = points[idx]

    xpoints = points['x'].astype(np.float32)
    xmax = np.max(xpoints)
    xmin = np.min(xpoints)
    if xmax - xmin < 1.0:
        xmax = xmin + 1.0
    xpoints_uint16 = (65535 * (xpoints - xmin) / (xmax - xmin)).astype(np.uint16)

    ypoints = points['y'].astype(np.float32)
    ymax = np.max(ypoints)
    ymin = np.min(ypoints)
    if ymax - ymin < 1.0:
        ymax = ymin + 1.0
    ypoints_uint16 = (65535 * (ypoints - ymin) / (ymax - ymin)).astype(np.uint16)
    
    if "z" in field_names:
        zpoints = points['z'].astype(np.float32)
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


def compress_laser_scan(msg, output):
    # compression scheme:
    # map ranges to _ranges_uint16 in the following format:
    # msg['_ranges_uint16'] = {
    #   bounds: [ range_min, range_max ] (exclude nan/inf/-inf in min/max computation)
    #   points: string: base64 encoded bytearray of struct uint16 r_frac
    # }
    # where r_frac = 0 reprents the minimum range, r_frac = 65534 maps to the max range, 65535 is invalid value (nan/-inf/inf)
    # msg['_intensities_uint16'] = {
    #   ... same as above but for intensities ...
    # }
    # then set msg['ranges'] = [] and msg['intensities'] = []

    output["ranges"] = []
    output["intensities"] = []
    output["__comp"] = ["ranges", "intensities"]

    rpoints = np.array(msg.ranges, dtype = np.float32)
    ipoints = np.array(msg.intensities, dtype = np.float32)

    if len(ipoints) > 0 and len(ipoints) != len(rpoints):
        output["_error"] = "LaserScan error: intensities must be empty or equal in size to ranges"
        return

    bad_indexes = np.isnan(rpoints) | np.isinf(rpoints)
    rpoints[bad_indexes] = 0.0
    rpoints_good = rpoints[~bad_indexes]

    if len(rpoints_good) > 0:
        rmax = np.max(rpoints_good)
        rmin = np.min(rpoints_good)
        if rmax - rmin < 1.0:
            rmax = rmin + 1.0

        rpoints_uint16 = (65534 * (rpoints - rmin) / (rmax - rmin)).astype(np.uint16)
        rpoints_uint16[bad_indexes] = 65535
    else:
        rmax = 1.0
        rmin = 0.0
        rpoints_uint16 = 65535 * np.ones(rpoints.shape, dtype = np.uint16)

    if len(ipoints) > 0:
        imax = np.max(ipoints)
        imin = np.min(ipoints)
        if np.isnan(imax) or np.isinf(imax) or np.isnan(imin) or np.isinf(imin):
            imax = 1000.0
            imin = 0.0
        if imax - imin < 1.0:
            imax = imin + 1.0
        ipoints_uint16 = (65534 * (ipoints - imin) / (imax - imin)).astype(np.uint16)
        ipoints_uint16[bad_indexes] = 65535
    else:
        imax = 1.0
        imin = 0.0
        ipoints_uint16 = np.array([], dtype=np.uint16)

    if not np.little_endian:
        rpoints_uint16 = rpoints_uint16.byteswap()
        ipoints_uint16 = ipoints_uint16.byteswap()

    output["_ranges_uint16"] = {
        "type": "r",
        "bounds": [float(rmin), float(rmax)],
        "points": base64.b64encode(rpoints_uint16).decode(),
    }

    output["_intensities_uint16"] = {
        "type": "i",
        "bounds": [float(imin), float(imax)],
        "points": base64.b64encode(ipoints_uint16).decode(),
    }
