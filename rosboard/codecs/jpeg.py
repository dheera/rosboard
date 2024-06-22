import io
import numpy as np

# ROSboard supports any ONE of the following 4 libraries for JPEG encoding and decoding:
# * simplejpeg (pip3 install simplejpeg; not available in apt)
# * TurboJPEG (apt install libturbojpeg)
# * OpenCV (pip3 install opencv-python OR apt install python3-opencv)
# * PIL (pip3 install pillow OR apt install python3-pil)

# simplejpeg and TurboJPEG are SIMD-optimized and highly performant, and are preferred. 

# OpenCV and PIL are slow, but are often already found on existing systems,
# so we support them as well, but there will be a significant performance hit.

LIBRARY = None
LIBRARY_SIMPLEJPEG = 1
LIBRARY_TURBOJPEG = 2
LIBRARY_OPENCV = 3
LIBRARY_PIL = 4

try:
    import simplejpeg
    LIBRARY = LIBRARY_SIMPLEJPEG
except ImportError:
    simplejpeg = None
    try: # try harder
        from .turbojpeg import TurboJPEG # libturbojpeg wrapper

        # If libturbojpeg is not installed, this wrapper will not fail on import,
        # it will only fail when attempting to initialize the class:
        turbojpeg = TurboJPEG()

        # This wrapper is not necessarily future-proof so let's run a test just in case
        # If this wrapper doesn't work, fail early so that it triggers a fallback to cv2 or PIL
        output = turbojpeg.encode(np.ones([5,5,3]))
        assert(type(output)) is bytes and len(output) > 0
        LIBRARY = LIBRARY_TURBOJPEG
    except:
        turbojpeg = None
        try: # try super hard
            import cv2
            print("Using cv2 for JPEG encoding. Consider installing either one of the following for faster performance:\n")
            print(" * simplejpeg (pip3 install simplejpeg)\n")
            print(" * libturbojpeg (sudo apt install libturbojpeg)\n")
            LIBRARY = LIBRARY_OPENCV
        except ImportError:
            cv2 = None
            try: # try super super hard
                from PIL import Image
                print("Using PIL for JPEG encoding. Consider installing either one of the following for faster performance:\n")
                print(" * simplejpeg (pip3 install simplejpeg)\n")
                print(" * libturbojpeg (sudo apt install libturbojpeg)\n")
                LIBRARY = LIBRARY_PIL
            except ImportError:
                LIBRARY = None

def decode_jpeg(input_bytes):
    if LIBRARY == LIBRARY_SIMPLEJPEG:
        return simplejpeg.decode_jpeg(input_bytes)
    elif LIBRARY == LIBRARY_TURBOJPEG:
        return turbojpeg.decode(input_bytes)
    elif LIBRARY == LIBRARY_OPENCV:
        return cv2.imdecode(np.frombuffer(input_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)[:,:,::-1]
    elif LIBRARY == LIBRARY_PIL:
        return np.asarray(Image.open(io.BytesIO(input_bytes)))
    else:
        raise RuntimeError("Please install simplejpeg, libturbojpeg, cv2 (OpenCV), or PIL (pillow) for image support.")

def encode_jpeg(img):
    if len(img.shape) == 2:
        img = np.expand_dims(img, axis=2)
    if img.dtype == np.uint16:
        img = (img >> 8).astype(np.uint8)

    if LIBRARY == LIBRARY_SIMPLEJPEG:
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
    elif LIBRARY == LIBRARY_TURBOJPEG:
        if img.shape[2] == 1:
            # TurboJPEG doesn't support 1-channel images
            img = np.dstack((img, img, img))
        if img.shape[2] == 4:
            # TurboJPEG doesn't support 4-channel images
            img = img[:,:,0:3]
        return turbojpeg.encode(img, quality = 50)
    elif LIBRARY == LIBRARY_OPENCV:
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = img[:,:,::-1]
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()
    elif LIBRARY == LIBRARY_PIL:
        if img.shape[2] == 1:
            # PIL doesn't support 1-channel images
            img = np.dstack((img, img, img))
        if img.shape[2] == 4:
            # PIL doesn't support 4-channel images
            img = img[:,:,0:3]
        pil_img = Image.fromarray(img)
        buffered = io.BytesIO()
        pil_img.save(buffered, format="JPEG", quality = 50)    
        return buffered.getvalue()
    else:
        raise RuntimeError("Please install simplejpeg, libturbojpeg, cv2 (OpenCV), or PIL (pillow) for image support.")

def unit_test():
    global LIBRARY
    global simplejpeg, turbojpeg, cv2, Image

    import traceback
    print("Importing all modules for unit test")

    import simplejpeg
    from .turbojpeg import TurboJPEG
    turbojpeg = TurboJPEG()
    import cv2
    from PIL import Image

    images = {
        "mono8_2": np.ones((32,32), dtype = np.uint8),
        "mono8_3": np.ones((32,32,1), dtype = np.uint8),
        "mono16_2": np.ones((32,32), dtype = np.uint16),
        "mono16_3": np.ones((32,32,1), dtype = np.uint16),
        "rgb8": np.ones((32,32,3), dtype = np.uint8),
        "rgba8": np.ones((32,32,4), dtype = np.uint8),
        "non_c_contiguous": np.ones((32,32,3), dtype=np.uint8)[::2,::2,:],
    }

    success = True

    for library in [LIBRARY_SIMPLEJPEG, LIBRARY_TURBOJPEG, LIBRARY_OPENCV, LIBRARY_PIL]:
        LIBRARY = library
        for image_type in images:
            try:
                encoded = encode_jpeg(images[image_type])
                decoded = decode_jpeg(encoded)
                assert(decoded.shape[0:2] == images[image_type].shape[0:2])
            except Exception as e:
                print(f"Test failed for library={library} image_type={image_type}:")
                traceback.print_exc()
                success = False
                print("\n\n")

    if success:
        print("All tests succeeded")

if __name__ == "__main__":
    unit_test()

