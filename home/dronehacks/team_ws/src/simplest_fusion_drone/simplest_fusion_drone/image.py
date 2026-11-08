"""Can the camera see? Two numbers from one mono8 image. Pure numpy, testable.

brightness   mean pixel value, 0-255. The dark tunnel drives it down.
texture      fraction of pixels whose gradient is strong. A blank grey wall drives it to ~0, which
             is exactly when cuVSLAM reports confident nonsense (field guide: the guard withholds
             frames under 1% texture).

Computed on every 4th pixel each way (1024x640 -> 256x160) to stay cheap next to cuVSLAM.
"""
import numpy as np

STEP = 4
EDGE = 12          # gradient (grey levels between neighbouring sampled pixels) that counts as texture


def seeing(pixels):
    """(brightness, texture) of a 2-D uint8 array."""
    a = np.asarray(pixels, dtype=np.int16)[::STEP, ::STEP]
    if a.size == 0:
        return 0.0, 0.0
    gx = np.abs(np.diff(a, axis=1))[:-1, :]
    gy = np.abs(np.diff(a, axis=0))[:, :-1]
    strong = np.maximum(gx, gy) >= EDGE
    return float(a.mean()), float(strong.mean()) if strong.size else 0.0


def from_ros_image(height, width, step, encoding, data):
    """A sensor_msgs/Image's fields -> a 2-D uint8 array (mono8 only), or None."""
    if encoding != 'mono8' or height * step > len(data):
        return None
    return np.frombuffer(bytes(data), dtype=np.uint8, count=height * step).reshape(height, step)[:, :width]
