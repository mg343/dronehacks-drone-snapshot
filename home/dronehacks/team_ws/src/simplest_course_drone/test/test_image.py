"""image.py: blank, dark and textured frames."""
import numpy as np

from simplest_course_drone.image import from_ros_image, seeing


def test_blank_grey_wall_has_no_texture():
    b, t = seeing(np.full((640, 1024), 128, np.uint8))
    assert b == 128.0 and t == 0.0


def test_dark_frame_is_dark():
    assert seeing(np.full((640, 1024), 4, np.uint8))[0] < 15


def test_checkerboard_is_textured():
    yy, xx = np.mgrid[0:640, 0:1024]
    board = (((yy // 32) + (xx // 32)) % 2 * 200).astype(np.uint8)
    assert seeing(board)[1] > 0.05


def test_ros_image_fields_with_row_padding():
    data = bytes(range(12)) * 2                 # 2 rows, step 12, width 10
    a = from_ros_image(2, 10, 12, 'mono8', data)
    assert a.shape == (2, 10) and a[1, 0] == 0
    assert from_ros_image(2, 10, 12, 'bgr8', data) is None
