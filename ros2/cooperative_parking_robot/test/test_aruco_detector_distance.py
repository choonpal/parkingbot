import cv2
import numpy as np
import pytest

from cooperative_parking_robot.aruco_utils import ArucoDetectorCompat


def _nested_id0_image():
    """Model the real black robot, white board, and inset black marker."""
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV ArUco module is not installed')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.zeros((400, 400), dtype=np.uint8)
    image[89:311, 89:311] = 255
    marker = np.zeros((182, 182), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 182, marker, 1)
    else:
        cv2.aruco.drawMarker(dictionary, 0, 182, marker, 1)
    image[109:291, 109:291] = marker
    return image


def _ids(detector, image):
    _, ids, _ = detector.detect_markers(image)
    return [] if ids is None else ids.flatten().tolist()


def test_lower_distance_rate_preserves_id0_inside_close_board_edge():
    image = _nested_id0_image()
    detector = ArucoDetectorCompat(
        cv2, 'DICT_4X4_50', min_marker_distance_rate=0.02)
    assert _ids(detector, image) == [0]
    assert detector.parameters.minMarkerDistanceRate == pytest.approx(0.02)


@pytest.mark.parametrize('invalid', [0.0, -0.01, 1.01])
def test_invalid_min_marker_distance_rate_is_rejected(invalid):
    with pytest.raises(ValueError, match='min_marker_distance_rate'):
        ArucoDetectorCompat(
            cv2, 'DICT_4X4_50', min_marker_distance_rate=invalid)
