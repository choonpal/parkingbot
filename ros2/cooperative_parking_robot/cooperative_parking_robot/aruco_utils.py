#!/usr/bin/env python3
"""ArUco geometry and OpenCV-version compatibility helpers."""

import math


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_yaw_from_rotation(rotation):
    """Return robot heading from an OpenCV object-to-camera rotation.

    The rear marker's local +Z normal points toward the camera, while the
    front robot heading points away from it.  The desired heading is therefore
    the negative marker normal projected onto the camera X-Z plane.
    """
    return normalize_angle(math.atan2(-float(rotation[0][2]),
                                      -float(rotation[2][2])))


def apply_relative_pose_alignment(forward, lateral, yaw, *,
                                  lateral_offset_m=0.0,
                                  yaw_offset_rad=0.0):
    """Apply measured Rear-camera mounting offsets to an ID0 planar pose."""
    values = tuple(float(value) for value in (
        forward, lateral, yaw, lateral_offset_m, yaw_offset_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('relative-pose alignment values must be finite')
    return (
        values[0],
        values[1] + values[3],
        normalize_angle(values[2] + values[4]),
    )


def marker_center_to_base_link(marker_x, marker_y, yaw, offset_x):
    """Shift a marker-center world pose to the robot rotation centre (base_link).

    The marker is mounted away from the robot centre along the robot's own
    +x (heading) axis by ``offset_x`` metres.  Downstream nodes
    (``rigid_body_sync_node``, the pair-centre and self-mask math) treat
    ``/{role}/odom`` as the robot centre, so the localization must subtract this
    body-frame offset, rotated into the world frame by the measured ``yaw``.

    Sign convention: ``offset_x`` is the marker position in the robot body
    frame.  A front-edge marker (ahead of centre) uses a positive value; a
    rear-edge marker (behind centre) uses a negative value.  ``offset_x == 0``
    reproduces the legacy behaviour where the marker sits at the robot centre.
    """
    base_x = marker_x - offset_x * math.cos(yaw)
    base_y = marker_y - offset_x * math.sin(yaw)
    return base_x, base_y


class ArucoDetectorCompat:
    """Use either OpenCV's new ArucoDetector API or the legacy OpenCV 4.x API."""

    def __init__(self, cv2_module, dictionary_name,
                 min_marker_distance_rate=None):
        self._aruco = getattr(cv2_module, 'aruco', None)
        if self._aruco is None:
            raise RuntimeError(
                'OpenCV ArUco module missing; install python3-opencv or '
                'opencv-contrib-python')
        if not hasattr(self._aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')

        dictionary_id = getattr(self._aruco, dictionary_name)
        self.dictionary = self._aruco.getPredefinedDictionary(dictionary_id)

        detector_type = getattr(self._aruco, 'ArucoDetector', None)
        if detector_type is not None:
            if not hasattr(self._aruco, 'DetectorParameters'):
                raise RuntimeError(
                    'OpenCV ArUco DetectorParameters API missing')
            self.parameters = self._aruco.DetectorParameters()
            self._detector = None
        else:
            # OpenCV 4.6 exposes both constructors, but its Python binding
            # segfaults inside legacy detectMarkers() when parameters came
            # from DetectorParameters(). The matching legacy factory is safe.
            if hasattr(self._aruco, 'DetectorParameters_create'):
                self.parameters = self._aruco.DetectorParameters_create()
            elif hasattr(self._aruco, 'DetectorParameters'):
                self.parameters = self._aruco.DetectorParameters()
            else:
                raise RuntimeError(
                    'OpenCV ArUco DetectorParameters API missing')
            self._detector = None

        if min_marker_distance_rate is not None:
            rate = float(min_marker_distance_rate)
            if not 0.0 < rate <= 1.0:
                raise ValueError(
                    'min_marker_distance_rate must be in (0, 1]')
            if not hasattr(self.parameters, 'minMarkerDistanceRate'):
                raise RuntimeError(
                    'OpenCV ArUco minMarkerDistanceRate API missing')
            self.parameters.minMarkerDistanceRate = rate

        # New OpenCV copies DetectorParameters into ArucoDetector during
        # construction, so apply overrides before constructing it.
        if detector_type is not None:
            self._detector = detector_type(
                self.dictionary, self.parameters)
        if self._detector is None and not hasattr(self._aruco, 'detectMarkers'):
            raise RuntimeError('No supported OpenCV ArUco detector API found')

    def detect_markers(self, image):
        if self._detector is not None:
            return self._detector.detectMarkers(image)
        return self._aruco.detectMarkers(
            image, self.dictionary, parameters=self.parameters)
