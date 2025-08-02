import sys
import cv2 # type: ignore
import numpy as np # type: ignore

# Insert path to Racecar library
sys.path.insert(1, 'library')
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

# Create the Racecar object
rc = racecar_core.create_racecar()

# AR tag detection setup
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

class ARMarker:
    def __init__(self, marker_id, corners, orientation, area):
        self.id = marker_id
        self.corners = corners
        self.orientation = orientation
        self.area = area


def find_orientation(corners):
    # Determine orientation based on the first two corners
    if corners[0][0] == corners[1][0]:
        return "LEFT" if corners[0][1] > corners[1][1] else "RIGHT"
    else:
        return "DOWN" if corners[0][0] > corners[1][0] else "UP"


def find_area(corners):
    # Compute area from opposing corners
    return abs((corners[2][0] - corners[0][0]) * (corners[2][1] - corners[0][1]))


def detect_ar_tags(image):
    markers = []
    corners_list, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return markers
    for idx, corner_array in enumerate(corners_list):
        corners = corner_array[0]
        orientation = find_orientation(corners)
        area = find_area(corners)
        markers.append(ARMarker(ids[idx][0], corners, orientation, area))
    return markers


def start():
    # No initialization needed
    pass


def update():
    # Keep the car stationary
    rc.drive.set_speed_angle(0, 0)

    # Capture image and detect AR tags
    image = rc.camera.get_color_image()
    if image is None:
        return

    markers = detect_ar_tags(image)
    for m in markers:
        print(f"Marker ID: {m.id}, Size: {m.area}")


if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
