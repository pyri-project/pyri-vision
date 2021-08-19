from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time

def vision_detect_aruco(aruco_dict, aruco_id, aruco_markersize, roi):
    """
    Detect ArUco markers using the OpenCV. This function returns the X, Y, Z, and Yaw
    components of the marker pose. Due to the low accuracy of Roll and Pitch, this
    function does not attempt to estimate rotation other than Yaw.

    Returns a list of `DetectedMarkers` structures. Use `vision_aruco_detected_get_pose()`
    and `vision_aruco_detected_get_id()` to retrieve the pose and ids of detected markers.

    Parameters:

    * aruco_dict (str): The ArUco dictionary to use. For example, DICT_6X6_250
    * aruco_id (int): The ArUco ID to detect. -1 for all
    * aruco_markersize (float): The size of the marker, in meters
    * roi (BoundingBox2D): The region of interest. None (Blockly null) for full image. Use
      ROI variable editor to create ROI global variables.

    Return (List[DetectedMarkers]): The detected markers
    """

    device_manager = PyriSandboxContext.device_manager
    aruco_detection = device_manager.get_device_client("vision_aruco_detection", 1)
    res = aruco_detection.detect_aruco_camera_capture("camera","camera_calibration_intrinsic","camera_calibration_extrinsic",aruco_dict,aruco_id,aruco_markersize,roi)
    return res.detected_markers

def vision_aruco_detected_get_pose(detected_marker):
    """
    Get the pose of a previously detected ArUco marker. Use list operations
    to get specific entries, otherwise returns value of first entry.

    Parameters:

    detected_marker (List[DectedMarker] or DetectedMarker): The detected markers

    Return (Pose): The pose of the detected marker
    """

    if isinstance(detected_marker,list):
        detected_marker = detected_marker[0]
    return detected_marker.marker_pose.pose

def vision_aruco_detected_get_id(detected_marker):
    """
    Get the id of a previously detected ArUco marker. Use list operations
    to get specific entries, otherwise returns value of first entry.

    Parameters:

    detected_marker (List[DectedMarker] or DetectedMarker): The detected markers

    Return (int): The id of the detected marker
    """
    if isinstance(detected_marker,list):
        detected_marker = detected_marker[0]
    return detected_marker.marker_id

def vision_template_match(template, object_z, roi):
    """
    Use planar cross-correlation image template matching to detect a known object.
    Image templates are stored as global variables, and created using 
    the "New Variable" button in the global variable table. Select a previously captured
    image, and crop to the desired object to detect. This function takes a specified
    global variable name containing a template, and detects it in the current camera image.
    This is a planar operation. `object_z` is used to specify the height of the object
    off the world XY surface. The world frame must be aligned with the table surface.

    Returns a list of `MatchedTemplate3D` structures. Use `vision_matched_template_get_pose()`
    to retrieve the pose of detected objects.

    Parameters:

    * template (str): The name of the global variable containing the image template
    * object_z (float): The Z offset of the object from world Z=0
    * roi (BoundingBox2D): The region of interest. None (Blockly null) for full image. Use
      ROI variable editor to create ROI global variables.

    Return (List[MatchedTemplate3D]): The detected markers
    """

    device_manager = PyriSandboxContext.device_manager
    template_matching = device_manager.get_device_client("vision_template_matching", 1)
    res = template_matching.match_template_world_pose_camera_capture("camera",template, "camera_calibration_intrinsic","camera_calibration_extrinsic",object_z,roi)
    return res.template_matches

def vision_matched_template_get_pose(template_match):
    """
    Get the pose of a previously detected template match. Use list operations
    to get specific entries, otherwise returns value of first entry.

    Parameters:

    template_match (List[MatchedTemplate3D] or MatchedTemplate3D): The template match(s)

    Return (Pose): The pose of the template match
    """
    if isinstance(template_match,list):
        template_match = template_match[0]
    return template_match.pose.pose.pose

def _get_sandbox_functions():
    return {
        "vision_detect_aruco": vision_detect_aruco,
        "vision_aruco_detected_get_pose": vision_aruco_detected_get_pose,
        "vision_aruco_detected_get_id": vision_aruco_detected_get_id,
        "vision_template_match": vision_template_match,
        "vision_matched_template_get_pose": vision_matched_template_get_pose

    }

class VisionSandboxFunctionsPluginFactory(PyriSandboxFunctionsPluginFactory):
    def get_plugin_name(self):
        return "pyri-vision"

    def get_sandbox_function_names(self):
        return list(_get_sandbox_functions().keys())

    def get_sandbox_functions(self):
        return _get_sandbox_functions()


def get_sandbox_functions_factory():
    return VisionSandboxFunctionsPluginFactory()