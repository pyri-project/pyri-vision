from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time

def vision_detect_aruco(aruco_dict, aruco_id, aruco_markersize, roi):
    device_manager = PyriSandboxContext.device_manager
    aruco_detection = device_manager.get_device_client("vision_aruco_detection", 1)
    res = aruco_detection.detect_aruco_camera_capture("camera","camera_calibration_intrinsic","camera_calibration_extrinsic",aruco_dict,aruco_id,aruco_markersize,roi)
    return res.detected_markers

def vision_aruco_detected_get_pose(detected_marker):
    if isinstance(detected_marker,list):
        detected_marker = detected_marker[0]
    return detected_marker.marker_pose.pose

def vision_aruco_detected_get_id(detected_marker):
    if isinstance(detected_marker,list):
        detected_marker = detected_marker[0]
    return detected_marker.marker_id

def _get_sandbox_functions():
    return {
        "vision_detect_aruco": vision_detect_aruco,
        "vision_aruco_detected_get_pose": vision_aruco_detected_get_pose,
        "vision_aruco_detected_get_id": vision_aruco_detected_get_id
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