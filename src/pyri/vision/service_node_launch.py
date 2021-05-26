from pyri.plugins.service_node_launch import ServiceNodeLaunch, PyriServiceNodeLaunchFactory


launches = [
    ServiceNodeLaunch("vision_aruco_detection", "pyri.vision", "pyri.vision.aruco_detection_service",\
        default_devices=[("pyri_aruco_detection_service","vision_aruco_detection")]),
    ServiceNodeLaunch("vision_camera_calibration", "pyri.vision", "pyri.vision.camera_calibration_service",\
        default_devices=[("pyri_camera_calibration_service","vision_camera_calibration")]),
    ServiceNodeLaunch("vision_viewer_service", "pyri.vision", "pyri.vision.camera_viewer_service",\
        default_devices=[("pyri_camera_viewer_service","vision_camera_viewer")]),
    ServiceNodeLaunch("vision_robot_calibration", "pyri.vision", "pyri.vision.robot_calibration_service",\
        default_devices=[("pyri_vision_robot_calibration_service","vision_robot_calibration")]),
    ServiceNodeLaunch("vision_template_matching", "pyri.vision", "pyri.vision.template_matching",\
        default_devices=[("pyri_template_matching_service","vision_template_matching")]),
]

class VisionLaunchFactory(PyriServiceNodeLaunchFactory):
    def get_plugin_name(self):
        return "pyri.vision"

    def get_service_node_launch_names(self):
        return ["vision_aruco_detection"]

    def get_service_node_launches(self):
        return launches

def get_service_node_launch_factory():
    return VisionLaunchFactory()

        
