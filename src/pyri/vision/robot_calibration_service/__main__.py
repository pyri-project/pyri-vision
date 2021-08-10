# @burakaksoy plugin-cameraRobotCalibration-service.py

import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import argparse
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.IdentifierUtil import IdentifierUtil
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

import general_robotics_toolbox as rox
from . import opencv_aruco_extrinsic_calibration as calibrator

import cv2

from pyri.util.service_setup import PyriServiceNodeSetup


class CameraRobotCalibrationService_impl(object):
    def __init__(self, device_manager, device_info = None, node : RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info
                
        self.service_path = None
        self.ctx = None

        self.device_manager = device_manager
        self.device_manager.connect_device_type("tech.pyri.variable_storage.VariableStorage")
        self.device_manager.connect_device_type("com.robotraconteur.robotics.robot.Robot")
        self.device_manager.device_added += self._device_added
        self.device_manager.device_removed += self._device_removed
        self.device_manager.refresh_devices(5)

        self.robot_util = RobotUtil(self._node)
        self.geom_util = GeometryUtil(self._node)
        self.image_util = ImageUtil(self._node)
        
    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx
        
    def _device_added(self, local_device_name):
       pass 

    def _device_removed(self, local_device_name):
        pass

    def calibrate_robot_origin(self, robot_local_device_name, camera_intrinsic_calibration_global_name, camera_extrinsic_calibration_global_name, \
        image_sequence_global_name, aruco_dict, aruco_id, aruco_markersize, flange_to_marker, output_global_name):
        

        var_storage = self.device_manager.get_device_client("variable_storage",1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        if len(output_global_name) > 0:
            if len(var_storage.filter_variables("globals",output_global_name,[])) > 0:
                raise RR.InvalidOperationException(f"Global {output_global_name} already exists")

        image_sequence = []
        joint_pos_sequence = []

        image_sequence_vars = var_storage.getf_variable_value("globals",image_sequence_global_name)
        for image_var in image_sequence_vars.data.splitlines():
            var2 = var_storage.getf_variable_value("globals",image_var)
            image_sequence.append(self.image_util.compressed_image_to_array(var2.data))
            var2_tags = var_storage.getf_variable_attributes("globals", image_var)
            var2_state_var_name = var2_tags["system_state"]
            var2_state = var_storage.getf_variable_value("globals", var2_state_var_name)
            joint_pos = None 
            for s in var2_state.data.devices_states[robot_local_device_name].state:
                if s.type == "com.robotraconteur.robotics.robot.RobotState":
                    joint_pos = s.state_data.data.joint_position
            assert joint_pos is not None, "Could not find joint position in state sequence"
            joint_pos_sequence.append(joint_pos)

        cam_intrinsic_calib = var_storage.getf_variable_value("globals",camera_intrinsic_calibration_global_name).data
        cam_extrinsic_calib = var_storage.getf_variable_value("globals",camera_extrinsic_calibration_global_name).data

        mtx = cam_intrinsic_calib.K
        dist_rr = cam_intrinsic_calib.distortion_info.data
        dist = np.array([dist_rr.k1, dist_rr.k2, dist_rr.p1, dist_rr.p2, dist_rr.k3],dtype=np.float64)

        cam_pose = self.geom_util.named_pose_to_rox_transform(cam_extrinsic_calib.pose)

        robot = self.device_manager.get_device_client(robot_local_device_name,1)
        robot_info = robot.robot_info
        rox_robot = self.robot_util.robot_info_to_rox_robot(robot_info,0)
        

        # Calibrate
        robot_pose1, robot_pose_cov, img, calibration_error = calibrator.calibrate(image_sequence, joint_pos_sequence, aruco_dict, aruco_id, aruco_markersize, flange_to_marker, mtx, dist, cam_pose, rox_robot, robot_local_device_name)
        
        robot_pose = self._node.NewStructure("com.robotraconteur.geometry.NamedPoseWithCovariance")
        robot_pose.pose = robot_pose1
        robot_pose.covariance = robot_pose_cov

        if len(output_global_name) > 0:
            var_storage.add_variable2("globals",output_global_name,"com.robotraconteur.geometry.NamedPoseWithCovariance", \
                RR.VarValue(robot_pose,"com.robotraconteur.geometry.NamedPoseWithCovariance"), ["robot_origin_pose_calibration"], 
                {"device": robot_local_device_name}, variable_persistence["const"], None, variable_protection_level["read_write"], \
                [], f"Robot \"{robot_local_device_name}\" origin pose calibration", False)

        ret = RRN.NewStructure("tech.pyri.vision.robot_calibration.CameraRobotBaseCalibrateResult")
        ret.robot_pose = robot_pose
        ret.display_images = img
        ret.calibration_error = calibration_error

        return ret

def main():

    with PyriServiceNodeSetup("pyri.tech.vision.robot_calibration", 55918, \
        extra_service_defs=[(__package__,'tech.pyri.vision.robot_calibration.robdef')], \
        default_info=(__package__,"pyri_vision_robot_calibration_service_default_info.yml"), \
        display_description="PyRI Vision Robot Calibration Service", device_manager_autoconnect=False, \
        distribution_name="pyri-vision") as service_node_setup:

          # create object
        CameraRobotCalibrationService_inst = CameraRobotCalibrationService_impl(service_node_setup.device_manager, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "CameraCalibration", type "experimental.pluginCameraCalibration.CameraCalibration", actual object: CameraCalibration_inst
        service_node_setup.register_service("camera_robot_calibration","tech.pyri.vision.robot_calibration.CameraRobotCalibrationService",CameraRobotCalibrationService_inst)
        
        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()