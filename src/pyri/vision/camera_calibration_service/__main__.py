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

import time
import threading
import traceback

import general_robotics_toolbox as rox

import cv2
from cv2 import aruco

from pyri.util.service_setup import PyriServiceNodeSetup

def _cv_img_to_rr_display_img(img):
    height,width = img.shape[0:2]
    
    s = 1

    # Clamp image to 720p
    s1 = 1280.0/width
    s2 = 720.0/height
    s = min(s1,s2)
    if s < 1.0:
        width = int(width*s)
        height = int(height*s)
        img = cv2.resize(img,(width,height))    

    img_util = ImageUtil()

    return img_util.array_to_compressed_image_jpg(img,70)

def _calibrate_camera_intrinsic2(images,board):
    # opencv_camera_calibration.py:6

    # TODO: Don't hardcode the calibration pattern
    if board == "chessboard":
        width = 7
        height = 6
        square_size=0.03
    else:
        raise RR.InvalidOperationException(f"Invalid calibration board {board}")

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    imgs = []

    image_util = ImageUtil()

    for image in images:
                
        frame = image_util.compressed_image_to_array(image)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(frame, (width, height), corners2, ret)
            imgs.append(img)
            # cv2.imshow("img", img)
            # cv2.waitKey(1000)

    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )

    return ret, mtx, dist.flatten(), rvecs, tvecs, mean_error, imgs

def _calibrate_camera_intrinsic(images, calibration_target):    
    ret, mtx, dist, rvecs, tvecs, mean_error, imgs = _calibrate_camera_intrinsic2(images,calibration_target)
    if not ret:
        raise RR.OperationFailedException("Camera intrinsic calibration failed")

    geom_util = GeometryUtil()

    calib = RRN.NewStructure("com.robotraconteur.imaging.camerainfo.CameraCalibration")
    calib.image_size = geom_util.wh_to_size2d([images[0].image_info.width,images[0].image_info.height],dtype=np.int32)
    
    calib.K = mtx

    dist_rr = RRN.NewStructure("com.robotraconteur.imaging.camerainfo.PlumbBobDistortionInfo")
    dist_rr.k1 = dist[0]
    dist_rr.k2 = dist[1]
    dist_rr.p1 = dist[2]
    dist_rr.p2 = dist[3]
    dist_rr.k3 = dist[4]

    calib.distortion_info = RR.VarValue(dist_rr,"com.robotraconteur.imaging.camerainfo.PlumbBobDistortionInfo")

    imgs2 = []
    for img in imgs:
        imgs2.append(_cv_img_to_rr_display_img(img))

    return calib, imgs2, mean_error

def _calibrate_camera_extrinsic(intrinsic_calib, image, board, camera_local_device_name):

    # TODO: verify calibration data

    mtx = intrinsic_calib.K
    dist_rr = intrinsic_calib.distortion_info.data
    dist = np.array([dist_rr.k1, dist_rr.k2, dist_rr.p1, dist_rr.p2, dist_rr.k3],dtype=np.float64)

    image_util = ImageUtil()
    frame = image_util.compressed_image_to_array(image)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    if board == "chessboard":
        width = 7
        height = 6
        square_size=0.03
    else:
        raise RR.InvalidOperationException(f"Invalid calibration board {board}")

    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
    assert ret, "Could not find calibration target"

    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

    ret,rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)

    R = cv2.Rodrigues(rvecs.flatten())[0]

    R_landmark = np.array([[0,1,0],[1,0,0],[0,0,-1]],dtype=np.float64)

    R_cam1 = R.transpose()
    p_cam1 = -R.transpose() @ tvecs

    R_cam = R_landmark.transpose() @ R_cam1
    p_cam = R_landmark.transpose() @ p_cam1

    cv_image2 = cv2.aruco.drawAxis(frame,mtx,dist,cv2.Rodrigues(R_cam.transpose())[0],-R_cam.transpose()@p_cam,0.1)

    T = rox.Transform(R_cam,p_cam,"world",camera_local_device_name)

    geom_util = GeometryUtil()
    cov = np.eye(6) * 1e-5
    return geom_util.rox_transform_to_named_pose(T), cov, image_util.array_to_compressed_image_jpg(cv_image2), 0.0
class CameraCalibrationService_impl:
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
        self.device_manager.device_added += self._device_added
        self.device_manager.device_removed += self._device_removed
        self.device_manager.refresh_devices(5)

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx
        
    def _device_added(self, local_device_name):
       pass 

    def _device_removed(self, local_device_name):
        pass
    
    def calibrate_camera_intrinsic(self, camera_local_device_name, image_sequence_global_name, calibration_target,
        output_global_name):
        
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        if len(output_global_name) > 0:
            if len(var_storage.filter_variables("globals",output_global_name,[])) > 0:
                raise RR.InvalidOperationException(f"Global {output_global_name} already exists")

        image_sequence = []

        image_sequence_vars = var_storage.getf_variable_value("globals",image_sequence_global_name)
        for image_var in image_sequence_vars.data.splitlines():
            var2 = var_storage.getf_variable_value("globals",image_var)
            image_sequence.append(var2.data)

        camera_calib, imgs, calibration_error = _calibrate_camera_intrinsic(image_sequence, calibration_target)

        if len(output_global_name) > 0:
            var_storage.add_variable2("globals",output_global_name,"com.robotraconteur.imaging.camerainfo.CameraCalibration", \
                RR.VarValue(camera_calib,"com.robotraconteur.imaging.camerainfo.CameraCalibration"), ["camera_calibration_intrinsic"], 
                {"device": camera_local_device_name}, variable_persistence["const"], None, variable_protection_level["read_write"], \
                [], f"Camera \"{camera_local_device_name}\" intrinsic calibration", False)

        ret = RRN.NewStructure("tech.pyri.vision.calibration.CameraCalibrateIntrinsicResult")
        ret.calibration = camera_calib
        ret.display_images = imgs
        ret.calibration_error = calibration_error

        return ret

    def calibrate_camera_extrinsic(self, camera_local_device_name, camera_intrinsic_calibration_global_name,
        image_global_name, origin_calibration_target, output_global_name):
        
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        if len(output_global_name) > 0:
            if len(var_storage.filter_variables("globals",output_global_name,[])) > 0:
                raise RR.InvalidOperationException(f"Global {output_global_name} already exists")

        image = var_storage.getf_variable_value("globals",image_global_name).data
        intrinsic_calib = var_storage.getf_variable_value("globals",camera_intrinsic_calibration_global_name).data

        camera_pose1, camera_pose_cov, img, calibration_error = _calibrate_camera_extrinsic(intrinsic_calib, image, origin_calibration_target, camera_local_device_name)

        camera_pose = self._node.NewStructure("com.robotraconteur.geometry.NamedPoseWithCovariance")
        camera_pose.pose = camera_pose1
        camera_pose.covariance = camera_pose_cov
        
        if len(output_global_name) > 0:
            var_storage.add_variable2("globals",output_global_name,"com.robotraconteur.geometry.NamedPoseWithCovariance", \
                RR.VarValue(camera_pose,"com.robotraconteur.geometry.NamedPoseWithCovariance"), ["camera_calibration_extrinsic"], 
                {"device": camera_local_device_name}, variable_persistence["const"], None, variable_protection_level["read_write"], \
                [], f"Camera \"{camera_local_device_name}\" extrinsic calibration", False)

        ret = RRN.NewStructure("tech.pyri.vision.calibration.CameraCalibrateExtrinsicResult")
        ret.camera_pose = camera_pose
        ret.display_image = img
        ret.calibration_error = calibration_error

        return ret

def main():
        
    with PyriServiceNodeSetup("tech.pyri.vision.camera_calibration", 55917, \
        extra_service_defs=[(__package__,'tech.pyri.vision.calibration.robdef')], \
        default_info=(__package__,"pyri_vision_camera_calibration_service_default_info.yml"), \
        display_description="PyRI Camera Calibration Service", device_manager_autoconnect=False, \
        distribution_name="pyri-vision") as service_node_setup:
        
        CameraCalibrationService_inst = CameraCalibrationService_impl(service_node_setup.device_manager, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "robotics_jog", type "tech.pyri.robotics.jog.RoboticsJogService", actual object: RoboticsJogService_inst
        service_node_setup.register_service("camera_calibration","tech.pyri.vision.calibration.CameraCalibrationService", CameraCalibrationService_inst)
        
        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()