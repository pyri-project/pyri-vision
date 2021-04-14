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

import time
import threading
import traceback

import general_robotics_toolbox as rox

import cv2
from cv2 import aruco

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

    for image in images:
        # TODO: more robust image conversion
        frame = image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            # img = cv2.drawChessboardCorners(frame, (width, height), corners2, ret)
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

    return ret, mtx, dist.flatten(), rvecs, tvecs

def _calibrate_camera_intrinsic(images, calibration_target):    
    ret, mtx, dist, rvecs, tvecs = _calibrate_camera_intrinsic2(images,calibration_target)
    if not ret:
        raise RR.OperationFailedException("Camera intrinsic calibration failed")

    calib = RRN.NewStructure("com.robotraconteur.imaging.camerainfo.CameraCalibration")
    size2d = RRN.GetNamedArrayDType("com.robotraconteur.geometryi.Size2D")
    calib.image_size=np.zeros((1,),dtype=size2d)
    calib.image_size[0]["width"]=images[0].image_info.width
    calib.image_size[0]["height"]=images[0].image_info.height

    calib.K = mtx

    dist_rr = RRN.NewStructure("com.robotraconteur.imaging.camerainfo.PlumbBobDistortionInfo")
    dist_rr.k1 = dist[0]
    dist_rr.k2 = dist[1]
    dist_rr.p1 = dist[2]
    dist_rr.p2 = dist[3]
    dist_rr.k3 = dist[4]

    calib.distortion_info = RR.VarValue(dist_rr,"com.robotraconteur.imaging.camerainfo.PlumbBobDistortionInfo")

    return calib

def _calibrate_camera_extrinsic(intrinsic_calib, image, board, aruco_dict, camera_local_device_name):

    # TODO: verify calibration data

    mtx = intrinsic_calib.K
    dist_rr = intrinsic_calib.distortion_info.data
    dist = np.array([dist_rr.k1, dist_rr.k2, dist_rr.p1, dist_rr.p2, dist_rr.k3],dtype=np.float64)

    frame = image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    assert len(corners)>0
    # SUB PIXEL DETECTION
    for corner in corners:
        cv2.cornerSubPix(gray, corner,
                            winSize = (3,3),
                            zeroZone = (-1,-1),
                            criteria = criteria)
    retval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
    assert retval

    retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, mtx, dist, None, None)
    assert retval

    im_with_charuco_board = aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)
    im_with_charuco_board = aruco.drawDetectedMarkers(im_with_charuco_board, corners, ids)
    cv2.imshow("img",im_with_charuco_board)
    cv2.waitKey()
    cv2.destroyAllWindows()


    R = (cv2.Rodrigues(rvec)[0]).transpose()
    p = -R @ tvec
    q = rox.R2q(R)

    pose_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose')
    pose = np.zeros((1,),dtype=pose_dt)
    pose[0]["orientation"]["w"] = q[0]
    pose[0]["orientation"]["x"] = q[1]
    pose[0]["orientation"]["y"] = q[2]
    pose[0]["orientation"]["z"] = q[3]
    pose[0]["position"]["x"] = p[0]
    pose[0]["position"]["y"] = p[1]
    pose[0]["position"]["z"] = p[2]

    ident_util = IdentifierUtil()

    named_pose = RRN.NewStructure("com.robotraconteur.geometry.NamedPose")
    named_pose.pose = pose
    named_pose.parent_frame = ident_util.CreateIdentifierFromName("world")
    named_pose.frame = ident_util.CreateIdentifierFromName(camera_local_device_name)

    return named_pose

    


class CameraCalibrationService_impl:
    def __init__(self, device_manager_url, device_info = None, node : RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info
                
        self.service_path = None
        self.ctx = None

        self.device_manager = DeviceManagerClient(device_manager_url)
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

        if len(var_storage.filter_variables("globals",output_global_name,[])) > 0:
            raise RR.InvalidOperationException(f"Global {output_global_name} already exists")

        image_sequence = []

        image_sequence_vars = var_storage.getf_variable_value("globals",image_sequence_global_name)
        for image_var in image_sequence_vars.data.splitlines():
            var2 = var_storage.getf_variable_value("globals",image_var)
            image_sequence.append(var2.data)

        camera_calib = _calibrate_camera_intrinsic(image_sequence, calibration_target)

        var_storage.add_variable2("globals",output_global_name,"com.robotraconteur.imaging.camerainfo.CameraCalibration", \
            RR.VarValue(camera_calib,"com.robotraconteur.imaging.camerainfo.CameraCalibration"), ["camera_calibration_intrinsic"], 
            {"device": camera_local_device_name}, variable_persistence["const"], None, variable_protection_level["read_write"], \
            [], f"Camera \"{camera_local_device_name}\" intrinsic calibration", False)

    def calibrate_camera_extrinsic(self, camera_local_device_name, camera_intrinsic_calibration_global_name,
        image_global_name, origin_calibration_target, output_global_name):
        
        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        if len(var_storage.filter_variables("globals",output_global_name,[])) > 0:
            raise RR.InvalidOperationException(f"Global {output_global_name} already exists")

        image = var_storage.getf_variable_value("globals",image_global_name).data
        intrinsic_calib = var_storage.getf_variable_value("globals",camera_intrinsic_calibration_global_name).data

        target, aruco_dict = _get_target_board(origin_calibration_target)
        camera_calib = _calibrate_camera_extrinsic(intrinsic_calib, image, target, aruco_dict, camera_local_device_name)

        var_storage.add_variable2("globals",output_global_name,"com.robotraconteur.geometry.NamedPose", \
            RR.VarValue(camera_calib,"com.robotraconteur.imaging.geometry.NamedPose"), ["camera_calibration_extrinsic"], 
            {"device": camera_local_device_name}, variable_persistence["const"], None, variable_protection_level["read_write"], \
            [], f"Camera \"{camera_local_device_name}\" extrinsic calibration", False)

def main():

    parser = argparse.ArgumentParser(description="PyRI Camera Calibration Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for devices states service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")
    
    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.vision.calibration.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)


    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("tech.pyri.vision.camera_calibration", 55917) as node_setup:

        # register service type
        

        # create object
        CameraCalibrationService_inst = CameraCalibrationService_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "robotics_jog", type "tech.pyri.robotics.jog.RoboticsJogService", actual object: RoboticsJogService_inst
        ctx = RRN.RegisterService("camera_calibration","tech.pyri.vision.calibration.CameraCalibrationService", CameraCalibrationService_inst)
        ctx.SetServiceAttributes(device_attributes)

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")


if __name__ == '__main__':
    main()