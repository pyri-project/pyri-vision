import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np

import cv2
from cv2 import aruco
import math
import argparse
import shapely
import shapely.geometry
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.IdentifierUtil import IdentifierUtil
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

import general_robotics_toolbox as rox

class VisionArucoDetection_impl(object):
    def __init__(self, device_manager_url, device_info = None, node: RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info
                
        self.service_path = None
        self.ctx = None

        self._detected_marker = self._node.GetStructureType("tech.pyri.vision.aruco_detection.DetectedMarker")
        self._aruco_detection_result = self._node.GetStructureType("tech.pyri.vision.aruco_detection.ArucoDetectionResult")
        self._pose2d_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D")
        self._point2d_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Point2D")

        self._image_util = ImageUtil(node=self._node)
        self._geom_util = GeometryUtil(node=self._node)

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

    def _do_aruco_detection(self, img, intrinsic_global_name, extrinsic_global_name, aruco_dict_str, aruco_id, aruco_markersize, roi):

        intrinsic_calib = None
        extrinsic_calib = None

        if intrinsic_global_name is not None and extrinsic_global_name is not None:
            var_storage = self.device_manager.get_device_client("variable_storage",0.1)
            intrinsic_calib = var_storage.getf_variable_value("globals", intrinsic_global_name).data
            extrinsic_calib = var_storage.getf_variable_value("globals", extrinsic_global_name).data
        

        display_img = img.copy()

        assert aruco_dict_str.startswith("DICT_"), "Invalid aruco dictionary name"

        aruco_dict_i = getattr(aruco, aruco_dict_str) # convert string to python
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_i)
        aruco_params = cv2.aruco.DetectorParameters_create()
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        corners1, ids1, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

        if aruco_id < 0:
            corners2 = corners1
            ids2 = ids1
        else:
            corners2 = []
            ids2 = []
            for id2,corner2 in zip(ids1,corners1):
                if id2 == aruco_id:
                    corners2.append(corner2)
                    ids2.append(id2)
            ids2 = np.array(ids2)

        if roi is None:
            corners = corners2
            ids = ids2
        else:
            roi_x = roi.center.pose[0]["position"]["x"]
            roi_y = roi.center.pose[0]["position"]["y"]
            roi_theta = roi.center.pose[0]["orientation"]
            roi_w = roi.size[0]["width"]
            roi_h = roi.size[0]["height"]
            geom_roi1 = shapely.geometry.box(-roi_w/2.,-roi_h/2.,roi_w/2.,roi_h/2.,ccw=True)
            geom_roi2 = shapely.affinity.translate(geom_roi1, xoff = roi_x, yoff = roi_y)
            geom_roi = shapely.affinity.rotate(geom_roi2, roi_theta, origin='centroid', use_radians=True)           

            corners = []
            ids = []

            for id3,corner3 in zip(ids2,corners2):
                centroid = shapely.geometry.Polygon([shapely.geometry.Point(corner3[0,i,0],corner3[0,i,1]) for i in range(4)])
                if geom_roi.contains(centroid):
                    corners.append(corner3)
                    ids.append(id3)
            ids = np.array(ids)

            roi_outline = np.array([geom_roi.exterior.coords],dtype=np.int32)
            display_img = cv2.polylines(display_img, roi_outline, True, color=(255,255,0))

        if len(ids) > 0:
            display_img = aruco.drawDetectedMarkers(display_img,corners,ids)

        poses = None
        if intrinsic_calib is not None and extrinsic_calib is not None:
            poses = []
            mtx = intrinsic_calib.K
            d = intrinsic_calib.distortion_info.data
            dist = np.array([d.k1,d.k2,d.p1,d.p2,d.k3])

            T_cam = self._geom_util.named_pose_to_rox_transform(extrinsic_calib.pose)

            for id4,corner4 in zip(ids,corners):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corner4, aruco_markersize, mtx, dist)

                display_img = cv2.aruco.drawAxis(display_img,mtx,dist,rvec,tvec,0.05)

                # R_marker1 = cv2.Rodrigues(rvec.flatten())[0]
                # TODO: 3D pose estimation from rvec is very innaccurate. Use a simple trigonometry
                # to estimate the Z rotation of the tag
                
                # compute vectors from opposite corners
                v1 = corner4[0,2,:] - corner4[0,0,:]
                v2 = corner4[0,3,:] - corner4[0,1,:]

                # Use atan2 on each vector and average
                theta1 = (np.arctan2(v1[1],v1[0]) - np.deg2rad(45)) % (2.*np.pi)
                theta2 = (np.arctan2(v2[1],v2[0]) - np.deg2rad(135)) % (2.*np.pi)

                theta = (theta1 + theta2)/2.
                
                R_marker1 = rox.rot([1.,0.,0.],np.pi) @ rox.rot([0.,0.,-1.], theta)

                p_marker1 = tvec.flatten()

                T_marker1 = rox.Transform(R_marker1,p_marker1,"camera", f"aruco{int(id4)}")
                T_marker = T_cam * T_marker1
                rr_marker_pose = self._geom_util.rox_transform_to_named_pose(T_marker)
                poses.append(rr_marker_pose)

        ret_markers = []
        if poses is None:
            poses = [None]*len(ids)
        for id_,corner,pose in zip(ids,corners,poses):
            m = self._detected_marker()
            m.marker_id = int(id_)
            m.marker_corners = np.zeros((4,),dtype=self._point2d_dtype)
            for i in range(4):
                m.marker_corners[i] = self._geom_util.xy_to_point2d(corner[0,i,:])

            m.marker_pose = pose

            ret_markers.append(m)

        ret = self._aruco_detection_result()
        ret.detected_markers = ret_markers
        ret.display_image = self._image_util.array_to_compressed_image_jpg(display_img, 70)
        return ret

    def detect_aruco_stored_image(self, image_global_name, camera_calibration_intrinsic, camera_calibration_extrinsic, aruco_dict, aruco_id, aruco_markersize, roi):
        
        if len(camera_calibration_intrinsic) == 0:
            camera_calibration_intrinsic = None

        if len(camera_calibration_extrinsic) == 0:
            camera_calibration_extrinsic = None

        var_storage = self.device_manager.get_device_client("variable_storage",0.1)

        img_rr = var_storage.getf_variable_value("globals", image_global_name)
        img = self._image_util.compressed_image_to_array(img_rr.data)

        return self._do_aruco_detection(img, camera_calibration_intrinsic, camera_calibration_extrinsic, aruco_dict, aruco_id, aruco_markersize, roi)

    def detect_aruco_camera_capture(self, camera_local_device_name, camera_calibration_intrinsic, camera_calibration_extrinsic, aruco_dict, aruco_id, aruco_markersize, roi):
        
        if len(camera_calibration_intrinsic) == 0:
            camera_calibration_intrinsic = None

        if len(camera_calibration_extrinsic) == 0:
            camera_calibration_extrinsic = None

        camera_device = self.device_manager.get_device_client(camera_local_device_name,1)
        img_rr = camera_device.capture_frame_compressed()
        img = self._image_util.compressed_image_to_array(img_rr)

        return self._do_aruco_detection(img, camera_calibration_intrinsic, camera_calibration_extrinsic, aruco_dict, aruco_id, aruco_markersize, roi)

def main():

    parser = argparse.ArgumentParser(description="PyRI Aruco Detection Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for aruco detection service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.vision.aruco_detection.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("tech.pyri.vision.aruco_detection", 55920) as node_setup:

        # create object
        VisionArucoDetection_inst = VisionArucoDetection_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "vision_aruco_detection", type "tech.pyri.vision.aruco_detection.VisionArucoDetectionService", 
        # actual object: VisionArucoDetection_inst
        ctx = RRN.RegisterService("vision_aruco_detection","tech.pyri.vision.aruco_detection.VisionArucoDetectionService",VisionArucoDetection_inst)
        ctx.SetServiceAttributes(device_attributes)

        #Wait for the user to shutdown the service
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