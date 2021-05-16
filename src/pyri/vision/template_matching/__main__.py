import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np

import cv2
import math
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

from .opencv_template_matching_ROI import TemplateMatchingMultiAngleWithROI

import general_robotics_toolbox as rox

from pyri.util.robotraconteur import add_default_ws_origins

class VisionTemplateMatching_impl(object):
    def __init__(self, device_manager_url, device_info = None, node: RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info
                
        self.service_path = None
        self.ctx = None

        self._matched_template_2d_type = self._node.GetStructureType("tech.pyri.vision.template_matching.MatchedTemplate2D")
        self._template_matching_result_2d_type = self._node.GetStructureType("tech.pyri.vision.template_matching.TemplateMatchingResult2D")
        self._template_matching_result_3d_type = self._node.GetStructureType("tech.pyri.vision.template_matching.TemplateMatchingResult3D")
        self._matched_template_3d_type = self._node.GetStructureType("tech.pyri.vision.template_matching.MatchedTemplate3D")
        self._named_pose_with_covariance_type = self._node.GetStructureType("com.robotraconteur.geometry.NamedPoseWithCovariance")
        self._pose2d_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Pose2D")

        self._image_util = ImageUtil(node=self._node)
        self._geom_util = GeometryUtil(node=self._node)

        self.device_manager = DeviceManagerClient(device_manager_url, autoconnect=False)
        self.device_manager.connect_device_type("tech.pyri.variable_storage.VariableStorage")
        self.device_manager.connect_device_type("com.robotraconteur.imaging.Camera")
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

    def match_template_stored_image(self, image_global_name, template_global_name, roi):
       
        var_storage = self.device_manager.get_device_client("variable_storage",1)

        image_var = var_storage.getf_variable_value("globals", image_global_name)
        image = self._image_util.compressed_image_to_array(image_var.data)

        template_var = var_storage.getf_variable_value("globals", template_global_name)
        template = self._image_util.compressed_image_to_array(template_var.data)

        return self._do_template_match(template, image, roi)

    def match_template_camera_capture(self, camera_local_device_name, template_global_name, roi):
       
        var_storage = self.device_manager.get_device_client("variable_storage",1)
        
        template_var = var_storage.getf_variable_value("globals", template_global_name)
        template = self._image_util.compressed_image_to_array(template_var.data)

        camera_device = self.device_manager.get_device_client(camera_local_device_name)
        image_compressed = camera_device.capture_frame_compressed()
        image = self._image_util.compressed_image_to_array(image_compressed)

        return self._do_template_match(template, image, roi)

    def _do_template_match(self, template, image, roi):

        matcher_roi = None

        if roi is not None:
            roi_x = roi.center.pose[0]["position"]["x"]
            roi_y = roi.center.pose[0]["position"]["y"]
            roi_theta = roi.center.pose[0]["orientation"]
            roi_w = roi.size[0]["width"]
            roi_h = roi.size[0]["height"]
            matcher_roi = (roi_x, roi_y, roi_w, roi_h, -roi_theta)


        # execute the image detection using opencv
        matcher = TemplateMatchingMultiAngleWithROI(template,image,matcher_roi)
        # return_result_image = True
        
        match_center, template_wh, match_angle, detection_result_img = matcher.detect_object(True)
                
        # return the pose of the object
        print("the object is found..:")
        print("center coordinates in img frame(x,y): " + str(match_center))
        print("(w,h): " + str(template_wh))
        print("angle: " + str(match_angle))


        match_result = self._template_matching_result_2d_type()

        matched_template_result = self._matched_template_2d_type()
        centroid = np.zeros((1,),dtype=self._pose2d_dtype)
        centroid[0]["position"]["x"] = match_center[0]
        centroid[0]["position"]["y"] = match_center[1]
        centroid[0]["orientation"] = match_angle
        matched_template_result.match_centroid = centroid
        matched_template_result.template_size = self._geom_util.wh_to_size2d(template_wh,dtype=np.int32)
        matched_template_result.confidence = 0

        match_result.template_matches =[matched_template_result]

        match_result.display_image = self._image_util.array_to_compressed_image_jpg(detection_result_img, 70)

        return match_result

    def _do_match_with_pose(self,image, template, intrinsic_calib, extrinsic_calib, object_z, roi):
        
        matcher_roi = None

        if roi is not None:
            roi_x = roi.center.pose[0]["position"]["x"]
            roi_y = roi.center.pose[0]["position"]["y"]
            roi_theta = roi.center.pose[0]["orientation"]
            roi_w = roi.size[0]["width"]
            roi_h = roi.size[0]["height"]
            matcher_roi = (roi_x, roi_y, roi_w, roi_h, -roi_theta)


        # execute the image detection using opencv
        matcher = TemplateMatchingMultiAngleWithROI(template,image,matcher_roi)
        # return_result_image = True
        
        match_center, template_wh, match_angle, detection_result_img = matcher.detect_object(True)
           
        # detection_result.width
        # detection_result.height
        x = match_center[0]
        y = match_center[1]
        theta = match_angle
        src = np.asarray([x,y], dtype=np.float32)
        src = np.reshape(src,(-1,1,2)) # Rehsape as opencv requires (N,1,2)
        # print(src)
        # Now the detection results in image frame is found,
        # Hence, find the detected pose in camera frame with given z distance to camera
        # To do that first we need to get the camera parameters
        # Load the camera matrix and distortion coefficients from the calibration result.
        

        mtx = intrinsic_calib.K
        d = intrinsic_calib.distortion_info.data
        dist = np.array([d.k1,d.k2,d.p1,d.p2,d.k3])

        T_cam = self._geom_util.named_pose_to_rox_transform(extrinsic_calib.pose)

        #TODO: Figure out a better value for this
        object_z_cam_dist = abs(T_cam.p[2]) - object_z

        # Find the corresponding world pose of the detected pose in camera frame
        dst = cv2.undistortPoints(src,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
        dst = dst * float(object_z_cam_dist) * 1000.0 # Multiply by given Zc distance to find all cordinates, multiply by 1000 is because of Zc is given in meters but others are in millimeters
        dst = np.squeeze(dst) * 0.001 # Xc and Yc as vector

        # Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
        Xc = dst[0]
        Yc = dst[1]
        Zc = float(object_z_cam_dist)
        T = np.asarray([Xc,Yc,Zc])

        # Now lets find the orientation of the detected object with respect to camera 
        # We are assuming +z axis is looking towards the camera and xy axes of the both object and camera are parallel planes
        # So the rotation matrix would be
        theta = np.deg2rad(theta) #convert theta from degrees to radian
        R_co = np.asarray([[math.cos(theta),-math.sin(theta),0],[-math.sin(theta),-math.cos(theta),0],[0,0,-1]])

        T_obj_cam_frame = rox.Transform(R_co, T, "camera", "object")

        T_obj = T_cam * T_obj_cam_frame
        
        # TODO: Better adjustment of Z height?
        T_obj.p[2] = object_z

        ret1 = self._matched_template_3d_type()
        ret1.pose = self._named_pose_with_covariance_type()
        ret1.pose.pose = self._geom_util.rox_transform_to_named_pose(T_obj)
        ret1.pose.covariance= np.zeros((6,6))
        ret1.confidence = 0

        ret = self._template_matching_result_3d_type()
        ret.template_matches = [ret1]
        ret.display_image = self._image_util.array_to_compressed_image_jpg(detection_result_img,70)

        return ret

    def _do_match_template_world_pose(self, image, template_global_name,
        camera_calibration_intrinsic, camera_calibration_extrinsic, object_z_height, roi, var_storage):

        template_var = var_storage.getf_variable_value("globals", template_global_name)
        template = self._image_util.compressed_image_to_array(template_var.data)

        intrinsic_calib = var_storage.getf_variable_value("globals", camera_calibration_intrinsic).data
        extrinsic_calib = var_storage.getf_variable_value("globals", camera_calibration_extrinsic).data

        return self._do_match_with_pose(image,template,intrinsic_calib,extrinsic_calib,object_z_height,roi)

    def match_template_world_pose_stored_image(self, image_global_name, template_global_name,
        camera_calibration_intrinsic, camera_calibration_extrinsic, object_z_height, roi):

        var_storage = self.device_manager.get_device_client("variable_storage",1)

        image_var = var_storage.getf_variable_value("globals", image_global_name)
        image = self._image_util.compressed_image_to_array(image_var.data)

        return self._do_match_template_world_pose(image, template_global_name, camera_calibration_intrinsic,
            camera_calibration_extrinsic, object_z_height, roi, var_storage)

    def match_template_world_pose_camera_capture(self, camera_local_device_name, template_global_name,
        camera_calibration_intrinsic, camera_calibration_extrinsic, object_z_height, roi):

        var_storage = self.device_manager.get_device_client("variable_storage",1)

        camera_device = self.device_manager.get_device_client(camera_local_device_name,1)
        img_rr = camera_device.capture_frame_compressed()
        image = self._image_util.compressed_image_to_array(img_rr)

        return self._do_match_template_world_pose(image, template_global_name, camera_calibration_intrinsic,
            camera_calibration_extrinsic, object_z_height, roi, var_storage)


def main():

    parser = argparse.ArgumentParser(description="PyRI Vision Template Matching Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for template matching service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")
    parser.add_argument("--pyri-webui-server-port",type=int,default=8000,help="The PyRI WebUI port for websocket origin (default 8000)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(resources.read_text(__package__,'tech.pyri.vision.template_matching.robdef'))

    with args.device_info_file:
        device_info_text = args.device_info_file.read()

    info_loader = InfoFileLoader(RRN)
    device_info, device_ident_fd = info_loader.LoadInfoFileFromString(device_info_text, "com.robotraconteur.device.DeviceInfo", "device")

    attributes_util = AttributesUtil(RRN)
    device_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(device_info)

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("tech.pyri.vision.template_matching", 55919) as node_setup:

        add_default_ws_origins(node_setup.tcp_transport,args.pyri_webui_server_port)

        # create object
        VisionTemplateMatching_inst = VisionTemplateMatching_impl(args.device_manager_url, device_info=device_info, node = RRN)
        # register service with service name "vision_template_matching", type "tech.pyri.vision.template_matching.VisionTemplateMatchingService", 
        # actual object: VisionTemplateMatching_inst
        ctx = RRN.RegisterService("vision_template_matching","tech.pyri.vision.template_matching.VisionTemplateMatchingService",VisionTemplateMatching_inst)
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