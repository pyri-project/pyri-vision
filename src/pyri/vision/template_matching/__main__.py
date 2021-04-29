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

    def find_object_pose_in_cam_frame(self,obj_img_filename, camera_name, value_z_distance):
        if self.is_plugins_connected and self.is_cameras_connected:
            # If the value of z distance is given bigger than zero use the given one 
            #otherwise camera has 3D capability, get z value from thecapability of the camera
            if value_z_distance <= 0:
                #TODO
                # value_z_distance = getFrom3DCapableCamera
                print("Provide a proper positive z distance")
            
            # Find the object in image frame 
            detection_result = self.find_object_in_img_frame(obj_img_filename, camera_name, False)
            
            # detection_result.width
            # detection_result.height
            x = detection_result.center_x
            y = detection_result.center_y
            theta = detection_result.angle
            src = np.asarray([x,y], dtype=np.float32)
            src = np.reshape(src,(-1,1,2)) # Rehsape as opencv requires (N,1,2)
            # print(src)
            # Now the detection results in image frame is found,
            # Hence, find the detected pose in camera frame with given z distance to camera
            # To do that first we need to get the camera parameters
            # Load the camera matrix and distortion coefficients from the calibration result.
            filename = str(camera_name) + ".yml"
            params = self.plugin_cameraCalibration.load_calibration(filename)

            mtx = params.camera_matrix 
            dist = params.distortion_coefficients
            # R_co = params.R_co 
            # T_co = params.T_co 
            # error = params.error

            # Find the corresponding world pose of the detected pose in camera frame
            dst = cv2.undistortPoints(src,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
            dst = dst * float(value_z_distance) * 1000.0 # Multiply by given Zc distance to find all cordinates, multiply by 1000 is because of Zc is given in meters but others are in millimeters
            dst = np.squeeze(dst) * 0.001 # Xc and Yc as vector

            # Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
            Xc = dst[0]
            Yc = dst[1]
            Zc = float(value_z_distance)
            T = np.asarray([Xc,Yc,Zc])

            # Now lets find the orientation of the detected object with respect to camera 
            # We are assuming +z axis is looking towards the camera and xy axes of the both object and camera are parallel planes
            # So the rotation matrix would be
            theta = np.deg2rad(theta) #convert theta from degrees to radian
            R_co = np.asarray([[math.cos(theta),-math.sin(theta),0],[-math.sin(theta),-math.cos(theta),0],[0,0,-1]])

            # Pack the results into CalibrationParameters structure
            pose = RRN.NewStructure("experimental.pluginCameraTracking.Pose")
            pose.R = R_co
            pose.T = T

            return pose

        else:
            # Give an error message to show that the robot is not connected
            print("Cameras or plugins are not connected to VisionTemplateMatching service yet!")




def main():

    parser = argparse.ArgumentParser(description="PyRI Vision Template Matching Service")    
    parser.add_argument("--device-info-file", type=argparse.FileType('r'),default=None,required=True,help="Device info file for template matching service (required)")
    parser.add_argument('--device-manager-url', type=str, default=None,required=True,help="Robot Raconteur URL for device manager service (required)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM or SIGINT (Linux only)")

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