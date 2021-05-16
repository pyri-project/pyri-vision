# @burakaksoy opencv_aruco_extrinsic_calibration.py

import numpy as np
import cv2
from cv2 import aruco
import general_robotics_toolbox as rox
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil


def _marker_corner_poses(T_marker, marker_size):
    corners = np.array([
        [-1,-1,0],
        [1,-1,0],
        [1,1,0],
        [-1,1,0]
    ]) * marker_size/2.0

    ret = []

    for i in range(4):
        ret.append(T_marker * rox.Transform(np.eye(3),corners[i,:].flatten()))

    return ret  


def calibrate(images, joint_poses, aruco_dict, aruco_id, aruco_markersize, flange_to_marker, mtx, dist, cam_pose, rox_robot, robot_local_device_name):
    """ Apply extrinsic camera calibration operation for images in the given directory path 
    using opencv aruco marker detection, the extrinsic marker poses given in a json file, 
    and the given intrinsic camera parameters."""

    assert aruco_dict.startswith("DICT_"), "Invalid aruco dictionary name"

    aruco_dict = getattr(aruco, aruco_dict) # convert string to python
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
    aruco_params = cv2.aruco.DetectorParameters_create()

    i = 0

    imgs_out = []

    geom_util = GeometryUtil()
    image_util = ImageUtil()

    object_points = []
    image_points = []

    for img,joints in zip(images,joint_poses):
        
        # Find the aruco tag corners
        # corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params,cameraMatrix=mtx, distCoeff=dist)
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

        # #debug
        # print(str(type(corners))) # <class 'list'>
        # print(str(corners))  # list of numpy arrays of corners
        # print(str(type(ids))) # <class 'numpy.ndarray'>
        # print(str(ids)) 

        if len(corners) > 0:
            # Only find the id that we desire
            indexes = np.flatnonzero(ids.flatten() == aruco_id).tolist()
            corners = [corners[index] for index in indexes]
            ids = np.asarray([ids[index] for index in indexes])

            # #debug
            # print(str(type(corners))) # <class 'list'>
            # print(str(corners))  # list of numpy arrays of corners
            # print(str(type(ids))) # <class 'numpy.ndarray'>
            # print(str(ids)) 

            if len(ids) > 0: # if there exist at least one id that we desire
                # Select the first detected one, discard the others
                corners = corners[0] # now corners is 1 by 4

                # # extract the marker corners (which are always returned
                # # in top-left, top-right, bottom-right, and bottom-left
                # # order)
                # corners = corners.reshape((4, 2))
                # (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Estimate the pose of the detected marker in camera frame
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_markersize, mtx, dist)

                # # Debug: Show the detected tag and axis in the image
                # # # cv2.aruco.drawDetectedMarkers(img, corners)  # Draw A square around the markers (Does not work)
                img1 = img.copy()
                img_out = cv2.aruco.drawAxis(img1, mtx, dist, rvec, tvec, aruco_markersize*0.75)  # Draw Axis
                imgs_out.append(img_out)
                
                transform_base_2_flange = rox.fwdkin(rox_robot, joints)
                transform_flange_2_marker = geom_util.pose_to_rox_transform(flange_to_marker)
                transform_base_2_marker = transform_base_2_flange * transform_flange_2_marker
                transform_base_2_marker_corners = _marker_corner_poses(transform_base_2_marker,aruco_markersize)
                # Structure of this disctionary is "filename":[[R_base2marker],[T_base2marker],[R_cam2marker],[T_cam2marker]]
                for j in range(4):
                    object_points.append(transform_base_2_marker_corners[j].p)
                    image_points.append(corners[0,j])
                #pose_pairs_dict[i] = (transform_base_2_marker_corners, corners)
                i+=1
    



    object_points_np = np.array(object_points,dtype=np.float64)
    image_points_np = np.array(image_points,dtype=np.float32)

    # Finally execute the calibration
    retval, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np, mtx, dist)
    R_cam2base = cv2.Rodrigues(rvec)[0]
    T_cam2base = tvec

    # Add another display image of marker at robot base
    img_out = cv2.aruco.drawAxis(img, mtx, dist, cv2.Rodrigues(R_cam2base)[0], T_cam2base, aruco_markersize*0.75)  # Draw Axis
    imgs_out.append(img_out)
    
    rox_transform_cam2base = rox.Transform(R_cam2base,T_cam2base,cam_pose.parent_frame_id,robot_local_device_name)
    rox_transform_world2base = cam_pose * rox_transform_cam2base

    #R_base2cam = R_cam2base.T
    #T_base2cam = - R_base2cam @ T_cam2base

    R_base2cam = rox_transform_world2base.inv().R
    T_base2cam = rox_transform_world2base.inv().p
    
    #debug
    print("FINAL RESULTS: ")
    print("str(R_cam2base)")
    # print(str(type(R_cam2base)))
    print(str(R_cam2base))
    print("str(T_cam2base)")
    # print(str(type(T_cam2base.flatten())))
    print(str(T_cam2base))
    
    print("str(R_base2cam)")
    # print(str(type(R_base2cam)))
    print(str(R_base2cam))
    print("str(T_base2cam)")
    # print(str(type(T_base2cam.flatten())))
    print(str(T_base2cam))
    
    pose_res = geom_util.rox_transform_to_named_pose(rox_transform_world2base)
    cov = np.eye(6) * 1e-5

    imgs_out2 = [image_util.array_to_compressed_image_jpg(i,70) for i in imgs_out]

    return pose_res, cov, imgs_out2, 0.0