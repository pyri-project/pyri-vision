# @burakaksoy opencv_aruco_extrinsic_calibration.py

import numpy as np
import cv2
from cv2 import aruco
import general_robotics_toolbox as rox
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

def rigid_transform_3D(A, B):
    # This function is taken from https://github.com/nghiaho12/rigid_transform_3D.git
    # Further information: http://nghiaho.com/?page_id=671
    
    # Input: expects 3xN matrix of points
    # Returns R,t
    # R = 3x3 rotation matrix
    # t = 3x1 column vector
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t

def calibrate(images, joint_poses, aruco_dict, aruco_id, aruco_markersize, flange_to_marker, mtx, dist, cam_pose, rox_robot, robot_local_device_name):
    """ Apply extrinsic camera calibration operation for images in the given directory path 
    using opencv aruco marker detection, the extrinsic marker poses given in a json file, 
    and the given intrinsic camera parameters."""

    assert aruco_dict.startswith("DICT_"), "Invalid aruco dictionary name"

    aruco_dict = getattr(aruco, aruco_dict) # convert string to python
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
    aruco_params = cv2.aruco.DetectorParameters_create()

    pose_pairs_dict={}
    i = 0

    imgs_out = []

    geom_util = GeometryUtil()
    image_util = ImageUtil()

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
                # cv2.imshow("detected tag",img_out)
                # cv2.waitKey()

                # Convert tvec to one dimensional array, it was [[tvec]].
                T_cam2marker = tvec[0].flatten() # in camera's frame
                # Convert rvec to Rotation matrix
                R_cam2marker = cv2.Rodrigues(rvec[0])[0]

                # Store the calculated marker pose pair
                #R_base2marker = np.asarray(saved_marker_poses_dict[fname.split('/')[-1]][0]) # Convert every R and T into numpy array in the dictionary
                #T_base2marker = np.asarray(saved_marker_poses_dict[fname.split('/')[-1]][1])

                transform_base_2_flange = rox.fwdkin(rox_robot, joints)
                transform_flange_2_marker = geom_util.pose_to_rox_transform(flange_to_marker)
                transform_base_2_marker = transform_base_2_flange * transform_flange_2_marker
                R_base2marker = transform_base_2_marker.R
                T_base2marker = transform_base_2_marker.p

                # Structure of this disctionary is "filename":[[R_base2marker],[T_base2marker],[R_cam2marker],[T_cam2marker]]
                pose_pairs_dict[i] = [R_base2marker,T_base2marker,R_cam2marker,T_cam2marker]
                i+=1
    
    # cv2.destroyAllWindows() # Debug
    print("HERE") # debug

    # Now its time to execute calibration
    src_lst = []
    dst_lst = []
    
    for img_number, value in pose_pairs_dict.items():
        R_base2marker = value[0]
        T_base2marker = value[1]
        R_cam2marker = value[2]
        T_cam2marker = value[3]

        # print("------------------------")
        # print(filename)
        # # #debug
        # # print(str(type(R_base2marker)))
        # # print(str(R_base2marker))  
        # # print(str(type(T_base2marker))) 
        # # print(str(T_base2marker))
        # # #debug
        # # print(str(type(R_cam2marker)))
        # # print(str(R_cam2marker))  
        # # print(str(type(T_cam2marker))) 
        # # print(str(T_cam2marker)) 

        # # debug
        # print("My own calculation T_cam2base:")
        # print(str(T_cam2marker - (R_cam2marker @ R_base2marker.T @ T_base2marker) ))
        # print("My own calculation R_cam2base:")
        # print(str(R_cam2marker @ R_base2marker.T))

        # print("My own calculation T_base2cam:")
        # print(str(T_base2marker - (R_base2marker @ R_cam2marker.T @ T_cam2marker ) ))
        # print("My own calculation R_base2cam:")
        # print(str(R_base2marker @ R_cam2marker.T))
        # print("++++++++++++++++++++++++")

        src_lst.append(T_base2marker)
        dst_lst.append(T_cam2marker)
        
    # Convert lists to numpy arrays because this is what opencv wants
    src_arr = np.asarray(src_lst)
    dst_arr = np.asarray(dst_lst)

    # Finally execute the calibration
    R_cam2base,T_cam2base = rigid_transform_3D(src_arr.T,dst_arr.T)

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