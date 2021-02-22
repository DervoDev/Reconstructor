import os
import cv2
import time
import imageio
import numpy as np
import pandas as pd
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pyntcloud import PyntCloud
from collections import deque
from PIL import Image

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor

############################################################
###############      Support functions       ###############
############################################################

def print_timestamps(delay=None, **kwargs):
    """
    Print timestamps of sensors in console. Print difference if more then 2
    :param delay: delay in seconds
    :param kwargs: should be in format SensorName1=timestamp1, SensorName2=timestamp2
    :return: None
    """
    timestamp = []
    for kw in kwargs.keys():
        print("{0} sensor timestamp: {1}".format(kw, kwargs[kw]))
        if type(kwargs[kw]) == float:
            timestamp.append(kwargs[kw])
    if len(timestamp) == 2:
        print("Difference between timestamps is [1]-[0]: {}".format(timestamp[1] - timestamp[0]))
    print('')
    if delay is not None:
        time.sleep(delay)

def readCam(folder="cameraToCloud/exampleData/"):
    pose_number = 0
    transformation_matrix_set265 = deque()
    transformation_matrix_set435 = deque()
    transformation_D435 = []
    transformation_trajectory_D435 = []
    points_trajectory_D435 = []
    points_trajectory_T265 = []
    # ask for folder of footage

    # assuming root folder
    for filename in os.listdir(folder):
        f = folder + filename
        if filename.endswith("T265.bag"):
            T265 = T265Sensor(is_device=False, source_name=f)
        if filename.endswith("D435.bag"):
            D435 = D435Sensor(is_device=False, source_name=f)

    #in folder find pairs of files with each pair ending on **D435i.* and **T235.*
    #T265 = T265Sensor(is_device=False, source_name='data/T265.bag')
    #D435 = D435Sensor(is_device=False, source_name='data/D435.bag')

    D435.attach(T265)  # subscribe T265 on D435 updates

    T265.start_sensor()
    time.sleep(0.250 + 0.02)  # TODO: include difference here (computers are different)
    D435.start_sensor()

    prev_t265_tr_mx = None
    cur_time = 0
    cur_frame = 0

    try:
        while True & cur_frame <10:
            depth_frame = D435.get_depth_frame()
            depth_image = D435.get_depth_image()
            rgb_image = D435.get_rgb_image()
            pose265 = T265.get_pose()

            if (depth_frame is not None) and (pose265 is not None) and (rgb_image is not None):
                print_timestamps(D435=depth_frame.get_timestamp(), T265=pose265.get_timestamp())

                # TODO: extract grayscale image here and show images

                #depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                #cv2.imshow('D435 Depth Frame', depth_image)
                #cv2.waitKey(1)

                if depth_frame.get_timestamp() < cur_time:
                    prev_t265_tr_mx = None
                    D435.point_cloud = None
                cur_time = depth_frame.get_timestamp()
                
                # # Transformation matrix for updating the pose per frame. 
                transformation_matrix265 = T265.get_transformation()
                if prev_t265_tr_mx is None:
                    rel_tr_mx_265 = transformation_matrix265.copy()
                else:
                    rel_tr_mx_265 = np.linalg.inv(prev_t265_tr_mx) @ transformation_matrix265

                prev_t265_tr_mx = transformation_matrix265.copy()
                print('transformation_matrix \n', transformation_matrix265)

                # # get_coordinates() gets the depth-frames points after it has been transformed by the pose addjustment. 
                # # pc now contains a frames point cloud
                pc = D435.get_coordinates()
                print('pc \n', pc)
                print('pc.shape \n', pc.shape)
                # octo_visualiser.update_points(pc)

                # Forward pc together with RGB image and pose to point cloud merging functions.  
                # # Frame ID
                frame_ID = str(cur_frame).zfill(5)
                print('frame ID: ',frame_ID)
                cur_frame = cur_frame+1

                # # Point cloud 
                save_pointCloud(frame_ID,pc)

                # # Pose matrix 
                save_pose2(frame_ID,pose265)
                    
                # # RGB frame image
                if rgb_image:
                    color_image = np.asanyarray(rgb_image.get_data())
                    color_image = color_image[..., ::-1]
                    save_image(frame_ID, color_image)
                else:
                    print("coult not get color_image, skip for this frame")
                # # TODO: find frame-rate for stored items



                # TODO: get transformation mask from D435
                transformation_matrix435 = D435.get_transformation(init_guess=rel_tr_mx_265)


                # TODO: Transformation points and append to existing (association)
                # # TODO: make this function work by sending point cloud to screen for verification
                # #     Skip of too hard, we have what we need allready. 
                # if show_points:
                #     pc = rs.pointcloud()
                #     points = pc.calculate(depth_frame).as_points()
                #     coordinates = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(480, 848, 3)) \
                #         .reshape((-1, 3))
                #     point_viewer.set_points(coordinates)
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()
    finally:
        D435.stop_sensor()
        T265.stop_sensor()

# sync frames

save_folder = "tmp/"


def save_pointCloud(id,cloud):
    print("Start saving point cloud file")
    os.makedirs(save_folder + "cloud/", exist_ok = True)
    file_cloud = PyntCloud(pd.DataFrame(
        data=cloud,
        columns=["x","y","z"]
    ))
    file_cloud.to_file(save_folder +"cloud/"+ id + ".ply")
    print("save complete")

def save_image(id,img):
    os.makedirs(save_folder + "img/",  exist_ok = True)
    #img.save(save_folder +"img/"+ id + ".png")
    imageio.imwrite(save_folder +"img/"+ id + ".png", img)

def save_pose1(id,data):
    pose = data.get_pose_data()
    os.makedirs(save_folder + "csv/", exist_ok = True)
    txt = save_folder + "csv/"+id + ".csv"
    print("Frame #{}".format(data.frame_number))
    print("Position: {}".format(pose.translation))
    print("Velocity: {}".format(pose.velocity))
    print("Rotation: {}".format(pose.rotation))
    f= open(txt,"w+")
    f.write("vecX,vecY,vecZ,velX,velY,velZ,rotW,rotZ,rotX,rotY\n")
    f.write(""+str(pose.translation.x)+","+str(pose.translation.y)+","+str(pose.translation.z)+","+
            str(pose.velocity.x)+","+str(pose.velocity.y)+","+str(pose.velocity.z)+","+
            str(pose.rotation.w)+","+str(pose.rotation.z)+","+str(pose.rotation.x)+","+str(pose.rotation.x))
    f.close
    #np.savetxt("foo.csv", matrix, delimiter=",")

def save_pose2(id,data):
    pose = data.get_pose_data()
    os.makedirs(save_folder + "pose/", exist_ok=True)
    txt = save_folder + "pose/"+id + ".npy"
    matrix = createMatrix(pose)
    np.save(txt, matrix)

def createMatrix(p):
    rot = R.from_quat([p.rotation.w, p.rotation.z, p.rotation.x, p.rotation.x])
    print("rot.as_matrix")
    print(rot.as_matrix)
    m = rot.as_matrix()
    matrix = [  [m[0][0],m[0][1],m[0][2],p.translation.x],
                [m[1][0],m[1][1],m[1][2],p.translation.y],
                [m[2][0],m[2][1],m[2][2],p.translation.z],
                [0,0,0,1]]
    print("posMatrix")
    print(matrix)
    return matrix