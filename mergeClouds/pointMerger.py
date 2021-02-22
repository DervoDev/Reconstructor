import os
import csv
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud
from pyntcloud.io import read_ply
from scipy.spatial.transform import Rotation as R

def mergeClouds():
    #create empty .ply file


    # read tmp/cloud folder (assume tmp/csv has equal amount of files numbered from 00000.csv to xxxxx.csv)


    tmpCl = "tmp/cloud/"
    tmpCs = "tmp/pose/"
    bracketTransfer = np.load('configs/T265toD435.npy')
    print("t265 to D435:\n", bracketTransfer)
    counter = 0
    for filename in os.listdir(tmpCl):
        counter = counter + 1
        if counter < 10:
            if filename.endswith(".ply"):
                #load camera position from /csv. 
                #tra.x, tra.y, tra.z, vel.x, vel.y, vel.z, rot.w, rot.z, rot.x, rot.y
                camPos = 0
                #with open(tmpCs + filename[:-4] + ".csv", newline='') as csvfile:
                #    camPos = list(csv.reader(csvfile, delimiter=',', quotechar='|'))
                camPos = np.load(tmpCs + filename[:-4] + ".npy")
                print(camPos)
                #load point cloud from /cloud. 
                cloud = PyntCloud.from_file(tmpCl+filename)
                print("before transformation")
                print(cloud.points)

                #move pc acording to camera position. 
                a = apply_transformation(camPos,cloud.points)
                
                #move pc acording to bracket displacement
                a = apply_transformation(bracketTransfer, a)
                print("after transformation")
                

                
                
                addToPointCloud(a,filename[:-4])
    print("Start merging to single point cloud file")
    all_files = os.listdir("tmp/shiftedCloud/")
    merged_points = pd.concat([read_ply("tmp/shiftedCloud/" + x)["points"] for x in all_files])
    os.makedirs("tmp/singleCloud/", exist_ok = True)
    file_cloud = PyntCloud(pd.DataFrame(
        data=merged_points,
        columns=["x","y","z"]
    ))
    file_cloud.to_file("tmp/singleCloud/pointCloud.ply")
    print("save complete")

# borrowed from Tsykunov (2020)
def apply_transformation(transformation, points):
        """

        :param transformation:
        :param points:
        :return:
        """
        if transformation is None or points is None:
            return None
        else:
            coordinates = np.hstack((points, np.ones((points.shape[0], 1))))
            return (transformation @ coordinates.T).T[:, :-1]

def createMatrix(camPos):
    rot = R.from_quat([camPos[1][6], camPos[1][7], camPos[1][8], camPos[1][9]])
    print("rot.as_matrix")
    print(rot.as_matrix)
    m = rot.as_matrix()
    matrix = [  [m[0][0],m[0][1],m[0][2],camPos[1][0]],
                [m[1][0],m[1][1],m[1][2],camPos[1][2]],
                [m[2][0],m[2][1],m[2][2],camPos[1][3]],
                [0,0,0,1]]
    print("posMatrix")
    print(matrix)
    return matrix

def addToPointCloud(cloud,name):
    print("Start adding to point cloud file")
    os.makedirs("tmp/shiftedCloud/", exist_ok = True)
    file_cloud = PyntCloud(pd.DataFrame(
        data=cloud,
        columns=["x","y","z"]
    ))
    file_cloud.to_file("tmp/shiftedCloud/"+ name + ".ply")
    print("save complete")

