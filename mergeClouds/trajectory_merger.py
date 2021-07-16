import os
import json
import numpy as np
import pandas as pd
import open3d as o3d
from pyntcloud import PyntCloud
from pyntcloud.io import read_ply
from scipy.spatial.transform import Rotation as rot




# read trajectory.log generated from Open3Ds reconstruction system for 3d-reconstruction of rgbd video. 



#input 
# - location of trajectory.log
# - folder with depth images.


#find
# - number of depth images 
# - 


#read and sort trajectory.log

#matrix = np.npasanarra
def convert_trajectory(folder):
    with open("tmp/"+folder+"/scene/trajectory.log") as f:
        lines = f.readlines()
        f = len(lines)/5
        i = int(f)
        m = np.ndarray(shape=(i,4,4),dtype=float)

        for l in range(0,len(lines),5):
            m[int(l/5)][0] = lines[l+1].split()
            m[int(l/5)][1] = lines[l+2].split()
            m[int(l/5)][2] = lines[l+3].split()
            m[int(l/5)][3] = lines[l+4].split()
        return m
def convert_trajectory2(folder):
    ImportedPos = []    # will contain a list of imported positions. 
    for i in os.listdir("tmp/"+folder+"/pose/"):
        ImportedPos.append(np.load("tmp/"+folder+"/pose/"+str(i).zfill(5)))
    print("Lenght of trajectory: {}".format(len(ImportedPos)))
    return ImportedPos

def createAlongPathRaw(folder):
    with open("tmp/"+folder+"/timestampPose.json") as poses_file:
        poses = json.load(poses_file)
        traj = []
        #process poses to get 
        for p in poses["frames"]:
            traj.append((p["ma"][0][3],p["ma"][1][3],p["ma"][2][3],))
        file_cloud = PyntCloud(pd.DataFrame(
            columns=["x","y","z"],
            data=traj
        ))
        file_cloud.to_file("tmp/"+folder+"/trajectoryRaw.ply")

def createConesAlongPath(folder):
    cloud = []
    print("start work")
    for p in os.listdir("tmp/"+folder+"/pose/"):
        c = np.load("tmp/"+folder+"/pose/"+p)
        cloud.append((c[0][3],c[1][3],c[2][3]))

    file_cloud = PyntCloud(pd.DataFrame(
        columns=["x","y","z"],
        data=cloud,
    ))
    file_cloud.to_file("tmp/"+folder+"/trajectorySynced.ply")

def createPointCloudsFromDepth2(folder, reductionFactor = 100):
    print("***************************************")
    print("****** START TRAJECTORY MERGING *******")
    print("***************************************")
    traj = convert_trajectory2(folder)
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")
    bracketTransfer = np.load('configs/T265toD435.npy')
    #print(bracketTransfer)
    i = 0
    
    names = ["_000","_00p","_00m","_0p0","_0pp","_0pm","_0m0","_0mp","_0mm","_m00","_m0p","_m0m","_mp0","_mpp","_mpm","_mm0","_mmp","_mmm","_mmm","_p00","_p0p","_p0m","_pp0","_ppp","_ppm","_pm0","_pmp","_pmm"]
    os.makedirs("tmp/"+folder+"/cloudShifted/", exist_ok = True)
    os.makedirs("tmp/"+folder+"/cloudRaw/", exist_ok = True)
    #print(traj)
    rots = []
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,  0,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,  0, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,  0,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0, 90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0, 90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0, 90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,-90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,-90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,-90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,  0,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,  0, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,  0,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90, 90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90, 90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90, 90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,-90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,-90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=( 90,-90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(  0,-90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,  0,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,  0, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,  0,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90, 90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90, 90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90, 90,-90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,-90,  0),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,-90, 90),degrees=True))
    rots.append(rot.from_euler(seq="XYZ",angles=(-90,-90,-90),degrees=True))
    for image in os.listdir("tmp/"+folder+"/depth/"):
            if image.endswith(".png") and i <1:
                depthImg = o3d.io.read_image("tmp/"+folder+"/depth/"+image)
                cloud = o3d.geometry.PointCloud.create_from_depth_image(depthImg, intr, depth_scale = 999.99993896484375)
                cloud = cloud.uniform_down_sample(reductionFactor)
                    
                for j in range(len(names)):
                    r = rots[j]
                    cen = np.ndarray(shape=(3,1), buffer=np.array([0.0,0.0,0.0]))
                    c = cloud.rotate(R=r.as_matrix(),center=cen)
                    o3d.io.write_point_cloud("tmp/"+folder+"/cloudShifted/"+ image[:-4]+names[j]+ ".ply",c)

                i += 1


def createPointCloudsFromDepth(folder, reductionFactor = 200):
    print("***************************************")
    print("****** START TRAJECTORY MERGING *******")
    print("***************************************")
    traj = convert_trajectory2(folder)
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")
    bracketTransfer = np.load('configs/T265toD435.npy')
    #print(bracketTransfer)
    i = 0
    #print(traj)
    
    os.makedirs("tmp/"+folder+"/cloudRaw/", exist_ok = True)
    os.makedirs("tmp/"+folder+"/cloudShifted/", exist_ok = True)
    for image in os.listdir("tmp/"+folder+"/depth/"):
            if image.endswith(".png") and True: #counter < 10:
                depthImg = o3d.io.read_image("tmp/"+folder+"/depth/"+image)
                #cloud = o3d.geometry.PointCloud.create_from_rgbd_image(depthImg,intr)    
                #j = o3d.t.geometry.Image(np.asanyarray(depthImg))
                
                cloud = o3d.geometry.PointCloud.create_from_depth_image(depthImg, intr, depth_scale = 999.99993896484375)
                s = len(np.asarray(cloud.points))
                cloud = cloud.uniform_down_sample(reductionFactor)
                o3d.io.write_point_cloud("tmp/"+folder+"/cloudRaw/"+ image[:-4] + ".ply",cloud)
                #print ("Cloud nr {}: lenght before {} and after {}".format(i,s,len(np.asarray(cloud.points))))
                #print (traj[i])
                trans = np.matmul(traj[i],bracketTransfer)
                c = cloud.transform(trans)
                o3d.io.write_point_cloud("tmp/"+folder+"/cloudShifted/"+ image[:-4] + ".ply",cloud)
                #print("cloud {} complete".format(i))
                i += 1 

def createPointCloudsFromDepth3(folder,reductionFactor = 100):
    print("***************************************")
    print("****** START TRAJECTORY MERGING *******")
    print("***************************************")
    traj = convert_trajectory(folder)
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")
    bracketTransfer = np.load('configs/T265toD435.npy')
    #print(bracketTransfer)
    i = 0
    os.makedirs("tmp/"+folder+"/cloudFinal/", exist_ok = True)
    for image in os.listdir("tmp/"+folder+"/depth/"):
            if image.endswith(".png") and True: #counter < 10:
                depthImg = o3d.io.read_image("tmp/"+folder+"/depth/"+image)
                cloud = o3d.geometry.PointCloud.create_from_depth_image(depthImg, intr, depth_scale = 999.99993896484375)
                cloud = cloud.uniform_down_sample(reductionFactor)
                #trans = np.matmul(bracketTransfer,traj[i])
                trans = np.matmul(traj[i],bracketTransfer)
                c = cloud.transform(trans)
                o3d.io.write_point_cloud("tmp/"+folder+"/cloudFinal/"+ image[:-4] + ".ply",c)
                i += 1 

def moveCloudByTrajectory(folder,reductionFactor = 100):
    i = 0
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")
    traj = convert_trajectory2(folder)
    for image in os.listdir("tmp/"+folder+"/depth/"):
        if image.endswith(".png") and True: #counter < 10:
            depthImg = o3d.io.read_image("tmp/"+folder+"/depth/"+image)
            os.makedirs("tmp/"+folder+"/cloudProcessed/", exist_ok = True)
            cloud = o3d.geometry.PointCloud.create_from_depth_image(depthImg, intr, depth_scale = 999.99993896484375)

def mergeCloud(folder):
    all_files = os.listdir("tmp/"+folder+"/shiftedCloud/")
    merged_points = pd.concat([read_ply("tmp/"+folder+"/shiftedCloud/" + x)["points"] for x in all_files])
    #os.makedirs("tmp/"+folder+"/singleCloud/", exist_ok = True)
    file_cloud = PyntCloud(pd.DataFrame(
        columns=["x","y","z"],
        data=merged_points
    ))
    file_cloud.to_file("tmp/"+folder+"/singleCloud/pointCloud.ply")
    print("save complete")


def addToPointCloud(cloud,name):
    print("Start adding to point cloud file")
    #os.makedirs("tmp/"+folder+"/shiftedCloud/", exist_ok = True)
    file_cloud = PyntCloud(pd.DataFrame(
        data=cloud,
        columns=["x","y","z"]
    ))
    file_cloud.to_file("tmp/"+folder+"/shiftedCloud/"+ name + ".ply")
    print("save complete")

def checkShift(folder,s,t):
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")
    img_s = o3d.io.read_image("tmp/"+folder+"/depth/"+str(s).zfill(5)+".png")
    img_t = o3d.io.read_image("tmp/"+folder+"/depth/"+str(t).zfill(5)+".png")
    pose_s = np.load("tmp/"+folder+"/pose/"+str(s).zfill(5)+ ".npy")
    pose_t = np.load("tmp/"+folder+"/pose/"+str(t).zfill(5)+ ".npy")
    cloud_s = o3d.geometry.PointCloud.create_from_depth_image(img_s, intr, depth_scale = 999.99993896484375)
    cloud_t = o3d.geometry.PointCloud.create_from_depth_image(img_t, intr, depth_scale = 999.99993896484375) 

   
    
    o3d.io.write_point_cloud("tmp/"+folder+"/blender/source{}.ply".format(str(s).zfill(5)),cloud_s)
    o3d.io.write_point_cloud("tmp/"+folder+"/blender/target{}.ply".format(str(t).zfill(5)),cloud_t)

    # merge point clouds after shifting cloud
    t_s = cloud_s.transform(pose_s)
    t_t = cloud_t.transform(pose_t)

    o3d.io.write_point_cloud("tmp/"+folder+"/blender/sourceMoved{}.ply".format(str(s).zfill(5)),t_s)
    o3d.io.write_point_cloud("tmp/"+folder+"/blender/targetMoved{}.ply".format(str(t).zfill(5)),t_t)

    # merge point clouds using open3d algorithm. 
    #c_t01 = cloud_t.transform(np.matmul(pose_s,pose_t))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}to{}.ply".format(str(s).zfill(5),str(t).zfill(5)),c_t01)

    #c_t02 = cloud_t.transform(np.matmul(pose_t,pose_s))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}to{}.ply".format(str(t).zfill(5),str(s).zfill(5)),c_t02)

    #c_t01 = cloud_t.transform(np.matmul(np.linalg.inv(pose_s),pose_t))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}inv_to{}.ply".format(str(s).zfill(5),str(t).zfill(5)),c_t01)

    #c_t02 = cloud_t.transform(np.matmul(np.linalg.inv(pose_t),pose_s))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}inv_to{}.ply".format(str(t).zfill(5),str(s).zfill(5)),c_t02)

    
    #c_t01 = cloud_t.transform(np.matmul(pose_s,np.linalg.inv(pose_t)))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}to{}inv.ply".format(str(s).zfill(5),str(t).zfill(5)),c_t01)

    #c_t02 = cloud_t.transform(np.matmul(pose_t,np.linalg.inv(pose_s)))
    #o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}to{}inv.ply".format(str(t).zfill(5),str(s).zfill(5)),c_t02)

    c_t03 = cloud_t.transform(np.matmul(pose_t,np.linalg.inv(pose_s)))
    c_t03 = ct03.transform()
    o3d.io.write_point_cloud("tmp/"+folder+"/blender/target_{}to{}inv.ply".format(str(t).zfill(5),str(s).zfill(5)),c_t03)