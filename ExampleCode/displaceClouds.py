import os
import numpy as np
import open3d as o3d

def convert_trajectory(folder):
    # will contain a list of imported positions.
    ImportedPos = []     
    for i in os.listdir(folder+"/pose/"):
        ImportedPos.append(np.load(folder+"/pose/"+str(i).zfill(5)))
    return ImportedPos

def createPointClouds(folder, reductionFactor = 200):
    traj = convert_trajectory(folder)
    intr = o3d.io.read_pinhole_camera_intrinsic(folder+"/intrinsic.json")
    bracketTransfer = np.load('configs/T265toD435.npy')
    i = 0
    
    os.makedirs(folder+"/cloudRaw/", exist_ok = True)
    os.makedirs(folder+"/cloudShifted/", exist_ok = True)
    for image in os.listdir(folder+"/depth/"):
        depthImg = o3d.io.read_image(folder+"/depth/"+image)
        cloud = o3d.geometry.PointCloud.create_from_depth_image(depthImg, intr,
            depth_scale = 999.99993896484375)
        cloud = cloud.uniform_down_sample(reductionFactor)

        #save point cloud without adding estimated position
        o3d.io.write_point_cloud(folder+"/cloudRaw/"+ image[:-4] + ".ply",cloud)
            
        #save point cloud adjusted for estimated position
        trans = np.matmul(traj[i],bracketTransfer)
        cloud = cloud.transform(trans)
        o3d.io.write_point_cloud(folder+"/cloudShifted/"+ image[:-4] + ".ply",cloud)
        i += 1 