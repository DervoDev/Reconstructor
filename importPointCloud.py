import numpy as np
import open3d as o3d
import os

#support file for extra features, might be included later. 

for filename in os.listdir("tmp/cloud2"):
    if filename.endswith(".txt"):
        print(filename)
        xyz = np.loadtxt("tmp/cloud2/" + str(filename))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        o3d.io.write_point_cloud("tmp/cloud2/" + str(filename) + ".ply", pcd)

# Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
#xyz = np.loadtxt('cloud\T-bar2clean.txt')
##xyz = np.random.rand(100, 3)
#print(xyz)
#pcd = o3d.geometry.PointCloud()
#pcd.points = o3d.utility.Vector3dVector(xyz)
#o3d.io.write_point_cloud("./data.ply", pcd)

