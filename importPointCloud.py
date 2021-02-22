import numpy as np
import open3d as o3d
import os


for filename in os.listdir("cloud"):
    if filename.endswith(".txt"):
        print(filename)
        xyz = np.loadtxt("cloud/" + str(filename))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        o3d.io.write_point_cloud("cloud/" + str(filename) + ".ply", pcd)

# Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
#xyz = np.loadtxt('cloud\T-bar2clean.txt')
##xyz = np.random.rand(100, 3)
#print(xyz)
#pcd = o3d.geometry.PointCloud()
#pcd.points = o3d.utility.Vector3dVector(xyz)
#o3d.io.write_point_cloud("./data.ply", pcd)

