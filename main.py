#import open3d as o3d
#from open3d.geometry import PointCloud as opc 
from cameraToCloud import camReader
from mergeClouds import pointMerger
#import cameraToCloud.camReader  





#camera to point cloud
camReader.readCam()

#merge point clouds
pointMerger.mergeClouds()

#point cloud to mesh

#- get normals on points

#- - orient_normals_consistent_tangent_plane(self, k)
#opc.orient_normals_consistent_tangent_plane(self, k)


# self model
# k, number of suroundlying points to calculate orientation from

#- Poisson surface reconstruction


#texturing mesh