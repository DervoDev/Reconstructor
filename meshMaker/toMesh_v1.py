import os
import open3d as o3d
from mergeClouds import trajectory_merger as trajMer




def mehsFragmentClouds(folder):
    print ("Start meshFragmentClouds")
    os.makedirs("tmp/"+folder+"/fragmentMesh/", exist_ok = True)
    # for each point cloud fragment in "[folder]/fragments"
    for fragment in os.listdir("tmp/"+folder+"/fragments/"):
        if(fragment.endswith(".ply")):
            print ("fragment: {}".format(fragment))
            f = o3d.io.read_point_cloud("tmp/"+folder+"/fragments/"+fragment)
            f.estimate_normals()
            radii = [0.5, 0.5, 0.5, 0.5]
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                f, o3d.utility.DoubleVector(radii))
            #mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(f,depth=10,linear_fit=True)
            o3d.io.write_triangle_mesh("tmp/"+folder+"/fragmentMesh/"+fragment,mesh)
    print("End of meshFragmentClouds")
    
    # create shared point cloud 
