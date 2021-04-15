import os
import json
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from open3d.cpu.pybind.geometry import Image as im


#test for depth camera only

bag_filename = "tmp/stream/D435.bag"

def processD435(file,folder):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(file)
    bag_reader.save_frames("tmp/r1/")

    #next step is to run 
    ## python run_system.py G:\git\Reconstructor\tmp\t3\t3.json --make --register --refine --integrate


def readD435():
    import open3d as o3d
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open("tmp/stream/20210202_03_D435i.bag")
    
    if True:
        bag_reader.save_frames("tmp/r1/")

    frame_count = 0
    print("Start cycle")
    while not bag_reader.is_eof():# and False:
        frame_ID = str(frame_count).zfill(5)
        print('frame ID: ',frame_ID)
        frame_count = frame_count+1
        
        
        im_rgbd = bag_reader.next_frame()
        
        p = o3d.t.geometry.PointCloud.create_from_depth_image(im_rgbd.depth, 
                bag_reader.metadata.intrinsics)
        
        #os.makedirs("tmp/image/", exist_ok = True)
        #os.makedirs("tmp/depth/", exist_ok = True)
        os.makedirs("tmp/cloud/", exist_ok = True)

        print("opa")
        a = np.asanyarray(im_rgbd.depth)
        print(a)
        b = im(a)
        print(b)
        #i = o3d.io.write_image("tmp/image/"+frame_ID+".png", im(im_rgbd.color))
        #j = o3d.io.write_image("tmp/depth/"+frame_ID+".png", im_rgbd.depth)
        c = o3d.io.write_point_cloud("tmp/cloud",p)
        print ("dept:" + str(i)+", color:"+str(j)+", cloud:"+str(c))
        

    bag_reader.close()