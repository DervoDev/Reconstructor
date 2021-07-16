from sensorReader import readSensor2
from mergeClouds import trajectory_merger
from meshMaker import toMesh_v1 as toMesh
import open3d as o3d
import pyrealsense2 as rs
from PIL import Image as im
import numpy as np

def o3dManual(bag_file):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(bag_file)
    frame_count = 0
    while not bag_reader.is_eof():
        im_rgbd = bag_reader.next_frame()
        color = im.fromarray(np.asanyarray(im_rgbd.color))
        if color:
            color.save("t/o3dmanual/color/"+str(frame_count).zfill(5) + ".jpg", "JPEG")
            frame_count = frame_count+1

def o3dAuto(bag_file):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(bag_file)
    bag_reader.save_frames("t/o3dauto")

def pyRs2ManualD435(bag_file):

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(file_name = bag_file, repeat_playback = False)
    pipe.start(cfg)

    profiles = pipe.get_active_profile()
    dev = profiles.get_device()
    
    playback = dev.as_playback()
    playback.set_real_time(False)
    
    try:
        frame_nr = 0
        success = True
        while(success):
            (success, frames) = pipe.try_wait_for_frames(1000)
            
            depth = im.fromarray(np.asanyarray(frames.get_depth_frame().get_data()))
            color = im.fromarray(np.asanyarray(frames.get_color_frame().get_data()))
            
            if depth and color:
                depth.save("t/pymanual/depth/"+str(frame_nr).zfill(5) + ".png", "PNG")
                color.save("t/pymanual/color/"+str(frame_nr).zfill(5) + ".jpg", "JPEG")
                frame_nr = frame_nr+1
    finally:
        pipe.stop()



folder = ["t11"]

# currently main.py must be run two times. once first, then second after the Open3D has been run. 
# Will later try and integrate Open3D - reconstruction_system example into this version. 
step_1 = True


if step_1:
    for f in folder:
    #Main function, replace [folder] to what folder you have used.
        #o3dManual("tmp/"+f+"/stream/D435.bag")
        #o3dAuto("tmp/"+f+"/stream/D435.bag")
        #pyRs2ManualD435("tmp/"+f+"/stream/D435.bag")
        readSensor2.processBag("tmp/"+f+"/stream/D435.bag", "tmp/"+f+"/stream/T265.bag", f)
        #trajectory_merger.createPointCloudsFromDepth(f,400)
        #trajectory_merger.createPointCloudsFromDepth3(f,400)
else:
    for f in folder:
        trajectory_merger.createPointCloudsFromDepth3(f,400)
        toMesh.mehsFragmentClouds(f)


#test.createRGBD()

#Create point cloud and translate using raw pose data. 
# output is stored in cloudRaw and cloudShifted. 
# 1000 is the factor of downsampling with the algorithm only storing every k'th point, 
# starting on 0, k, 2k, 3k, ... This to make it easier to process and analyse results. 
#trajectory_merger.createPointCloudsFromDepth(folder,400)

