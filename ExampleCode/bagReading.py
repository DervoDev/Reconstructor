import open3d as o3d
import numpy as np
import pyrealsense2 as rs
from PIL import Image as im

def o3dManual(bag_file):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(bag_file)
    frame_count = 0
    while not bag_reader.is_eof():
        im_rgbd = bag_reader.next_frame()
        color = im.fromarray(np.asanyarray(im_rgbd.color))
        if color:
            color.save("pymanual/color/"+str(frame_count).zfill(5)+".jpg", "JPEG")
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
                depth.save("pymanual/depth/"+str(frame_nr).zfill(5) +".png", "PNG")
                color.save("pymanual/color/"+str(frame_nr).zfill(5) +".jpg", "JPEG")
                frame_nr = frame_nr + 1
    finally:
        pipe.stop()