
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

#############################################################################
############# Open3D library ################################################
#############################################################################

def o3DTimeStampTest(file):
    print("O3d timestamp test")
    reader = o3d.t.io.RSBagReader()
    reader.open(file)
    reader.next_frame()
    start_timer = reader.get_timestamp()
    end_timer = 0
    while not reader.is_eof():
        end_timer = reader.get_timestamp()
        print(end_timer)
        reader.next_frame()

    print ("Result: s_timer: {}, e_timer: {}, difference = {}".format(start_timer,end_timer, end_timer-start_timer))

#############################################################################
############# PyRealsense library ###########################################
#############################################################################

#py realsense reads bag files as live format, and thus skips frames it does 
# have time to process, the frames object is a Composite_frame as stated here:
# https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.composite_frame.html
# it has a foreach() function but no example of how to utilise it. 
# the size() tells how many frames are in the composite_frame, but it will only 
# read the first frame skipping the rest. 

# the pipe object is of pipeline 
# https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.pipeline.html
# which explains wait_for_frames() at the bottom


def pyRSTimeStampTest(file):
    print("pyRS timestamp test")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(file)
    pipe.start(cfg)

    start_timer = pipe.wait_for_frames().get_timestamp()
    last_timer = 0.0
    end_timer = 0.0
    eof = False
    while not eof:
        frames = pipe.wait_for_frames()
        end_timer = frames.get_timestamp()
        if last_timer > end_timer:
            eof = True
        else:
            last_timer = end_timer
    pipe.stop()
    print ("Result: s_timer: {}, e_timer: {}, difference = {}".format(start_timer,last_timer, last_timer-start_timer))