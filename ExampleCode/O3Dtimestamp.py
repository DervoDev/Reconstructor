import open3d as o3d
import pyrealsense2 as rs

def o3DTimeStamp(bag_file):
    reader = o3d.t.io.RSBagReader()
    reader.open(bag_file)
    while not reader.is_eof():
        print(reader.get_timestamp())
        reader.next_frame()

def pyRealsense2Timestamp(bag_file):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(file_name = bag_file, repeat_playback = False)
    pipe.start(cfg)
    frame_id = 0

    profiles = pipe.get_active_profile()
    dev = profiles.get_device()
    
    playback = dev.as_playback()
    playback.set_real_time(False)
    success = True
    try:
        while(success):
            (success, frames) = pipe.try_wait_for_frames(1000)
            if success:
                print(frames.get_timestamp())
            else:
                print("Complete")
    finally:
        pipe.stop()