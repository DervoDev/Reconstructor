import os
import cv2
import json
import shutil
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from PIL import Image as im
from scipy.spatial.transform import Rotation as R



def activateDevice(file):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(file_name = file, repeat_playback = False)
    pipe.start(cfg)
    frame_id = 0

    profiles = pipe.get_active_profile()
    dev = profiles.get_device()
    
    playback = dev.as_playback()
    playback.set_real_time(False)
    return pipe

def createMatrix(p):
    rot = R.from_quat([p.rotation.w, p.rotation.z, p.rotation.x, p.rotation.x])
    m = rot.as_matrix()
    matrix = [  [m[0][0],m[0][1],m[0][2],p.translation.x],
                [m[1][0],m[1][1],m[1][2],p.translation.y],
                [m[2][0],m[2][1],m[2][2],p.translation.z],
                [0,0,0,1]]
    #print("posMatrix")
    #print(matrix)
    return matrix

def processRGBD(file, folder):

    #read file and store the folloing
    # - timestampRGBD.json - containing a structure with each frame and its connected timestamp and frameeID
    # - "folder/color/" - folder where all color frames are stored by "xxxxx.jpg" and x is the frame number starting with frame 0 
    # - "folder/depth/" - folder where all depth frames are stored by "xxxxx.png" and x is the frame number starting with frame 0
    # - intrinsic.json - containing info generated from the bag file to be used later in the Open3D pipeline
    pipe = activateDevice(file)
    
    # create folders
    if not os.path.exists("tmp/"+folder+"/depth/"):
        os.mkdir("tmp/"+folder+"/depth/")
    if not os.path.exists("tmp/"+folder+"/color/"):
        os.mkdir("tmp/"+folder+"/color/")
    try:
        data = {}
        data["frames"] = []
        frame_nr = 0
        while(True):
            (success, frames) = pipe.try_wait_for_frames(1000)
            if not success:
                print("Finished")
                break
            
            depth = im.fromarray(np.asanyarray(frames.get_depth_frame().get_data()))
            color = im.fromarray(np.asanyarray(frames.get_color_frame().get_data()))
            
            

            if depth and color:

                frame_ts = frames.get_timestamp()
                frame_id = frames.frame_number
                data["frames"].append({
                    'nr':frame_nr,
                    'id':frame_id,
                    'ts':frame_ts,
                    'cpi':0, # closest pose frame id
                    'cps':0 # closest pose seconds difference
                })
                
                # save pictures
                #depth.resize((1280,720), resample=im.BICUBIC)
                depth.save("tmp/"+folder+"/depth/"+str(frame_nr).zfill(5) + ".png", "PNG")
                color.save("tmp/"+folder+"/color/"+str(frame_nr).zfill(5) + ".jpg", "JPEG")
                frame_nr = frame_nr+1

                #depth is not in the right size. the next step has cv2 load, resize and replace each depth frame.

                    
        with open("tmp/"+folder+"/timestampRGBD.json", "w") as outfile:
            json.dump(data,outfile)
    finally:
        pipe.stop()


def processPose(file, folder):
    
    #read file and store the following
    # timestampPose.json - containing a structure with timestamp, pose matrix and frameID 
    pipe = activateDevice(file)
    try:
        data = {}
        data["frames"] = []
        frame_nr = 0
        while(True):
            (success, frames) = pipe.try_wait_for_frames(100)
            if not success:
                break
            pose = frames.get_pose_frame()
            if pose:
                
                frame_ts = frames.get_timestamp()
                frame_id = pose.frame_number
                print("frame-id: {}, -ts: {}, pose: {}, size:{}".format(frame_nr,frame_ts,pose,str(frames.size()).zfill(5)))
                #m = createMatrix(pose.get_pose_data())
                #print(m)
                data["frames"].append({
                    'nr':frame_nr,
                    'id':frame_id,
                    'ts':frame_ts,
                    'ma':createMatrix(pose.get_pose_data())
                }) 
                frame_nr = frame_nr+1
        with open("tmp/"+folder+"/timestampPose.json", "w") as outfile:
            json.dump(data,outfile)
    finally:
        pipe.stop()

def alignFrames(folder):
    # read timestampPose.json and timestampRGBD.json
    # and for each object in timestampRGBD find the closest timestamp connected in timestampPose. 
    # when found save that pose as a matrix in tmp/folder/pose/xxxxx.npy

    with open("tmp/"+folder+"/timestampPose.json") as poses_file:
        poses = json.load(poses_file)
        with open("tmp/"+folder+"/timestampRGBD.json") as rgbd_file:
            rgbd_timestamps = json.load(rgbd_file)
            os.makedirs("tmp/"+folder+"/pose/", exist_ok=True)
            for ts in rgbd_timestamps["frames"]:
                ts["cpi"] = 0
                ts["cps"] = np.abs(ts["ts"] - poses["frames"][0]["ts"])
                #find closest by comparing timeframe
                for pos in poses["frames"]:
                    #print("compare rgbd {} ({}) to pose {} ({}) with diff {}".format(ts["nr"],ts["ts"],pos["nr"],pos["ts"], np.abs(ts["ts"] - pos["ts"])))
                    if (ts["cps"] > np.abs(ts["ts"] - pos["ts"])):
                        ts["cpi"] = pos["nr"]
                        ts["cps"] = np.abs(ts["ts"] - pos["ts"])
                # save pose
                txt = "tmp/"+folder+"/pose/"+str(ts["nr"]).zfill(5)+".npy"
                print("save frame {} with pose nr {} that is ts[cpi] {} and matrix: {}".format(ts["nr"],poses["frames"][ts["cpi"]]["nr"],ts["cpi"],poses["frames"][ts["cpi"]]["ma"]))
                np.save(txt,poses["frames"][ts["cpi"]]["ma"])

def createConfig(folder):
    file = {
        "name": folder+" reconstruction",
        "path_dataset": "G:/git/Reconstructor/tmp/"+folder+"/",
        "path_intrinsic": "G:/git/Reconstructor/tmp/"+folder+"/intrinsic.json",
        "max_depth": 0.7, #in meters
        "voxel_size": 0.01,
        "max_depth_diff": 0.07,
        "preference_loop_closure_odometry": 0.1,
        "preference_loop_closure_registration": 5.0,
        "tsdf_cubic_size": 0.4,
        "icp_method": "color",
        "global_registration": "ransac",
        "python_multi_threading": False,
        "n_frames_per_fragment": 1000,
        "n_keyframes_per_n_frame": 20
    }
    with open("tmp/"+folder+"/config.json", "w") as outfile:
        json.dump(file,outfile)


def processBag(bag_d435,bag_t265,folder):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(bag_d435)
    bag_reader.save_frames("tmp/"+folder)
    
    try:
        shutil.rmtree("tmp/"+folder+"/depth/")
        shutil.rmtree("tmp/"+folder+"/color/")
    except OSError as e:
        print ("Error: %s - %s." % (e.filename, e.strerror))
    
    processRGBD(bag_d435,folder)
    processPose(bag_t265,folder)
    alignFrames(folder)
    createConfig(folder)

    