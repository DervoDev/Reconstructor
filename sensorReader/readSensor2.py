import os
import cv2
import json
import shutil
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import PIL
from PIL import ImageOps as imOps
from PIL import Image as im
from PIL import ImageChops as imCop
from scipy.spatial.transform import Rotation as R

def activateDevice(file):
    print("Activating device at "+ file)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(file_name = file, repeat_playback = False)
    pipe.start(cfg)

    profiles = pipe.get_active_profile()
    dev = profiles.get_device()
    
    if "D435" in file:
        d_scale = (dev.first_depth_sensor().get_depth_scale())
        print("d_scale:"+str(d_scale))
    
    playback = dev.as_playback()
    playback.set_real_time(False)
    return pipe

#quaternion + position to position matrix
def createMatrix(p):
    rot = R.from_quat([p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w])
    m = rot.as_matrix()
    matrix = [  [m[0][0],m[0][1],m[0][2],p.translation.x],
                [m[1][0],m[1][1],m[1][2],p.translation.y],
                [m[2][0],m[2][1],m[2][2],p.translation.z],
                [0,0,0,1]]
    return matrix

def processRGBD(file, folder):

    #read file and store the folloing
    # - timestampRGBD.json - containing a structure with each frame and its 
    #       connected timestamp and frameeID
    # - "folder/color/" - folder where all color frames are stored by 
    #       "xxxxx.jpg" and x is the frame number starting with frame 0 
    # - "folder/depth/" - folder where all depth frames are stored by 
    #       "xxxxx.png" and x is the frame number starting with frame 0
    # - intrinsic.json - containing info generated from the bag file to be 
    #       used later in the Open3D pipeline
    pipe = activateDevice(file)

    
    # create folders
    os.mkdir("tmp/"+folder+"/depth/")
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
            #fitImg(np.asanyarray(frames.get_depth_frame().get_data()), 0.15)
            depth = im.fromarray(fitImg(np.asanyarray(frames.get_depth_frame().get_data()),0.15))
            color = im.fromarray(np.asanyarray(frames.get_color_frame().get_data()))
            
            if depth and color:
                frame_ts = frames.get_timestamp()
                frame_id = frames.frame_number
                data["frames"].append({
                    'nr':frame_nr,
                    'id':frame_id,
                    'ts':frame_ts,
                    'cpi':0, # closest pose frame id - to be used later
                    'cps':0,  # closest pose seconds difference - to be used later
                    'ma':""
                })
                
                # save pictures
                depth.save("tmp/"+folder+"/depth/"+str(frame_nr).zfill(5) + ".png", "PNG")
                color.save("tmp/"+folder+"/color/"+str(frame_nr).zfill(5) + ".jpg", "JPEG")
                frame_nr = frame_nr+1
        with open("tmp/"+folder+"/timestampRGBD.json", "w") as outfile:
            json.dump(data,outfile)
    finally:
        pipe.stop()
    

def processPose(file, folder):
    # read file and store the following
    # timestampPose.json- containing a structure with timestamp, 
    # pose matrix and frameID
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
                print("frame-id: {}, -ts: {}, pose: {}, size:{}".format(
                    frame_nr, frame_ts,pose, str(frames.size()).zfill(5)))
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

def alignFrames2(folder):
    # read timestampPose.json and timestampRGBD.json
    # and for each object in timestampRGBD find the closest timestamp connected 
    # in timestampPose. When found save that pose as a matrix in 
    # tmp/folder/pose/xxxxx.npy

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
                    if (ts["cps"] > np.abs(ts["ts"] - pos["ts"])):
                        ts["cpi"] = pos["nr"]
                        ts["cps"] = np.abs(ts["ts"] - pos["ts"])
                # save pose
                txt = "tmp/"+folder+"/pose/"+str(ts["nr"]).zfill(5)+".npy"
                print("save frame {} with pose nr {} that is ts[cpi] {} and "+
                    "matrix: {}".format(ts["nr"], poses["frames"][ts["cpi"]]["nr"],
                    ts["cpi"],poses["frames"][ts["cpi"]]["ma"]))
                #np.save(txt,poses["frames"][ts["cpi"]]["ma"])
                ts["ma"] = poses["frames"][ts["cpi"]]["ma"]
            
            #find which color and color 2 frames are a match. 
            for color in os.listdir("tmp/"+folder+"/color/"):
                #print("color:" + color)
                i_source = im.open("tmp/"+folder+"/color/"+color)
                for target in os.listdir("tmp/"+folder+"/color2/"):
                    #reduce amount of comparisons
                    if(int(color[:-4]) < (int(target[:-4])+5)):
                        i_target = im.open("tmp/"+folder+"/color2/"+target)
                        diff = imCop.difference(i_source, i_target)
                        dif_img = np.asarray(diff)
                        sum = 0
                        for i in dif_img:
                            for j in i:
                                sum = sum + j[0]+j[1]+j[2]
                        #print(sum)
                        #print("{} pixels between {} and {}".format(sum,color[:-4],target[:-4]))
                        #diff.save("tmp/"+folder+"/test.jpg")
                        
                        if sum < 10000000:
                            print("match found between {} and {}".format(color[:-4],target[:-4]))
                            #match found 
                            txt = "tmp/"+folder+"/pose/"+str(color[:-4])+".npy"
                            np.save(txt,rgbd_timestamps["frames"][int(target[:-4])])
                            break

            

def alignFrames(folder):
        # read timestampPose.json and timestampRGBD.json
    # and for each object in timestampRGBD find the closest timestamp connected 
    # in timestampPose. When found save that pose as a matrix in 
    # tmp/folder/pose/xxxxx.npy

    with open("tmp/"+folder+"/timestampPose.json") as poses_file:
        poses = json.load(poses_file)
        with open("tmp/"+folder+"/timestampRGBD.json") as rgbd_file:
            rgbd_timestamps = json.load(rgbd_file)
            os.makedirs("tmp/"+folder+"/pose/", exist_ok=True)
            #for each rgbd frame connect a timestamp to it. 
            for ts in rgbd_timestamps["frames"]:
                ts["cpi"] = 0
                ts["cps"] = np.abs(ts["ts"] - poses["frames"][0]["ts"])
                #find closest by comparing timeframe
                for pos in poses["frames"]:
                    if (ts["cps"] > np.abs(ts["ts"] - pos["ts"])):
                        ts["cpi"] = pos["nr"]
                        ts["cps"] = np.abs(ts["ts"] - pos["ts"])
                # save pose
                txt = "tmp/"+folder+"/pose/"+str(ts["nr"]).zfill(5)+".npy"
                print("save frame {} with pose nr {} that is ts[cpi] {} and "+
                    "matrix: {}".format(ts["nr"], poses["frames"][ts["cpi"]]["nr"],
                    ts["cpi"],poses["frames"][ts["cpi"]]["ma"]))
                np.save(txt,poses["frames"][ts["cpi"]]["ma"])

def lerpPos(a,b,t):
    return (int((1-t)*a[0]+t*b[0]),int((1-t)*a[1]+t*b[1]))

def fitImg(img, p):
    h, w = img.shape
    img = img[int(h*p):int(h-(h*p)), int(w*p): int(w-(w*p))]
    img = cv2.resize(img,(w,h),interpolation=cv2.INTER_NEAREST)
    return img

def fitImg2(img, p):
    h, w = img.shape
    img = img[int(h*p):int(h-(h*p)), int(w*p): int(w-(w*p))]
    mask = img # need to fill all black areas back to black so that any interpolation do not drag a point cloud into the
    
    kernel = np.ones((5,5),np.uint16)
    mask = cv2.erode(mask,kernel,iterations = 1)
    mH, mW = mask.shape 
    img = cv2.resize(img,(w,h),interpolation=cv2.INTER_NEAREST)
    #for each pixel in img
    #print("mask w:{},h:{}".format(mW,mH))
    #for i in range(w):#1280
    #    pW = i/w
    #    if int(pW*mW) < mH:
    #        #for j in range(h):#720
    #            pH = j/h
                #if int(pH*mH) < pH:
                #print("i{}in{},j{}in{},".format(i,w,j,h))
                    #print("s:i:{},j={},mI:{},mJ:{},mask={}".format(i,j,int(pW*mW),int(pH*mH),mask[int(pW*mW),int(pH*mH)]))
                    #if(mask[int(pW*mW),int(pH*mH)] == 0):
                        
                        #img[i,j] = 0
                        #print("before:{},after:{}".format(tmp,img[i,j]))
    #print("img refitted")
    return (img)

def testDepthDifference(bag, folder):
    #processRGBDAuto(bag,folder)
    if True:
        try:
            #shutil.rmtree("tmp/"+folder+"/depth2/")
            #shutil.rmtree("tmp/"+folder+"/depthcomp/")
            #shutil.rmtree("tmp/"+folder+"/color2/")
            shutil.rmtree("tmp/"+folder+"/depthCrop/")
        except OSError as e:
            print ("Error: %s - %s." % (e.filename, e.strerror))
        #os.mkdir("tmp/"+folder+"/depth2/")
        #os.mkdir("tmp/"+folder+"/depthcomp/")
        #os.mkdir("tmp/"+folder+"/color2/")
        os.mkdir("tmp/"+folder+"/depthCrop/")

        pipe = activateDevice(bag)
        try:
            data = {}
            data["frames"] = []
            frame_nr = 0
            while(True):
                (success, frames) = pipe.try_wait_for_frames(1000)
                
                if not success:
                    print("Finished")
                    break
                d = fitImg(np.asanyarray(frames.get_depth_frame().get_data()), 0.15)
                depth = im.fromarray(d)
                
                #color = im.fromarray(np.asanyarray(frames.get_color_frame().get_data()))
                if depth:
                    #print("ts:"+str(frames.get_depth_frame().get_timestamp()))
                    # save pictures
                    #depth.save("tmp/"+folder+"/depth2/"+str(frame_nr).zfill(5) + ".png", "PNG")
                    #color.save("tmp/"+folder+"/color2/"+str(frame_nr).zfill(5) + ".png", "PNG")
                    #depth = to_8bit(depth)
                    
                    #depthCrop = imOps.fit(depth, (1280,729), bleed=0.15, centering=(0.5, 0.5))
                    #depthCrop = to_16bit(depthCrop)
                    depth.save("tmp/"+folder+"/depthCrop/"+str(frame_nr).zfill(5) + ".png", "PNG")
                    #np.asarray(depth)

                frame_nr = frame_nr+1
        finally:
            pipe.stop()   
        

    if False:
        print("comparing depth frames")
        for o in os.listdir("tmp/"+folder+"/depth/"):
            imO = force_8bit(im.open("tmp/"+folder+"/depth/"+o))
            #imO = PIL.ImageOps.fit(imO, (1280,729), bleed=0.15, centering=(0.5, 0.5))
            imOclosest = ""
            comp = 0
            for r in os.listdir("tmp/"+folder+"/depth2/"):
                imR = force_8bit(im.open("tmp/"+folder+"/depth2/"+r))
                imR = imOps.fit(imR, (1280,729), bleed=0.15, centering=(0.5, 0.5))
                diff = imCop.difference(imO,imR)
                dif_img = np.asarray(diff)
                sum = 0
                for i in dif_img:
                    for j in i:
                        sum = sum + j
                if imOclosest == "":
                    print("f"+str(sum))
                    imOclosest = imR
                    comp = sum
                elif comp > sum: 
                    print("n"+str(sum))
                    imOclosest = imR
                    comp = sum
            print("Closest for: {} is: {}, with closeness as: {}".format(o,r,comp))
            diff.save("tmp/"+folder+"/depthcomp/o"+str(o[:-4])+"_r"+str(r[:-4])+".png", "PNG")

def createConfig(folder, d_scale = 1000):
    
    # this assumes the intrinsics.json has been created/updated.
    # current d_scale:  999.9999389648438
                    #   999.9999389648438
    if d_scale == 1000:
        with open("G:/git/Reconstructor/tmp/"+folder+"/intrinsic.json") as intr_file:
            intr = json.load(intr_file)
        d_scale = intr["depth_scale"]
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
        "n_frames_per_fragment": 100,
        "n_keyframes_per_n_frame": 5,
        "depth_scale": d_scale
    }
    with open("tmp/"+folder+"/config.json", "w") as outfile:
        json.dump(file,outfile)

def processRGBDAuto(file,folder):
    rgbd_video = o3d.t.io.RGBDVideoReader.create(file)
    rgbd_video.save_frames("tmp/"+folder)

def processBagreaderAuto(bag_d435, folder):
    bag_reader = o3d.t.io.RSBagReader()
    bag_reader.open(bag_d435)
    bag_reader.save_frames("tmp/"+folder)

def processBag(bag_d435,bag_t265,folder):
    processRGBDAuto(bag_d435,folder)
    #processBagreaderAuto(bag_d435,folder)
    try:
        shutil.rmtree("tmp/"+folder+"/depth/")
        shutil.rmtree("tmp/"+folder+"/color/")
    except OSError as e:
        print ("Error: %s - %s." % (e.filename, e.strerror))
    processRGBD(bag_d435,folder)
    processPose(bag_t265,folder)
    alignFrames(folder)
    #alignFrames2(folder)
    createConfig(folder)
    #testDepthDifference(bag_d435,folder)

