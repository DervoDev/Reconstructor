import os
import time
import json
import math as m
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from datetime import datetime as dt
from scipy.spatial.transform import Rotation as R

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor

def print_timestamps(delay=None, **kwargs):
    """
    Print timestamps of sensors in console. Print difference if more then 2
    :param delay: delay in seconds
    :param kwargs: should be in format SensorName1=timestamp1, SensorName2=timestamp2
    :return: None
    """
    timestamp = []
    for kw in kwargs.keys():
        print("{0} sensor timestamp: {1}".format(kw, kwargs[kw]))
        if type(kwargs[kw]) == float:
            timestamp.append(kwargs[kw])
    if len(timestamp) == 2:
        print("Difference between timestamps is [1]-[0]: {}".format(timestamp[1] - timestamp[0]))
    print('')
    if delay is not None:
        time.sleep(delay)


def createMatrix(p):
    rot = R.from_quat([p.rotation.w, p.rotation.z, p.rotation.x, p.rotation.x])
    #print("rot.as_matrix")
    #print(rot.as_matrix)
    m = rot.as_matrix()
    matrix = [  [m[0][0],m[0][1],m[0][2],p.translation.x],
                [m[1][0],m[1][1],m[1][2],p.translation.y],
                [m[2][0],m[2][1],m[2][2],p.translation.z],
                [0,0,0,1]]
    #print("posMatrix")
    #print(matrix)
    return matrix

def save_pose2(id,pose,folder):
    os.makedirs("tmp/"+folder + "/pose/", exist_ok=True)
    txt = "tmp/{}/pose/{}.npy".format(folder,id)
    np.save(txt, pose)

def toPositive(a):
    if a<0:
        return a*-1
    return a

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


def processPoseO3D(D435,folder):
    bagReader = o3d.t.io.RSBagReader()
    bagReader.open(D435)




def processPosePyRS(D435,folder):
    # now that we have a series of color and depth frames, and a file with all t265 frame poses,
    # we then want to find for each depth frame find the best match of a pose using timestamp. 
    # we then want to save the corresponding pose with the same id as the depth frame id. 
    print("process pose")
    
    

    #read json file
    with open("tmp/"+folder+"/t265_frames.json") as json_file:
        t265 = json.load(json_file)
        print("open and load json file: {}".format(t265["frames"][0]))
        #for each frame in bagReader
        
        #prepare depth-reader
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device_from_file(D435)
        pipe.start(cfg)
        frame_id = 0

        last_ts = 0.0
        eof= False

        try:
            while True and not eof:
                frames = pipe.wait_for_frames()
                frames.foreach()
                depth = frames.get_depth_frame()

                
                ts = frames.get_timestamp()
                fnum = frames.get_frame_number()
                
                
                closestFrame = {}
                closestFrame["id"] = t265["frames"][0]["id"]
                closestFrame["ts"] = t265["frames"][0]["ts"]
                closestFrame["ma"] = t265["frames"][0]["ma"]
                closestFrame["closeness"] = toPositive(float(t265["frames"][0]["ts"])-ts)
                for element in t265["frames"]:
                    #print("frame ({})vs element ({})".format(ts, toPositive(toPositive(float(element["ts"])))))
                    if toPositive(toPositive(float(element["ts"])-ts)) < closestFrame["closeness"]:
                        #closest possible id = element.id
                        closestFrame["id"] = element["id"]
                        closestFrame["ma"] = element["ma"]
                        closestFrame["ts"] = element["ts"]
                        closestFrame["closeness"] = toPositive(toPositive(float(element["ts"])-ts)) 
                print_timestamps(DEPT=ts, POSE=closestFrame["ts"])
                #print("frame id({}), frame num({}, closeness({}))".format(frame_id,fnum,closestFrame["closeness"]))
                save_pose2(str(frame_id).zfill(5), closestFrame["ma"],folder)
                if ts < last_ts:
                    pipe.stop()
                    eof = True
                else:
                    last_ts = frames.get_timestamp()
                    frame_id = frame_id +1
        finally:
            if not eof:
                pipe.stop()

def processt265(t265,folder):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(t265)
    pipe.start(cfg)
    frame_id = 0

    last_ts = 0.0
    eof= False
    data = {}
    data["frames"] = []
    try:
        while True and not eof:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            print("t265 frames: {}".format(frames.size()))
            if pose:
                
                frame_id 
                frame_ts = frames.get_timestamp()
                print("frame-id: {}, -ts: {}, pose: {}".format(frame_id,frame_ts,pose))
                #m = createMatrix(pose.get_pose_data())
                #print(m)


                data["frames"].append({
                    'id':frame_id,
                    'ts':frame_ts,
                    'ma':createMatrix(pose.get_pose_data())
                }) 



                # check if end of file, or continue. 
                if frame_ts < last_ts:
                    pipe.stop()
                    eof = True
                else:
                    last_ts = frames.get_timestamp()
                    frame_id = frame_id +1
        with open("tmp/"+folder+"/t265_frames.json", "w") as outfile:
            json.dump(data,outfile)
    finally:
        if not eof:
            pipe.stop()

def processSensorData(folder_name):
    # Assumes two *.bag-files to be placed in stream folder before running. 
    # One with ending "T265.bag" and other with "D435.bag"
    #bagReader = o3d.t.io.RSBagReader()

    folder = "tmp/"+folder_name+"/stream/"
    D435 = ""
    for filename in os.listdir("tmp/"+folder_name+"/stream/"):
        ff = folder + filename
        
        print(ff)
        #if filename.endswith("T265.bag"):
            #T265 = T265Sensor(is_device=False, source_name=ff)
            #processt265(ff, folder_name)
        if filename.endswith("D435.bag"):
            #D435 = D435Sensor(is_device=False, source_name=ff)
            #bagReader.open(ff)
            D435 = ff
    
    o3DTimeStampTest(D435)
    pyRSTimeStampTest(D435)
    


    #creates a color and depth folder containing images. 
    #bagReader.save_frames("tmp/"+folder_name)
    #link depth and pose frames. 
    #processPose(D435, folder_name)
    
    

