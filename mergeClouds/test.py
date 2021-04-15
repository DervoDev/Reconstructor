import json
import numpy as np
import open3d as o3d

def createRGBD():
    color = o3d.io.read_image("G:/git/Reconstructor/tmp/t02/color/00000.jpg")
    depth = o3d.io.read_image("G:/git/Reconstructor/tmp/t02/depth/00000.png")
    print(color)
    print(depth)
    a = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=True)
    print (a)

# borrowed from Tsykunov (2020)
def apply_transformation(transformation, points):
        """

        :param transformation:
        :param points:
        :return:
        """
        if transformation is None or points is None:
            return None
        else:
            coordinates = np.hstack((points, np.ones((points.shape[0], 1))))
            return (transformation @ coordinates.T).T[:, :-1]

def mergeClouds(folder, num=5, bracket=False): #num is the number of clouds to be merged together
    ImportedCloud = []   # will contain a list of point clouds without placement
    ImportedPos = []    # will contain a list of imported positions. 
    cloud = []          # will contain all points with a shared world origin
    bracketTransfer = np.load('configs/T265toD435.npy') # matrix-vector between cameras placed on a bracket 
    config = json.load("tmp/"+folder+"/intrinsic.json")
    intr = o3d.io.read_pinhole_camera_intrinsic("tmp/"+folder+"/intrinsic.json")


    for i in range(num):
        d = o3d.io.read_image("tmp/"+folder+"/depth/"+ str(i).zfill(5)+".png")
        c = o3d.geometry.PointCloud.create_from_depth_image(d,intr,depth_scale=config["depth_scale"])
        ImportedCloud.append(c)

        ImportedPos.append(np.load("tmp/"+folder+"/pose/"+str(i).zfill(5)+".npy"))

        cloud.append(apply_transformation(ImportedPos[i],ImportedCloud[i]))
    

frame-id: 0, -ts: 1618325504413.5168, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235BC028EF0>, size:00005
frame-id: 1, -ts: 1618325504438.4768, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235BC028A30>, size:00005
frame-id: 2, -ts: 1618325504478.4128, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235AA7A4570>, size:00005
frame-id: 3, -ts: 1618325504508.3726, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235BC028EF0>, size:00005
frame-id: 4, -ts: 1618325504538.3245, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FC4E70>, size:00005
frame-id: 5, -ts: 1618325504573.2673, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235AA7A4570>, size:00005
frame-id: 6, -ts: 1618325504608.2104, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA270>, size:00005
frame-id: 7, -ts: 1618325504638.1624, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA530>, size:00005
frame-id: 8, -ts: 1618325504673.1064, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA670>, size:00005
frame-id: 9, -ts: 1618325504708.0493, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA7B0>, size:00005
frame-id: 10, -ts: 1618325504738.0005, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA3B0>, size:00005
frame-id: 11, -ts: 1618325504772.9443, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDA530>, size:00005
frame-id: 12, -ts: 1618325504807.8884, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDAB30>, size:00005
frame-id: 13, -ts: 1618325504837.8394, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDAC70>, size:00005
frame-id: 14, -ts: 1618325504877.7754, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDADB0>, size:00005
frame-id: 15, -ts: 1618325504907.7266, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDAEF0>, size:00005
frame-id: 16, -ts: 1618325504947.6626, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDAC70>, size:00005
frame-id: 17, -ts: 1618325504972.6216, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB170>, size:00005
frame-id: 18, -ts: 1618325505007.5657, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB2B0>, size:00005
frame-id: 19, -ts: 1618325505037.5166, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB3F0>, size:00005
frame-id: 20, -ts: 1618325505072.4604, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB530>, size:00005
frame-id: 21, -ts: 1618325505107.4036, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB670>, size:00005
frame-id: 22, -ts: 1618325505142.3464, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB7B0>, size:00005
frame-id: 23, -ts: 1618325505172.2986, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCB8F0>, size:00005
frame-id: 24, -ts: 1618325505207.2424, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBA30>, size:00005
frame-id: 25, -ts: 1618325505242.1855, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBB70>, size:00005
frame-id: 26, -ts: 1618325505272.1375, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBCB0>, size:00005
frame-id: 27, -ts: 1618325505312.0725, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBDF0>, size:00005
frame-id: 28, -ts: 1618325505342.0244, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBF30>, size:00005
frame-id: 29, -ts: 1618325505371.9753, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FCBCB0>, size:00005
frame-id: 30, -ts: 1618325505406.9194, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB1F0>, size:00005
frame-id: 31, -ts: 1618325505441.8623, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB330>, size:00005
frame-id: 32, -ts: 1618325505471.8145, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB470>, size:00005
frame-id: 33, -ts: 1618325505506.7573, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB5B0>, size:00005
frame-id: 34, -ts: 1618325505546.6934, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB6F0>, size:00005
frame-id: 35, -ts: 1618325505571.6526, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB830>, size:00005
frame-id: 36, -ts: 1618325505606.5967, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDB970>, size:00005
frame-id: 37, -ts: 1618325505646.5315, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBAB0>, size:00005
frame-id: 38, -ts: 1618325505671.4917, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBBF0>, size:00005
frame-id: 39, -ts: 1618325505706.4346, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBD30>, size:00005
frame-id: 40, -ts: 1618325505741.4106, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBE70>, size:00005
frame-id: 41, -ts: 1618325505771.3616, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBFB0>, size:00005
frame-id: 42, -ts: 1618325505806.3054, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDBD30>, size:00005
frame-id: 43, -ts: 1618325505841.2485, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF270>, size:00005
frame-id: 44, -ts: 1618325505871.2004, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF3B0>, size:00005
frame-id: 45, -ts: 1618325505906.1436, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF4F0>, size:00005
frame-id: 46, -ts: 1618325505946.0796, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF630>, size:00005
frame-id: 47, -ts: 1618325505971.0386, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF770>, size:00005
frame-id: 48, -ts: 1618325506005.9824, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF8B0>, size:00005
frame-id: 49, -ts: 1618325506040.9255, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDF9F0>, size:00005
frame-id: 50, -ts: 1618325506070.8774, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDFB30>, size:00005
frame-id: 51, -ts: 1618325506110.8125, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDFC70>, size:00005
frame-id: 52, -ts: 1618325506140.7644, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDFDB0>, size:00005
frame-id: 53, -ts: 1618325506175.7073, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDFEF0>, size:00005
frame-id: 54, -ts: 1618325506205.6594, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FDFC70>, size:00005
frame-id: 55, -ts: 1618325506240.6023, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE11B0>, size:00005
frame-id: 56, -ts: 1618325506270.5544, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE12F0>, size:00005
frame-id: 57, -ts: 1618325506305.4983, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1430>, size:00005
frame-id: 58, -ts: 1618325506340.373, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1570>, size:00005
frame-id: 59, -ts: 1618325506375.317, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE16B0>, size:00005
frame-id: 60, -ts: 1618325506405.268, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE17F0>, size:00005
frame-id: 61, -ts: 1618325506440.212, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1930>, size:00005
frame-id: 62, -ts: 1618325506475.155, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1A70>, size:00005
frame-id: 63, -ts: 1618325506505.107, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1BB0>, size:00005
frame-id: 64, -ts: 1618325506540.0498, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1CF0>, size:00005
frame-id: 65, -ts: 1618325506574.993, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1E30>, size:00005
frame-id: 66, -ts: 1618325506604.9448, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1F70>, size:00005
frame-id: 67, -ts: 1618325506639.888, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE1CF0>, size:00005
frame-id: 68, -ts: 1618325506674.8318, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4230>, size:00005
frame-id: 69, -ts: 1618325506704.784, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4370>, size:00005
frame-id: 70, -ts: 1618325506739.7268, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE44B0>, size:00005
frame-id: 71, -ts: 1618325506774.671, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE45F0>, size:00005
frame-id: 72, -ts: 1618325506804.6218, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4730>, size:00005
frame-id: 73, -ts: 1618325506839.566, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4870>, size:00005
frame-id: 74, -ts: 1618325506874.5088, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE49B0>, size:00005
frame-id: 75, -ts: 1618325506904.4607, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4AF0>, size:00005
frame-id: 76, -ts: 1618325506939.4038, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4C30>, size:00005
frame-id: 77, -ts: 1618325506974.3604, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4D70>, size:00005
frame-id: 78, -ts: 1618325507004.3115, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4EB0>, size:00005
frame-id: 79, -ts: 1618325507039.2556, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE4C30>, size:00005
frame-id: 80, -ts: 1618325507074.1985, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7170>, size:00005
frame-id: 81, -ts: 1618325507104.1506, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE72B0>, size:00005
frame-id: 82, -ts: 1618325507139.0935, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE73F0>, size:00005
frame-id: 83, -ts: 1618325507174.0376, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7530>, size:00005
frame-id: 84, -ts: 1618325507203.9885, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7670>, size:00005
frame-id: 85, -ts: 1618325507238.9324, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE77B0>, size:00005
frame-id: 86, -ts: 1618325507273.8755, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE78F0>, size:00005
frame-id: 87, -ts: 1618325507308.8193, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7A30>, size:00005
frame-id: 88, -ts: 1618325507338.7705, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7B70>, size:00005
frame-id: 89, -ts: 1618325507373.7144, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7CB0>, size:00005
frame-id: 90, -ts: 1618325507403.6655, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7DF0>, size:00005
frame-id: 91, -ts: 1618325507438.6094, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7F30>, size:00005
frame-id: 92, -ts: 1618325507473.5525, pose: <pyrealsense2.pyrealsense2.pose_frame object at 0x00000235C1FE7CB0>, size:00005

    
