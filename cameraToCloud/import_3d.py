from pyntcloud import PyntCloud
from PIL import Image
import numpy as np
import pandas as pd
import os
import imageio

save_folder = "reconstruction/test/"

class reconstructor():
    def save_pointCloud(id,cloud):
        print("Start saving point cloud file")
        os.makedirs(save_folder + "cloud/", exist_ok = True)
        file_cloud = PyntCloud(pd.DataFrame(
            data=cloud,
            columns=["x","y","z"]
        ))
        file_cloud.to_file(save_folder +"cloud/"+ id + ".ply")
        print("save complete")

    def save_image(id,img):
        os.makedirs(save_folder + "img/",  exist_ok = True)
        #img.save(save_folder +"img/"+ id + ".png")
        imageio.imwrite(save_folder +"img/"+ id + ".png", img)

    def save_pose(id,data):
        pose = data.get_pose_data()
        os.makedirs(save_folder + "csv/", exist_ok = True)
        txt = save_folder + "csv/"+id + ".csv"
        print("Frame #{}".format(data.frame_number))
        print("Position: {}".format(pose.translation))
        print("Velocity: {}".format(pose.velocity))
        print("Rotation: {}".format(pose.rotation))
        f= open(txt,"w+")
        f.write("vecX,vecY,vecZ,velX,velY,velZ,rotW,rotZ,rotX,rotY\n")
        f.write(""+str(pose.translation.x)+","+str(pose.translation.y)+","+str(pose.translation.z)+","+
                str(pose.velocity.x)+","+str(pose.velocity.y)+","+str(pose.velocity.z)+","+
                str(pose.rotation.w)+","+str(pose.rotation.z)+","+str(pose.rotation.x)+","+str(pose.rotation.y))
        f.close
        #np.savetxt("foo.csv", matrix, delimiter=",")

