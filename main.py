from sensorReader import readSensor2
from mergeClouds import trajectory_merger





#Main function, replace [folder] to what folder you have used.
#readSensor2.processBag("tmp/[folder]/stream/D435.bag", "tmp/[folder]/stream/T265.bag", "[folder]")


#test.createRGBD()

#Create point cloud and translate using raw pose data. 
# output is stored in cloudRaw and cloudShifted. 
# 1000 is the factor of downsampling with the algorithm only storing every k'th point, 
# starting on 0, k, 2k, 3k, ... This to make it easier to process and analyse results. 
trajectory_merger.createPointCloudsFromDepth("t02",1000)

