from sensorReader import readSensor2
from mergeClouds import trajectory_merger




#readSensor.processSensorData("t01")
#camera to point cloud
#camReader.readCam()
#camReader2.readD435()

#camReader2.processD435("tmp/stream/20210319_004514.bag", "tmp/t1/")
#camReader2.processD435("tmp/stream/20210319_004540.bag", "tmp/t2/")
#camReader2.processD435("tmp/stream/20210324_190511.bag", "tmp/r1/")

readSensor2.processBag("tmp/[folder]/stream/D435.bag", "tmp/[folder]/stream/T265.bag", "[folder]")


#test.createRGBD()
#trajectory_merger.createPointCloudsFromDepth("t02",1000)
#trajectory_merger.mergeCloud("t02")
#trajectory_merger.createAlongPathRaw("t02")
#trajectory_merger.createConesAlongPath("t02")

#trajectory_merger.checkShift("t02",1,5)
#trajectory_merger.checkShift("t02",1,20)
#trajectory_merger.checkShift("t02",1,50)
#trajectory_merger.checkShift("t02",1,100)