Requirements: 
- Python 3.7 - 64 bit. (developed in 3.7.9) 

Deployment:
- python -m venv env
- env\scripts\activate
- pip install -r requirements.txt

Example data available at: 
- Download D435.bag & T265.bag from https://drive.google.com/drive/folders/18B_08fYDP-PGJVgCITZreCvBfy1RwKmO?usp=sharing
- create folder tmp/[folder]/stream/
- place files in tmp/[folder]/stream/





Run
- env\scripts\activate
- modify main.py with what [folder] you want to use
- python main.py

The data has now been created, you will be left with three folders in the [folder]: color, depth and pose. 

If you wnat to create a 3d-model out of it, do the following:
- Install Open3D via downloading from source git clone --recursive https://github.com/intel-isl/Open3D
- in examples/python/reconstruction_system open make_fragments.py and add the two functions from my Open3d/open3d.py. Rename the function with conflicting name, you want to use the one from open3d.py
- in open3d.py you find the callable you can use to activeate the reconstruction system. more info on that here: http://www.open3d.org/docs/release/tutorial/reconstruction_system/system_overview.html


If you want to manually look at the data do the following: 
make sure that you ran main.py with "trajectory_merger.createPointCloudsFromDepth(folder,400)" active. This creates two folders: cloudRaw and cloudShifted. Each folder creates a point cloud for each frame in the bag, cloudRaw have the point cloud stationary as is, and cloudShifted have shifted the point cloud so they all share the world origin. 
