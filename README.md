Requirements: 
- Python 3.7 - 64 bit. (developed in 3.7.9) 

Deployment:
- python -m venv env
- env\scripts\activate
- pip install -r requirements.txt

Example data available at: 
- Download D435.bag & T265.bag from https://drive.google.com/drive/folders/18B_08fYDP-PGJVgCITZreCvBfy1RwKmO?usp=sharing
- create folder tmp/[folder]/stream
- place files in tmp/[folder]/stream/

Run
- env\scripts\activate
- modify main.py with what [folder] you want to use
- python main.py
- Install Open3D via downloading from source git clone --recursive https://github.com/intel-isl/Open3D
- in examples/python/reconstruction_system open make_fragments.py and add the two functions from my Open3d/open3d.py. Rename the function with conflicting name, you want to use the one from open3d.py
- in open3d.py you find the callable you can use to activeate the reconstruction system. more info on that here: http://www.open3d.org/docs/release/tutorial/reconstruction_system/system_overview.html

