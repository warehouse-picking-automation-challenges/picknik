![Team PickNik](http://picknik.io/PickNik_Logo3.png)

# Installation Instructions

## Install Manipulation Pipeline

Dave occasionally releases a new zip file with a lot of custom ROS code, that can be built into one workspace. Download the latest zip (~710MB) from here:

    Directory: http://picknik.io/secure
    User: picknik
    Password: sfd798asfiahfl89o7df980791324jhkls

Unzip the file and build

    unzip ws_picknik.zip
	cd ws_picknik
	rosdep install -y --from-paths src --ignore-src --rosdistro indigo
	catkin build

NOTE: before running ``catkin build`` you might need to use ``catkin config --install`` to have the ARPG code link correctly. Not sure yet.

To run simulation on your computer, add to your bashrc:

    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=`hostname -I`

To run from someone else's roscore, add to your bashrc:

    export ROS_MASTER_URI=http://THEIR_IP_ADRESS:11311
    export ROS_IP=`hostname -I`

Also, to reduce debug output, add to your bashrc:

    export ROSCONSOLE_CONFIG_FILE=~/ws_picknik/src/picknik/rosconsole.yaml
    export ROSCONSOLE_FORMAT='${severity} ${logger}: ${message}'

## Install Lu Ma's Perception Pipeline
## Tested in MacOSX 10.09, 10.10, Ubuntu 14.04, Cuda 6.0, Cuda 6.5, Cuda 7.0
If use CUDA 6.0, 6.5, please use gcc-46 as the cuda host compiler.

### Install CUDA and NVIDIA Driver. Make sure your cuda driver works correctlly by runing any of the cuda example demo:

### Install dependencies:
 1, install protobuf
     download:  https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gzinstall 
       
       run ./configure
       make -j3
       sudo make install all

       By default, the upper command will install the lib to /usr/local/lib/proto*, make sure all the protobuf libraries files is in /usr/lib/proto*. so you probability need to do
	   cp /usr/local/lib/proto* /usr/lib/
 2, install google ceres solver:
    sudo apt-get install libceres-dev 
    
 3, install google log and google flags:
    sudo apt-get install libgoogle-glog-dev
    sudo apt-get install libgflags-dev 
  
 4, also make sure you have opencv, boost  

### Install and Configure CoreDev
   1, git clone git@github.com:arpg/CoreDev.git
   
       https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gzinstall 
       run ./configure
       make -j3
       sudo make install all

By default, the upper command will install the lib to `/usr/local/lib/proto*`, make sure all the protobuf libraries files is in `/usr/lib/proto*`. so you probability need to do
	   `cp /usr/local/lib/proto* /usr/lib/`

Configurations
=======
   2, mkdir build
   
   3, cd build

   4, disable pangolin_video by doing:
	
       ccmake .
       set BUILD_PANGOLIN_GUI to OFF
       
   4.5 also make sure realsense is OFF by now.    

   5, go to the CMakeList.txt file under HAL/Applications, comment out everything expect for the SensorViewer:

      now you should be able to compile CoreDev by:
      cmake .
      make -j4
	
### Install kangaroo (for SDF fusion, ray casting):

    git clone git@github.com:arpg/Kangaroo.git
	cd kangaroo
	mkdir build
    cd build
	ccmake ..
	make -j
	
Notice: you may have some errors when building the examples, this is because we disabled the pangolin::video function before. just ignore it by now.

### Install wallaby (for grid sdf fusion and voxel hashing):
	1, install cudpp (optional, if you need voxel hashing)
	git clone git@github.com:cudpp/cudpp.git
	git submodule init
	git submodule update
	mkdir build
	cd build
	cmake ..
	make -j4

	now copy the following header files..
	cp /cudpp/cudpp/include/cudpp_config.h /usr/local/include/
	cp /cudpp/cudpp/include/cudpp_hash.h /usr/local/include/

	change permission from root to the user
	sudo chown -R robot cudpp_config.h
	sudo chown -R robot cudpp_hash.h

	2, install libglm (required)
	sudo apt-get install libglm-dev

	3, compile wallaby 
	cd/wallaby
	mkdir build
	cmake ..
	make -j4

### Install DDTR
	git init submodule
	git update submodule
	mkdir build
	cd build
	cmake ..
	make -j4


## Applications:
  ### RGFusion: 
    A Rolling Grid Implementation of the kinect fusion. Support any source of the input images including stereo, RGBD. 
    
  ### Amazon:
    A perception software for Amazon Picking Challenge. 
 

### Install Camera

    sudo apt-get install libavcodec-dev libudev-dev

    git clone https://github.com/arpg/Sophus.git
    cd ~/Sophus/
    mkdir build
    cd build
    cmake ..
    make -j4

    git clone https://github.com/arpg/miniglog.git
    cd miniglog/
    mkdir build
    cd build
    cmake ..
    make -j4

    git clone https://github.com/arpg/Pangolin
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    make -j4	

	git clone https://github.com/arpg/HAL.git
    cd HAL/
	git co -b ros_bridge origin/features/ros_bridge 
    mkdir build
    cd HAL/build
    ccmake ..
    make -j4

### Test Camera

     cd /home/dave/ros/HAL/build/Applications/SensorViewer
    ./SensorViewer -cam ros:[topics=/camera/image/rgb_raw]//
    ./SensorViewer -cam ros:[topics=/camera/image/rgb_raw+/camera/image/depth_raw]//
    ./SensorViewer -cam convert:[fmt=MONO8]//ros:[topics=/camera/image/rgb_raw+/camera/image/depth_raw]//
