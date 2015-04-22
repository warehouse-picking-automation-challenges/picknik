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

## Install Perception Pipeline

Tested in MacOSX 10.09, 10.10, Ubuntu 14.04, Cuda 6.0, Cuda 6.5, Cuda 7.0

If use CUDA 6.0, 6.5, please use gcc-46 as the cuda host compiler.

### Install CUDA and NVIDIA Driver

Make sure your cuda driver works correctlly by runing any of the cuda example demo:

### Install dependencies:

 0, pre-request:
 
    sudo apt-get install autoconf2.13 libglm-dev libtinyxml2-dev

 1, install protobuf
 
    https://github.com/google/protobuf.git
    ./autogen.sh
    run ./configure
    make -j3
    sudo make install all
    cp /usr/local/lib/libproto* /usr/lib/
    
 2, install google ceres solver: (you don't need to do this if you have ros installed)
 
    sudo apt-get install libceres-dev 
    
 3, install google log and google flags: (you don't need to do this if you have ros installed)
 
    sudo apt-get install libgoogle-glog-dev
    sudo apt-get install libgflags-dev 
  
 4, also make sure you have opencv, boost (you don't need to do this if you have ros installed)  

### Install CoreDev

    cd CoreDev
    mkdir build
    cd build
    make -j4

### Install kangaroo (for SDF fusion, ray casting):

	cd kangaroo
	mkdir build
    cd build
	ccmake ..
	make -j
	
### Install wallaby (for grid sdf fusion and voxel hashing):

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
 
### Install Camera (you dont need to do this if you followed the upper directions)

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
