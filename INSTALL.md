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

### Install CUDA and NVIDIA Driver

* Tested in MacOSX 10.09, 10.10, Ubuntu 14.04, Cuda 6.0, Cuda 6.5, Cuda 7.0

If use CUDA 6.0, 6.5, please use gcc-46 as the cuda host compiler.

Recommended: [Cuda 7](https://developer.nvidia.com/cuda-downloads)

Switch virtual terminal (Ctrl Alt F1)

    sudo service lightdm stop
	~/Downloads
	chmod -x CUDA_FILE
	./CUDA_FILE

Say yes to everything... restart.


Make sure your cuda driver works correctly by runing any of the cuda example demo:

    cd ~/NVIDIA_CUDA-7.0_Samples/
	make

[Getting Started Documentation](http://www.google.com/url?q=http%3A%2F%2Fdeveloper.download.nvidia.com%2Fcompute%2Fcuda%2F7_0%2FProd%2Fdoc%2FCUDA_Getting_Started_Linux.pdf&sa=D&sntz=1&usg=AFQjCNH-aytZIB1ufyiMTTi-okbCJXSYrg)

### Install dependencies:

 0, pre-request:
 
    sudo apt-get install autoconf2.13 libglm-dev libtinyxml2-dev

 1, install protobuf
 
    git clone https://github.com/google/protobuf.git
	cd protobuf
    ./autogen.sh
    ./configure
    make -j3
    sudo make install all
    cp /usr/local/lib/libproto* /usr/lib/
    
 2, OPTIONAL install google ceres solver: (you don't need to do this if you have ros installed)
 
    sudo apt-get install libceres-dev 
    
 3, OPTIONAL install google log and google flags: (you don't need to do this if you have ros installed)
 
    sudo apt-get install libgoogle-glog-dev
    sudo apt-get install libgflags-dev 
  
 4, also make sure you have opencv, boost (you don't need to do this if you have ros installed)

### Install CoreDev

    cd CoreDev
    mkdir build
    cd build
	cmake ..
    make -j4

### Install kangaroo (for SDF fusion, ray casting):

	cd kangaroo
	mkdir build
    cd build
	cmake ..
	make -j4
	
### Install wallaby (for grid sdf fusion and voxel hashing):

	cd/wallaby
	mkdir build
	cmake ..
	make -j4

### Install DDTR

	git submodule init
	git submodule update
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
