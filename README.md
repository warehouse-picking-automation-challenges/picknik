![Team PickNik](http://picknik.io/PickNik_Logo3.png)

**Amazon Picking Challenge**

 - Website: [http://picknik.io](http://picknik.io)
 - [Team Members](https://bitbucket.org/cuamazonchallenge/profile/members)
 - [Timeline](https://docs.google.com/spreadsheets/d/1GG_j6BVir-J8VGwbU8RWDHA8kD8ZSeXrtlLtW9N851o/edit?usp=sharing)
 - [Item Data](https://docs.google.com/spreadsheets/d/1e0Fousz9TUxf9YHeVfnaKVgf06Z0WC50blMGBWJ9cp8/edit#gid=2088756835)

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

### Install dependencies for node (optional, not require):
   1, install zmq
   
       sudo apt-get install libzmq3-dev

   2, install gui version of cmake
   
       sudo apt-get install cmake-curses-gui

   3, add cpp binders for zmq
   
       git clone git@github.com:zeromq/zmqpp.git
 
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
    
## Architecture

![Pipeline](https://bytebucket.org/cuamazonchallenge/picknik/raw/3f6788816ad7733051493f55f142655b2702adb1/picknik_main/docs/apc_picknik_pipeline.png?token=ef4e18838e57f4cb97be4ecff9691b3740dd8a8e)

## Run

### Generate Mock Amazon order

Create a simulated bin inventory and random order by running

    rosrun picknik_main random_orders.py order.json

Note that you can repeat experiments setting the used seed, and modify
the likelyhood of the number of objects per bin too:

    usage: random_orders.py [-h] [--probabilites PROBABILITES] [--seed SEED]
                            filename

    positional arguments:
      filename              filename to save the json order to

    optional arguments:
      -h, --help            show this help message and exit
      --probabilites PROBABILITES, -p PROBABILITES
                            Quote delimited list of probabilites. Eg "[0.5, 0.2,
                            0.2, 0.1]"
      --seed SEED, -s SEED


### Sort order in terms of priority

Take an order, sort it according to the (decreasing) expected
punctuation we might get from each object, and write a new sorted
order.

    rosrun picknik_main sort_order.py order.json sorted_order.json

The expected score for each object is the product of the probability
of grasping it correctly (tweak them in
`picknik_main/orders/items_data.csv`) times the score for doing it
right (depends on the number of objects in the bin). Then, if there're
multiple objects in the bin, we remove the product of the number of
objects in the bin times the probablity of removing an object we
shouldn't have touched and the score we'd lose.

Its help documentation:

    usage: sort_order.py [-h] input_filename output_filename

    positional arguments:
      input_filename   filename for the json order
      output_filename  filename for the sorted order

    optional arguments:
      -h, --help       show this help message and exit

## Start Realsense Camera

Start driver on computer where USB3 camera is plugged in

    roslaunch realsense_camera realsense_camera.launch

Testing

    rosrun image_view image_view image:=/camera/image/rgb_raw

## Start Primesense Camera

    roslaunch openni_launch openni.launch depth_registration:=true

## Start Robots

### Simulation of BAXTER

Start roscore:

    roscore &
	
Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch jacob:=false

Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch jacob:=false

Run APC Manager (main program) for BAXTER

    roslaunch picknik_main baxter_apc.launch mode:=1 fake_execution:=true
	
### Setup Hardware of BAXTER

Enable Baxter:

    rostopic pub -1 /robot/set_super_enable std_msgs/Bool True	

Use Rethink's controllers

    roslaunch baxter_control baxter_hardware_rethink.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch jacob:=false

Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch jacob:=false
	
Run APC Manager (main program) for BAXTER

    roslaunch picknik_main baxter_apc.launch mode:=1
	
### Setup Simulation of JACOB

Start roscore:

    roscore &

Start this separate to speed up launching:

    roslaunch jacob_control jacob_simulation.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch

Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 

Run APC Manager (main program) for JACOB in simulation

	roslaunch picknik_main jacob_apc.launch fake_execution:=true mode:=1 auto:=1

### Setup HARDWARE of Jacob

Start roscore:

    roscore &

Start controller: WARNING - **THIS STARTS THE CALIBRATION ROUTINE AND WILL MOVE ROBOT INTO POSSIBLE COLLISION WITH GANTRY!!**

	roslaunch jacob_control jacob_hardware.launch

Rviz Visualizers of robot states and debug markers

    roslaunch picknik_main rviz.launch
	
Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 

Run APC Manager (main program) for JACOB on hardware

	roslaunch picknik_main jacob_apc.launch mode:=1

### ROS Video Integration

    rosrun image_view image_view image:=/camera/image/rgb_raw

### Jaco Joystick Control

Button Mapings

    1 - Next (not implemented in ros_control yet)
	2 - Disable actuators (0 PID gains)
	3 - Disable control (turn off PC controller)

### XBox Joystick Control

Button Mapings

    A - Next Step in Manipulation Pipeline
	B - Motion Stop (switch controllers to manual mode)
	Y - Stop Manipulation Pipeline
	X - Motion start (switch controllers to trajectory mode)
	Back - Go home
	Xbox Button - AUTO
	RB - Calibrate gantry (home it)
	Up/Down Axis Stick Left - manually move gantry

### PickNik Main Optional Arguments:

    mode - what program to run inside the apc_manager, defaults to 1
	  Available Modes:
	    1. Actual APC contest mode
		2. GO home
		3. GO goal bin
		4. GO each bin location and request ObjectRecognitionServer
		5. GO up and down with arms
		6. GO to random valid locations
		7. GO to verify shelf locaiton
		8. Open close end effectors
		
		9. Playback calibration trajectory
		10. Record a calibration trajectory
		11. Record a bin observing trajectory
		12. Playback bin observing trajectory (perceive)

        13. Visualize shelf
		14. SRDF: Get the current pose of the robot for the SRDF
		15. Check if current state is in collision
		16. Test grasp generator abilities and score results
		17. Test joint limits
		18. Test requesting preception results
		19. Train experience database mode / workspace analysis

        20. GO in and out of bin
		
	jump_to - which step in the manipulation pipeline to start on
	  Steps: NOT CORRECT ANYMORE
	    0. Move to initial position
		1. Open end effectors
		2. Generate and choose grasp
		3. Setting the-grasp
		4. Get pre-grasp by generateApproachPath()
		5. N/A
		6. Moving to pre-grasp position
		7. Cartesian move to the-grasp position
		8. Grasping
		9. Lifting product UP slightly
		10. Moving BACK to pre-grasp position
		11. Moving back to INITIAL position
		12. Releasing product
	auto - whether to go into auto step mode, but does not allow trajectories to be executed without verification
	full_auto - whether to go into autonomous mode, without any human intervention even for execution
    order - which json file to use, defaults to orders/simple.json
	order_start - specify the index of the product to skip to, based on the ordering in the json file
	num_orders -how many products to pick from the order, 0 = all
	use_experience - whether to use cached planned (Lightning Database) or not
	saving_enabled - allow new plans to be saved to experience database
	show_database - whether to pause between motion plans and show all the saved paths (debug)
	debug - slower and more verbose

## Working Tests

Document here all roslaunch files for testing/verifying various parts of the system. Also specify if you've checked it in
simulation, hardware, or both.

### Loading meshes

Simply displays all meshes from our mesh library in Rviz.

 - Visualization: Working Feb 11
 - Hardware: N/A

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch picknik_main mesh_publisher_rviz.launch

Load meshes

    rosrun picknik_main mesh_publisher

## Debugging Tools

### Record CSV Files of Controller Data

    rosrun ros_control_boilerplate controller_state_to_csv /home/dave/ros/combined_analysis/jaco_trajectory_1.csv /jacob/kinova/velocity_trajectory_controller/state
	rosrun ros_control_boilerplate controller_state_to_csv /home/dave/ros/combined_analysis/gantry_trajectory_1.csv /jacob/zaber/velocity_trajectory_controller/state


### See Kinova USB Hub

    lsusb -t | grep ftdi

## See Gantry USB Device

    ll /dev/zaber_vert
