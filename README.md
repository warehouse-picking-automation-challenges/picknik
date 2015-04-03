![Team PickNik](http://picknik.io/PickNik_Logo3.png)

**Amazon Picking Challenge**

 - Website: [http://picknik.io](http://picknik.io)
 - [Team Members](https://bitbucket.org/cuamazonchallenge/profile/members)
 - [Timeline](https://docs.google.com/spreadsheets/d/1GG_j6BVir-J8VGwbU8RWDHA8kD8ZSeXrtlLtW9N851o/edit?usp=sharing)
 - [Item Data](https://docs.google.com/spreadsheets/d/1e0Fousz9TUxf9YHeVfnaKVgf06Z0WC50blMGBWJ9cp8/edit#gid=2088756835)

## Install

### Perception Pipeline

TODO

### Manipulation Pipeline

Dave occasionally releases a new zip file with a lot of custom ROS code, that can be built into one workspace. Download the latest zip (~710MB) from here:

    Directory: http://picknik.io/secure
    User: picknik
    Password: sfd798asfiahfl89o7df980791324jhkls

Unzip the file and build

    unzip ws_picknik.zip
	cd ws_picknik
	rosdep install -y --from-paths src --ignore-src --rosdistro indigo
	catkin build

Also, to reduce debug output add the following to your bashrc:

    export ROSCONSOLE_CONFIG_FILE=~/ws_picknik/src/picknik/rosconsole.yaml
    export ROSCONSOLE_FORMAT='${severity} ${logger}: ${message}'

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

### Turn on Yale Controller

If needed.

    roslaunch open_hand_controller controller_manager.launch

Check for correct USB

    ls /dev/ttyUSB*

Check to make sure you have ``dialout`` group

    sudo adduser second_user dialout

## Start Robots

### Simulation of BAXTER

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizers of robot states and debug markers in differnet windows

    roslaunch picknik_main rviz_display.launch jacob:=false
    roslaunch picknik_main rviz.launch jacob:=false

Run either the real or a fake object recognition server

    roslaunch moveit_arpg_perception object_recognition.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 

Run APC Manager (main program) for BAXTER

    roslaunch picknik_main baxter_apc.launch mode:=1 fake_execution:=true
	
### Setup Hardware of BAXTER

Use Rethink's controllers

    roslaunch baxter_control baxter_hardware_rethink.launch

Rviz Visualizers of robot states and debug markers in differnet windows

    roslaunch picknik_main rviz_display.launch jacob:=false
    roslaunch picknik_main rviz.launch jacob:=false

Run either the real or a fake object recognition server

    roslaunch moveit_arpg_perception object_recognition.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 
	
Run APC Manager (main program) for BAXTER

    roslaunch picknik_main baxter_apc.launch mode:=1
	
### Setup Simulation of JACOB

Start roscore:

    roscore &

Start this separate to speed up launching:

    roslaunch jacob_control jacob_sim_hardware.launch

Rviz Visualizers of robot states and debug markers in differnet windows

    roslaunch picknik_main rviz_display.launch 
    roslaunch picknik_main rviz.launch

Run either the real or a fake object recognition server

    roslaunch moveit_arpg_perception object_recognition.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 

Run APC Manager (main program) for JACOB in simulation

	roslaunch picknik_main jacob_apc.launch fake_execution:=true

### Setup HARDWARE of Jacob

Start roscore:

    roscore &

Plugin in robot then choose one of the 2 control methods:

	roslaunch jacob_control jacob_control_old.launch  # uses velocity+position trajectory controller
	roslaunch jacob_control jacob_control.launch      # experimental ros_control method

Rviz Visualizers of robot states and debug markers in differnet windows (different windows)

    roslaunch picknik_main rviz_display.launch
    roslaunch picknik_main rviz.launch
	
Run either the real or a fake object recognition server

    roslaunch moveit_arpg_perception object_recognition.launch

A transform of the camera is needed

    roslaunch picknik_main camera_calibration.launch 

Run APC Manager (main program) for JACOB on hardware

	roslaunch picknik_main jacob_apc.launch mode:=1

### Joystick Control

Button Mapings

    A - Next Step
	B - Stop
	Back - Go home
	Xbox Button - AUTO


### PickNik Main Optional Arguments:

    mode - what program to run inside the apc_manager, defaults to 1
	  Available Modes:
	    1. Actual APC contest mode
		2. Train experience database mode / workspace analysis
		3. Test open close end effectors
		4. Visualize shelf
		5. Up and Down arms
		6. Verify shelf location
		7. SRDF: Get the current pose of the robot for the SRDF
		8. Go to goal bin pose
		9. Check if current state is in collision
		10. Plan to random valid locations
		11. Move camera to each bin location and request ObjectRecognitionServer
		12. Calibrate camera
		13. Record a calibration trajectory
		14. Go home
		15. Test grasp generator abilities and score results
		16. Test joint limits
		17. Test requesting preception results
		18. Record a bin observing trajectory
		19. Perceive (playback) a bin with camera
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
	auto - whether to go into autonomous mode, without any human intervention
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

### Random Planning

Has Baxter choose random poses with both arms and plan to them.

 - Visualization: UNTESTED
 - Hardware: UNTESTED

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch picknik_main rviz.launch

Planner

    roslaunch baxter_moveit_scripts random_planning.launch

### End Effector Test

Open and closes both end effectors

 - Visualization: Working Jan 30
 - Hardware: UNTESTED

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch picknik_main rviz.launch

Load meshes

    roslaunch baxter_moveit_scripts gripper_open_close.launch arm:=right

Notes: make sure you have a Robot STATE display added in Rviz.

### Grasp Generator

TODO

## Collaboration Notes

This just helps Dave know what to pull from when updating.

### Repos Andy commits to:

- cu_amazon
- open_hand_controller
- baxter_common
- baxter_ssh

### Repos Jorge commits to:

- cu_amazon
- ?
