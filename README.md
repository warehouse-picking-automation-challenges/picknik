![Team PickNik](http://picknik.io/PickNik_Logo3.png)

**Amazon Picking Challenge**

 - Website: [http://picknik.io](http://picknik.io)
 - [Team Members](https://bitbucket.org/cuamazonchallenge/profile/members)
 - [Timeline](https://docs.google.com/spreadsheets/d/1GG_j6BVir-J8VGwbU8RWDHA8kD8ZSeXrtlLtW9N851o/edit?usp=sharing)
 - [Item Data](https://docs.google.com/spreadsheets/d/1e0Fousz9TUxf9YHeVfnaKVgf06Z0WC50blMGBWJ9cp8/edit#gid=2088756835)

# Installation Instructions

The installation instructions were to long, and have been moved to INSTALL.md (no link provided because of BitBucket bug that does not allow you to go to latest version of the file)
    
## Architecture

![Pipeline](https://bytebucket.org/cuamazonchallenge/picknik/raw/2d87e203d681d303616f7a8abbdff190b20d33c6/picknik_main/docs/apc_picknik_pipeline.png?token=e5bb167125b3e41ad534c539614c37974db3cc31)

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

Baxter documentation has been moved ot BAXTER.md
	
### Setup Simulation of JACOB

Start roscore:

    roscore &

Start this separate to speed up launching:

    roslaunch jacob_control jacob_simulation.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch

Camera calibration:

    roslaunch picknik_main camera_calibration.launch

Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

Run APC Manager (main program) for JACOB in simulation

	roslaunch picknik_main jacob_apc.launch fake_execution:=1 fake_perception:=1 mode:=1 auto:=1

### Setup HARDWARE of Jacob

Start roscore:

    roscore &

Start controller:

	roslaunch jacob_control jacob_hardware.launch

Then calibrate the gantry by checking that it is safe to lower the robot to the bottom, then press RB on Xbox controller.

Rviz Visualizers of robot states and debug markers

    roslaunch picknik_main rviz.launch

Camera calibration:

    roslaunch picknik_main camera_calibration.launch
	
Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

Run APC Manager (main program) for JACOB on hardware

	roslaunch picknik_main jacob_apc.launch mode:=1

### Shelf to Robot Calibration

Adjust the values in ``config/apc_jacob.yaml`` for ``world_to_shelf_transform``:

To quickly view updates yaml settings:

     roslaunch picknik_main jacob_apc.launch mode:=13
	 
Calibrate z axis

    roslaunch picknik_main jacob_apc.launch fake_execution:=0 fake_perception:=1 mode:=24 pose:=z_calibration

Calibrate y axis

    roslaunch picknik_main jacob_apc.launch fake_execution:=0 fake_perception:=1 mode:=24 pose:=y_calibration

Calibrate x axis

    roslaunch picknik_main jacob_apc.launch fake_execution:=0 fake_perception:=1 mode:=24 pose:=x_calibration

### Shelf to Camera Calibration

Run the pre-recorded trajectory so that perception can build shelf model:

    roslaunch picknik_main jacob_apc.launch mode:=10 fake_perception:=1

### ROS Video Integration

Not in use at the moment...

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
		8. Open and close end effector(s)
		
		9. Record a calibration trajectory
		10. Playback calibration trajectory				
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
		21. Show experience database
		22. Test approach, lift, and retreat motion for random objects
		23. Unit tests for manipulation
		24. Go to pose pose:=NAME
        25. Test IK solver with simple pose request
		
	jump_to - which step in the manipulation pipeline to start on
	auto - whether to go into auto step mode, but does not allow trajectories to be executed without verification
	full_auto - whether to go into autonomous mode, without any human intervention even for execution
    order - which json file to use, defaults to orders/simple.json
	order_start - specify the index of the product to skip to, based on the ordering in the json file
	num_orders -how many products to pick from the order, 0 = all
	use_experience - whether to use cached planned (Lightning Database) or not
	saving_enabled - allow new plans to be saved to experience database
	debug - slower and more verbose
	fake_execution - runs at higher speeds and uses simulated controllers
	bin_id - specify which bin to look at, as a index number, starting at 0, e.g A=0, B=1
	
## Mesh Test

Simply displays all meshes from our mesh library in Rviz.

 - Visualization: Working Feb 11

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


### Debug Kinova Connection

See what is connected

    lsusb -t | grep ftdi

### Debug Gantry Connection

See what is connected

    ll /dev/zaber_vert

### Debug Xbox Controlelrs

See what is connected

    ll /dev/input/

Change device ID

    rosed jacob_control jacob_joy.launch
