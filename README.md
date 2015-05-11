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

## Hardware

 - [Kinova Jacob](kinovarobotics.com/products/jaco-robotics/)
 - [Zaber](zaber.com/products/product_detail.php?detail=A-LST1000D)
 - Asus Xtion Pro
 
## Run

### Generate Mock Amazon order

Create a simulated bin inventory and random order by running

    rosrun picknik_main random_orders.py `rospack find picknik_main`/orders/random.json

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

## Run DDTR Perception Pipeline

### Run Shelf Calibration

First move robot to calibration pose:

    roslaunch picknik_main jacob_apc.launch fake_perception:=1 mode:=9 pose:=camera_view
	 
Then change robot to manual mode, move up and down using joystick and this node:

    roslaunch picknik_perception perception_server_ddtr_calibrate.launch

You should be able to see the live camera images in the top left GUI.

 - Press key 'f' to start Kinect Fusion.
 - Press 'space' to stop building the shelf model when you got the 3D model you want.
 - Press the button 'Clean Host (Del SDFs/Poses files)' on the control panel (left side) of the application to clean the old shelf model files.
 - Press 'Save Pose'
 - Press 'Save SDF' to save the latest 3D model of the shelf.
 
Now you are safe the leave the application and run the Amazon app .

## Start Robots

Baxter documentation has been moved ot BAXTER.md
	
### Run Simulation of JACOB

Start roscore:

    roscore &

Start this separate to speed up launching:

    roslaunch jacob_control jacob_simulation.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch

Camera calibration:

    roslaunch picknik_perception multi_xtion_calibrate.launch

Run fake object recognition server:

	roslaunch picknik_perception perception_server_fake.launch

Run APC Manager (main program) for JACOB in simulation

	roslaunch picknik_main jacob_apc.launch fake_execution:=1 fake_perception:=1 mode:=1 full_auto:=1

### Run HARDWARE of Jacob

Start roscore and sync times

    roscore &
	sudo ntpdate pool.ntp.org

Start controller:

	roslaunch jacob_control jacob_hardware.launch

Then calibrate the gantry by checking that it is safe to lower the robot to the bottom, then press RB on Xbox controller.

Rviz Visualizers of robot states and debug markers

    roslaunch picknik_main rviz.launch

Start cameras (on correct computer):

    sudo ntpdate pool.ntp.org 
    roslaunch picknik_perception multi_xtion.launch

Camera calibration:

    roslaunch picknik_perception multi_xtion_calibrate.launch
 
Run APC Manager (main program) for JACOB on hardware. It will wait for perception server to start (below)

	roslaunch picknik_main jacob_apc.launch mode:=1

### Run Perception
   
    roslaunch picknik_perception perception_server_ddtr.launch 

Arguments

    -wsp gives the dir of the source code of DDTR
	-mode select the running mode of the application.

Keys

 - Press SPACE to switch between two views

You might also need the offset transform hack:

    roslaunch picknik_perception tf_keyboard_perception_offset.launch

### Move Robot to Shutdown Mode

Safe for power-off:

    roslaunch picknik_main jacob_apc.launch fake_perception:=1 mode:=9 pose:=collapsed

### Shelf to Robot Calibration

Adjust the values in ``config/apc_jacob.yaml`` for ``world_to_shelf_transform``:

To quickly view updates yaml settings:

    roslaunch picknik_main jacob_apc.launch mode:=40
	 
Calibrate z axis

    roslaunch picknik_main jacob_apc.launch fake_perception:=1 mode:=9 pose:=z_calibration

Calibrate y axis

    roslaunch picknik_main jacob_apc.launch fake_perception:=1 mode:=9 pose:=y_calibration

Calibrate x axis

    roslaunch picknik_main jacob_apc.launch fake_perception:=1 mode:=9 pose:=x_calibration

### Shelf to Robot Calibration - New Method

Get world to target

    steve?

Show target to shelf

    roslaunch picknik_perception tf_keyboard_shelf_offset.launch

### Jaco Joystick Control

Button Mapings

    1 - Enable actuators EXPERIMENTAL aka it doesn't work
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

        MOVEMENT
		2. GO home
		3. GO goal bin
		4. GO each bin location and request ObjectRecognitionServer
		5. GO up and down with arms
		6. GO to random valid locations
		7. GO to verify shelf locaiton
		8. GO to open and close end effector(s)
		9. GO to pose pose:=NAME
        10. GO in and out of bin
		11. GO in circle for calibration

        TESTING
		16. Test grasp generator abilities and score results
		17. Test joint limits
		18. Test requesting preception results, using bin id:=[1-12]
		19. Test requesting preception results from each bin in a loop
		20. Test variable grasp positions
		22. Test approach, lift, and retreat motion for random objects
		23. UNIT TESTS for manipulation		
        25. Test IK solver with simple pose request
		26. Unit test for perception communication
		27. Test planning ONLY from a shelf bin to the goal bin

        TRAJECTORY HANDLING
		30. Record a calibration trajectory, using id:=[0 left |1 right]
		31. Playback calibration trajectory, using id:=[0 left |1 right]
		32. Record a bin observing trajectory, using id:=[1-12]
		33. Playback bin observing trajectory, using id:=[1-12]
		34. Playback waypoint path specified in a csv
		
        DEBUGGING
        40. Visualize shelf
        41. SRDF: Get the current pose of the robot for the SRDF
		42. Check if current state is in collision		

        EXPERIENCE PLANNING
		50. Train experience database mode / workspace analysis
		51. Show experience database		
		
	jump_to - which step in the manipulation pipeline to start on
	auto - whether to go into auto step mode, but does not allow trajectories to be executed without verification
	full_auto - whether to go into autonomous mode, without any human intervention even for execution
    order - which json file to use, defaults to orders/simple.json
	order_start - specify the index of the product to skip to, based on the ordering in the json file
	num_orders -how many products to pick from the order, 0 = all
	debug - slower and more verbose
	fake_execution - runs at higher speeds and uses simulated controllers
	fake_perception - do not use perception server
	id - specify which bin to look at, as a index number, starting at 0, e.g A=1, B=2,
	     or id of camera 0 (left) or 1 (right)
	
## Mesh Test

Simply displays all meshes from our mesh library in Rviz.

 - Visualization: Working Feb 11

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch picknik_main mesh_publisher_rviz.launch

Load meshes

    rosrun picknik_main mesh_publisher

## Running ROS -> HAL Bridge

HAL git branch features/ros_bridge includes a ROS connector to consume imagery from ROS

    Run like: ./SensorViewer -cam ros:[topics=/camera/image_raw+/camera/depth/image_raw]

    This will publish a Protobuf with one image per topic. The ROS bridge knows about mono16, bgr8, rgb8, mono8, and 32FC1 images.

    Restrictions:

	Images must be 640x480, this is a statically compiled thing
    	ROS_MASTER must be set in the environment since the command-line args are buried within HAL
	
## Debugging Tools

### Record CSV Files of Controller Data

    rosrun ros_control_boilerplate controller_state_to_csv /home/dave/ros/combined_analysis/jaco_trajectory_1.csv /jacob/kinova/velocity_trajectory_controller/state
	rosrun ros_control_boilerplate controller_state_to_csv /home/dave/ros/combined_analysis/gantry_trajectory_1.csv /jacob/zaber/velocity_trajectory_controller/state



## Debugging USB Things

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

### Debug ASUS Xtion

Xtion backend process XnSensorServer stopped, 'ps -aux' to find PID, kill -9 it, then restart
    
When one runs 'strace /usr/bin/Sample-NiSimpleViewer', recvmsg() calls will fail, indicating a lack of comm with the Xtion backend. 

### Reset USB stack without a reboot

WARNING: If your keyboard is plugged in via this USB node, you will lose control of the console
As root:

    cd /sys/bus/pci/drivers/xhci_hcd
    ls -l
    echo "0000:00:14.0" > unbind

This will unbind the driver from the hardware. To reset:

    echo "0000:00:14.0" > bind

This will re-enumerate all USB devices, including running udev rules, etc.


### Syncing Time of Computers

    # Check the offset of time of a IP address
    alias btimeoffset="ntpdate -q 128.138.224.231"

    ## Sync computer to standard syncing server
    alias syncmytime="sudo ntpdate pool.ntp.org"
  
### Debug Perception
If you use linux to run the application, make sure use the opengl shipped with Nvidia. To Check it, run

    glxinfo | grep "OpenGL version"
	
And you should see something like:

    OpenGL version string: 4.5.0 NVIDIA 346.46

If you see something like "mesa", please install Nvidia Video Card Driver Again.

Test camera view

    rosrun image_view image_view image:=/xtion_right/rgb/image_color
    rosrun image_view image_view image:=/xtion_left/rgb/image_color

