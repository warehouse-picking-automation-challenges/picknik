![Team PickNik](http://picknik.io/PickNik_Logo3.png)

**Amazon Picking Challenge**

 - Website: [http://picknik.io](http://picknik.io)
 - [Team Members](https://bitbucket.org/cuamazonchallenge/profile/members)
 - [Timeline](https://docs.google.com/spreadsheets/d/1GG_j6BVir-J8VGwbU8RWDHA8kD8ZSeXrtlLtW9N851o/edit?usp=sharing)
 - [Item Data](https://docs.google.com/spreadsheets/d/1e0Fousz9TUxf9YHeVfnaKVgf06Z0WC50blMGBWJ9cp8/edit#gid=2088756835)
 
## Install

Dave occasionally releases a new zip file with a lot of custom ROS code, that can be built into one workspace. Download the latest zip (~700MB) from here:

    Directory: http://picknik.io/secure
    User: picknik
    Password: sfd798asfiahfl89o7df980791324jhkls

Unzip the file and put into a catkin workspace. Build using catkin_tools.

![Pipeline](https://bytebucket.org/cuamazonchallenge/picknik/raw/3f6788816ad7733051493f55f142655b2702adb1/baxter_apc_main/docs/apc_picknik_pipeline.png?token=ef4e18838e57f4cb97be4ecff9691b3740dd8a8e)

## Run

### Generate Mock Amazon order

Create a simulated bin inventory and random order by running

    rosrun baxter_apc_main random_orders.py order.json

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

### Setup Simulation

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Now skip to section **Run Main Routine**

### Setup Hardware

#### Version 1

BETA - Start actual controllers

    roslaunch baxter_control baxter_hardware.launch

Now go to section **Run Main Routine**

#### Version 2

Use Rethink's controllers

    roslaunch baxter_control baxter_hardware_rethink.launch

Make sure correct controller name is chosen in baxter_controllers.yaml

### Turn on Yale Controller

    roslaunch open_hand_controller controller_manager.launch

Check for correct USB

    ls /dev/ttyUSB*

Check to make sure you have ``dialout`` group

    sudo adduser second_user dialout


### Run Main Routine

Rviz Visualizer of robot state

    roslaunch baxter_apc_main moveit_display_rviz.launch
	
Rviz Visualizer of planning scene and debug markers

    roslaunch baxter_apc_main moveit_rviz.launch

Run APC Manager (main program)

    roslaunch baxter_apc_main apc_manager.launch mode:=1 verbose:=1 use_experience:=1 saving_enabled:=1 debug:=0 show_database:=0

Optional Arguments:

    mode - what program to run inside the apc_manager, defaults to 1
	  Available Modes:
	    1. Actual APC contest mode
		2. Train experience database mode / workspace analysis
		3. Test end effectors mode
		4. Only load JSON and visualize shelf
		5. Raise the roof (go up and down)
	jump_to - which step in the manipulation pipeline to start on
	  Steps:
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
    order - which json file to use, defaults to orders/simple.json
	order_start - specify the index of the product to skip to, based on the ordering in the json file
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

    roslaunch baxter_apc_main mesh_publisher_rviz.launch

Load meshes

    rosrun baxter_apc_main mesh_publisher

### Random Planning

Has Baxter choose random poses with both arms and plan to them.

 - Visualization: UNTESTED
 - Hardware: UNTESTED

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

Planner

    roslaunch baxter_moveit_scripts random_planning.launch

### End Effector Test

Open and closes both end effectors

 - Visualization: Working Jan 30
 - Hardware: UNTESTED

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

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
