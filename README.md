![Team PickNik](http://picknik.io/PickNik_Logo3.png)

 - Team PickNik - Amazon Picking Challenge
 - Website: [http://picknik.io](http://picknik.io)
 - [APC PickNik Team](https://bitbucket.org/cuamazonchallenge/profile/members)

## Install

Dave occasionally releases a new zip file with a lot of custom ROS code, that can be built into one workspace. Download the latest zip (~700MB) from here:

    Directory: http://picknik.io/secure
    User: picknik
    Password: sfd798asfiahfl89o7df980791324jhkls

Unzip the file and put into a catkin workspace. Build using catkin_tools.

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

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

Now skip to section **Run Main Routine**

### Setup Hardware

BETA - Start actual controllers

    roslaunch baxter_control baxter_hardware.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

Now go to section **Run Main Routine**

### Run Main Routine

Run APC Manager (main program)

    roslaunch baxter_apc_main apc_manager.launch verbose:=true use_scratch:=true saving_enabled:=false debug:=false order:=order.json

Optional Arguments:

    order - which json file to use, defaults to orders/simple.json
	use_scratch - whether to always plan from scratch (true) or use experience database to speed up planning (false)
	saving_enabled - allow new plans to be saved to experience database
	debug - slower and more verbose

## Working Tests

Document here all roslaunch files for testing/verifying various parts of the system. Also specify if you've checked it in
simulation, hardware, or both.

### Loading meshes

Simply displays all meshes from our mesh library in Rviz. Be sure to set the right planning scene in Rviz by changing it to '/mesh_publisher/baxter_apc_planning_scene'

 - Visualization: Working Jan 30
 - Hardware: N/A

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

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
