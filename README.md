# Team PickNik - Amazon Picking Challenge

See http://picknik.io

## Team Members:

- Dave Coleman <david.t.coleman@colorado.edu>
- Lu Ma <Lu.Ma@colorado.edu>
- Andy McEvoy <mcevoy.andy@gmail.com>
- Jorge Cañardo Alastuey <jorgecanardo@gmail.com>
- Nicholas Farrow <Nicholas.Farrow@colorado.edu>

### Advisers

- Gabe Sibley <gsibley@colorado.edu>
- Nikolaus Correll <nikolaus.correll@colorado.edu>

## Install

Dave occasionally releases a new zip file with a lot of custom ROS code, that can be built into one workspace. Download the latest zip (~700MB) from here:

Directory: [http://picknik.io/secure](http://picknik.io/secure)

User: picknik

Password: sfd798asfiahfl89o7df980791324jhkls

Unzip the file and put into a catkin workspace. Build using catkin_tools.

## Run

### Run In Simulation

Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

Run APC Manager (main program)

    roslaunch baxter_apc_main apc_manager.launch verbose:=true use_scratch:=true saving_enabled:=false debug:=false

### Run On Hardware

Start fake controllers

    roslaunch baxter_control baxter_hardware.launch

Rviz Visualizer

    roslaunch baxter_apc_main moveit_rviz.launch

Run APC Manager (main program)

    roslaunch baxter_apc_main apc_manager.launch

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