# Baxter-specific documentation

### Simulation of BAXTER

Start roscore:

    roscore &
	
Start fake controllers

    roslaunch baxter_control baxter_visualization.launch

Rviz Visualizers of robot states and debug markers 

    roslaunch picknik_main rviz.launch jacob:=false

Run the fake object recognition server: (or real one if you have Lu Ma skillz)

	roslaunch picknik_main fake_perception_server.launch

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

Run APC Manager (main program) for BAXTER

    roslaunch picknik_main baxter_apc.launch mode:=1
