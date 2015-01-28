# MoveIt! Autonomous Robotics & Perception Group Interface

Interface between the [Autonomous Robotics and Perception Group](https://github.com/arpg) at CU Boulder and MoveIt!

MoveIt! has a client-server interface where it polls the perception pipeline for detected objects, and they are returned in a ROS message. This package contains that interface.

## Install

Assuming you have never used ROS on your computer before...

* Install ROS Indigo

    http://wiki.ros.org/indigo/Installation/Ubuntu

* Create a catkin workspace 

    ```
    mkdir -p ~/ros/ws_moveit/src
    cd ~/ros/ws_moveit/src
    ```

* Download this package

    ```
	git clone git@github.com:davetcoleman/moveit_arpg_perception.git
    ```
	
* Setup ROS if you haven't already (you can probably skip this):

    ```
    sudo apt-get update
    sudo apt-get dist-upgrade
    source /opt/ros/indigo/setup.bash
    rosdep update
    ```

* Install dependencies and build

    ```
    cd ..
    rosdep install --from-paths . --ignore-src --rosdistro indigo -y
    catkin_make
    ```

* Add workspace setup.bash to your .bashrc (recommended)

    ```
    echo 'source ~/ros/ws_moveit/devel/setup.bash' >> ~/.bashrc
    ```

## Use

To run the action server:

    roscore &
    rosrun moveit_arpg_perception object_recognition_server

To test you should see some topics now available:

    rostopic list

Now when running the Rviz Motion Planning Plugin, the 'Detect' button calls this script to publish any visible objects.
