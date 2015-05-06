# PickNik Perception

## PCL-Based Perception Server

Startup Jacob hardware or simulation stack as usual, but run the main process in mode ``18``. Use right side of shelf camera (close to desk).

See main README.md for starting cameras.

Merge point clouds

    rosrun picknik_perception merge_point_clouds

Run the perception server:
    
    roslaunch picknik_perception perception_server_pcl.launch

Test perception server:

    roslaunch picknik_main jacob_apc.launch fake_perception:=0 fake_execution:=0 mode:=18

## Notes on Filtering

**NOTE: the simple preprocessor assumes that the camera has already been aligned**

The `SimplePointCloudFilter` class uses the PCL library's `PassThrough`, `RadialOutlierRemover`, and
`StatisticalOutlierRemoval`. For general preprocessing of the point cloud it is recommended that only the
`PassThrough` filters be used (default).

The other filters are slower and can be turned on by setting `outlier_removal_` to true.

## Manual alignment for cameras:

Before launching a node to manually align a camera, create a config file for it.

**NOTE:**
**Roll, pitch, and yaw angles are absolute rotations with respect to the FROM coordinate system. These are NOT Euler
angles. They are absolute angles from the FROM coordinate system.**

An example from `.../picknik_perception/config/tf_xtion_right.yaml`

```
initial_x: -0.6
initial_y: -0.38
initial_z: 1.26
initial_roll: 3.15159
initial_pitch: 0.05
initial_yaw: -0.03
from: world
to: xtion_right_link
file_name: tf_xtion_right
topic_name: /keyboard_right/keydown
```
Pressing `p` in the keyboard window will automatically save this file in `../picknik_perception/config` using the
`file_name` parameter.

## Launching multiple ASUS Xtion cameras

**NOTE: To run multiple ASUS Xtions, make sure they are on different USB busses.**

Run `lsusb` to make sure that the ASUS Xtion cameras are on different USB busses. For example, note below that the ASUS cameras are connected to USB bus 002 and 001

```
Bus 002 Device 031: ID 1d27:0601 ASUS
Bus 002 Device 004: ID 0461:4d22 Primax Electronics, Ltd
Bus 002 Device 026: ID 04e8:6860 Samsung Electronics Co., Ltd GT-I9100 Phone [Galaxy S II], GT-I9300 Phone [Galaxy S III], GT-P7500 [Galaxy Tab 10.1]
Bus 002 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 001 Device 006: ID 1d27:0600 ASUS
Bus 001 Device 003: ID 03f0:0024 Hewlett-Packard KU-0316 Keyboard
Bus 001 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub

```

To figure out which camera is on which bus, launch use `roslaunch openni_launch openni.launch` normally and look for the following message:

```
INFO ros.openni_camera./xtion_right/driver: Number devices connected: 2
INFO ros.openni_camera./xtion_right/driver: 1. device on bus 002:31 is a SensorV2 (601) from PrimeSense (1d27) with serial id '1407160044'
INFO ros.openni_camera./xtion_right/driver: 2. device on bus 001:06 is a SensorV2 (600) from PrimeSense (1d27) with serial id '1205100079'
INFO ros.openni_camera./xtion_right/driver: Searching for device with bus@address = 1@0	
```

Create a launch file that will change the camera topic names (from `multi_xtion.launch`):

```
<!-- Start Xtions -->
<include file="$(find openni_launch)/launch/openni.launch">
   <arg name="camera" value="xtion_right" />
   <arg name="device_id" value="1@0" />
   <arg name="depth_registration" value="true" />
</include>

<include file="$(find openni_launch)/launch/openni.launch">
   <arg name="camera" value="xtion_left" />
   <arg name="device_id" value="2@0" />
   <arg name="depth_registration" value="true" />
</include>
```
### Merging point clouds

If two cameras are being used and are aligned, the clouds can be merged into a single topic with

```
rosrun picknik_perception merge_point_clouds
```

by default, this subscribes to `/xtion_left/depth_registered/points` and `/xtion_right/depth_registered/points`.
