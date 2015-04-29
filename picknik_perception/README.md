# PickNik Perception

## PCL-Based Perception Server

Startup Jacob hardware or simulation stack as usual, but run the main process in mode ``18``. Use right side of shelf camera (close to desk). To calibrate use

    roslaunch picknik_perception dave_test_tf_keyboard.launch

Then run the perception server:
    
    roslaunch picknik_percpetion pcl_perception_server

And subscribe to ``/pcl_perception_server/roi_cloud``
