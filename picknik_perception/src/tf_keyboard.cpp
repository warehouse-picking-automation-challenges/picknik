/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com), Dave Coleman
  Desc  : Tweak a TF transform using a keyboard
*/
#include <picknik_perception/manual_tf_alignment.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_keyboard");
  ROS_INFO_STREAM_NAMED("tf_keyboard","Starting keyboard control");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::ManualTFAlignment tf_align;
  tf_align.printMenu();

  ros::Rate rate(40.0); // hz
  while ( ros::ok() )
  {
    // publish transform to camera
    tf_align.publishTF();

    rate.sleep();
  }

}
