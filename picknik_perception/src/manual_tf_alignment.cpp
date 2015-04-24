/*
 * Author : Andy McEvoy
 * Desc   : Allows manual control of a TF through the keyboard (see *.h file for documentation)
 */

#include <picknik_perception/manual_tf_alignment.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_perception
{

ManualTFAlignment::ManualTFAlignment()
  : nh_("~")
{
  // set defaults
  mode_ = 1;
  delta_ = 0.010;

  // Get settings from rosparam

  // initial camera transform
  const std::string parent_name = "manipulation_data"; // for namespacing logging messages
  double x, y, z, roll, pitch, yaw;
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_x", x);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_y", y);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_z", z);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_roll", roll);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_pitch", pitch);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "initial_yaw", yaw);
  rviz_visual_tools::getStringParameter(parent_name, nh_, "file_name", file_name_);
  rviz_visual_tools::getStringParameter(parent_name, nh_, "topic_name", topic_name_);
  setPose(Eigen::Vector3d(x, y, z), Eigen::Vector3d(roll, pitch, yaw));

  // get frame names
  rviz_visual_tools::getStringParameter(parent_name, nh_, "from", from_);
  rviz_visual_tools::getStringParameter(parent_name, nh_, "to", to_);

  // default, save in picknik_perception/data
  std::string package_path = ros::package::getPath("picknik_perception");
  save_path_ = package_path + "/config/" + file_name_ + ".yaml";

  // listen to keyboard topic
  keyboard_sub_ = nh_.subscribe(topic_name_, 100,
                                &ManualTFAlignment::keyboardCallback, this);

  // Echo info
  ROS_INFO_STREAM_NAMED("manualTF","Listening to topic : " << topic_name_);
  ROS_INFO_STREAM_NAMED("manualTF","Transform from     : " << from_);
  ROS_INFO_STREAM_NAMED("manualTF","Transform to       : " << to_);
  ROS_INFO_STREAM_NAMED("manualTF","Will save TF data to: " << save_path_);
}

void ManualTFAlignment::keyboardCallback(const keyboard::Key::ConstPtr& msg)
{
  int entry = msg->code;
  double fine = 0.001;
  double coarse = 0.01;

  switch(entry)
  {
    case 112: //
      std::cout << "Writing transformation to file..." << std::endl;
      writeTFToFile();
      break;
    case 105: // (coarse delta)
      std::cout << "Delta = coarse (0.010)" << std::endl;
      delta_ = coarse;
      break;
    case 107: // (fine delta)
      std::cout << "Delta = fine (0.001)" << std::endl;
      delta_ = fine;
      break;

    // X axis
    case 113: // up
      updateTF(1, delta_);      
      break;
    case 97: // down
      updateTF(1, -delta_);      
      break;

    // y axis
    case 119: // up
      updateTF(2, delta_);      
      break;
    case 115: // down
      updateTF(2, -delta_);      
      break;

    // z axis
    case 101: // up
      updateTF(3, delta_);      
      break;
    case 100: // down
      updateTF(3, -delta_);      
      break;

    // roll
    case 114: // up
      updateTF(4, delta_);      
      break;
    case 102: // down
      updateTF(4, -delta_);      
      break;

    // pitch
    case 116: // up
      updateTF(5, delta_);      
      break;
    case 103: // down
      updateTF(5, -delta_);      
      break;

    // yaw
    case 121: // up
      updateTF(6, delta_);      
      break;
    case 104: // down
      updateTF(6, -delta_);      
      break;

    default:
      // don't do anything
      break;
  }
}

void ManualTFAlignment::printMenu()
{
  std::cout << "Manual alignment of camera to world CS:" << std::endl;
  std::cout << "=======================================" << std::endl;
  std::cout << "MOVE: X  Y  Z  R  P  YAW " << std::endl;
  std::cout << "------------------------" << std::endl;
  std::cout << "up    q  w  e  r  t  y " << std::endl;
  std::cout << "down  a  s  d  f  g  h " << std::endl;
  std::cout << std::endl;
  std::cout << "Fast: u " << std::endl;
  std::cout << "Slow: j " << std::endl;
  std::cout << "Save: p " << std::endl;
  // std::cout << "c\tSet adjustment delta to COARSE (0.010)" << std::endl;
  // std::cout << "f\tSet adjustment delta to FINE (0.001)" << std::endl;
  // std::cout << "e\tWrite transform values to file" << std::endl;
}

void ManualTFAlignment::publishTF()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  // set camera pose translation
  transform.setOrigin( tf::Vector3( translation_[0],
                                    translation_[1],
                                    translation_[2]) );

  // set camera pose rotation
  q.setRPY(rotation_[0], rotation_[1], rotation_[2]);
  transform.setRotation(q);

  // publish
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from_ , to_));
}

void ManualTFAlignment::setPose(Eigen::Vector3d translation, Eigen::Vector3d rotation)
{
  translation_ = translation;
  rotation_ = rotation;
}

void ManualTFAlignment::updateTF(int mode, double delta)
{
  ROS_DEBUG_STREAM_NAMED("tf_alignment","mode = " << mode << ", delta = " << delta);

  switch(mode)
  {
    case 1:
      translation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 2:
      translation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 3:
      translation_ += Eigen::Vector3d(0, 0, delta);
      break;
    case 4:
      rotation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 5:
      rotation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 6:
      rotation_ += Eigen::Vector3d(0, 0, delta);
      break;
    default:
      // don't do anything
      break;
  }
}


void ManualTFAlignment::writeTFToFile()
{
  std::ofstream file (save_path_.c_str()); //, std::ios::app);

  if (!file.is_open())
    ROS_ERROR_STREAM_NAMED("tf_align.write","Output file could not be opened: " << save_path_);
  else
  {
    ROS_INFO_STREAM_NAMED("tf_align.write","Camera translation = \n " << translation_);
    ROS_INFO_STREAM_NAMED("tf_align.write","Camera rotations = \n" << rotation_);
    
    file << "initial_x: " << translation_[0] << std::endl;
    file << "initial_y: " << translation_[1] << std::endl;
    file << "initial_z: " << translation_[2] << std::endl;
    file << "initial_roll: " << rotation_[0] << std::endl;
    file << "initial_pitch: " << rotation_[1] << std::endl;
    file << "initial_yaw: " << rotation_[2] << std::endl;
    file << "from: " << from_ << std::endl;
    file << "to: " << to_ << std::endl;
    file << "file_name: " << file_name_ << std::endl;
    file << "topic_name: " << topic_name_ << std::endl;
  }
  file.close();

}

}
