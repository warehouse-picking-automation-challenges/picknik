/*
 * Author : Andy McEvoy
 * Desc   : Allows manual control of a TF through the keyboard (see *.h file for documentation)
 */

#include <picknik_perception/manual_tf_alignment.h>

namespace picknik_perception
{

ManualTFAlignment::ManualTFAlignment()
{
  // default, save in picknik_perception/data
  std::string package_path = ros::package::getPath("picknik_perception");
  save_path_ = package_path + "/data/tf_alignment.txt";
  ROS_INFO_STREAM_NAMED("manualTF","Will save TF data to: " << save_path_);
  
  // set defaults
  mode_ = 1;
  delta_ = 0.010;

  // set default transform
  from_ = "/world";
  to_ = "/base";

}

ManualTFAlignment::~ManualTFAlignment()
{

}

void ManualTFAlignment::keyboardCallback(const keyboard::Key::ConstPtr& msg)
{
  int entry = msg->code;
  double fine = 0.001;
  double coarse = 0.01;

  switch(entry)
  {
    case 101: // e
      std::cout << "Writing transformation to file..." << std::endl;
      writeTFToFile();
      break;
    case 99: // c (coarse delta)
      std::cout << "Delta = coarse (0.010)" << std::endl;
      delta_ = coarse;
      break;
    case 102: // f (fine delta)
      std::cout << "Delta = fine (0.001)" << std::endl;
      delta_ = fine;
      break;
    case 120: // x 
      std::cout << "Press +/- to adjust X translation" << std::endl;
      mode_ = 1;
      break;
    case 121: // y
      std::cout << "Press +/- to adjust Y translation" << std::endl;
      mode_ = 2;
      break;
    case 122: // z
      std::cout << "Press +/- to adjust Z translation" << std::endl;
      mode_ = 3;
      break;          
    case 114: // r (roll)
      std::cout << "Press +/- to adjust roll angle" << std::endl;
      mode_ = 4;
      break;
    case 112: // p (pitch)
      std::cout << "Press +/- to adjust pitch angle" << std::endl;
      mode_ = 5;
      break;          
    case 119: // w (yaw)
      std::cout << "Press +/- to adjust yaw angle" << std::endl;
      mode_ = 6;
      break;
    case 270: // + (on numberpad)
      updateTF(mode_, delta_);
      break;
    case 269: // - (on numberpad)
      updateTF(mode_, -delta_);
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
  std::cout << "\nChoose Mode:" << std::endl;
  std::cout << "x\tAdjust X translation" << std::endl;
  std::cout << "y\tAdjust Y translation" << std::endl;
  std::cout << "z\tAdjust Z translation" << std::endl;    
  std::cout << "r\tAdjust Roll (rotation about X)" << std::endl;
  std::cout << "p\tAdjust Pitch (rotation about Y)" << std::endl;
  std::cout << "w\tAdjust Yaw (rotation about Z)" << std::endl;
  std::cout << "c\tSet adjustment delta to COARSE (0.010)" << std::endl;
  std::cout << "f\tSet adjustment delta to FINE (0.001)" << std::endl;
  std::cout << "e\tWrite transform values to file" << std::endl;
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
  ROS_DEBUG_STREAM_NAMED("PC_filter.update","mode = " << mode << ", delta = " << delta);

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
  std::string file_path = save_path_ + "/camera_transform.txt";
  std::ofstream file (file_path.c_str(), std::ios::app);

  if (!file.is_open())
    ROS_ERROR_STREAM_NAMED("PC_filter.write","output file could not be opened");
  else
  {
    ROS_INFO_STREAM_NAMED("PC_filter.write","Camera translation = \n " << translation_);
    ROS_INFO_STREAM_NAMED("PC_filter.write","Camera rotations = \n" << rotation_);
    for (std::size_t i = 0; i < 3; i++)
      file << translation_[i] << "\t ";
    for (std::size_t i = 0; i < 3; i++)
      file << rotation_[i] << "\t";
    file << std::endl;
  }
  file.close();

}

}
