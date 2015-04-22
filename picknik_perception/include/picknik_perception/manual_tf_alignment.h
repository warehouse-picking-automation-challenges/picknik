/*
 * Author : Andy McEvoy (mcevoy.andy@gmail.com)
 * Desc   : Allows manual control of a TF through the keyboard
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <string>

#include <keyboard/Key.h>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

namespace picknik_perception
{

class ManualTFAlignment
{
public:

  /*
   * \brief
   */
  ManualTFAlignment();
  
  /*
   * \brief
   */
  ~ManualTFAlignment();
  
  /*
   * \brief
   */
  void keyboardCallback(const keyboard::Key::ConstPtr& msg);
  
  /*
   * \brief
   */
  void printMenu();

  /* 
   * \brief 
   */
  void publishTF();

  /*
   * \brief
   */
  void setPose(Eigen::Vector3d translation, Eigen::Vector3d rotation);

  /*
   * \brief
   */
  void updateTF(int mode, double delta);
  
  /*
   * \brief
   */
  void writeTFToFile();

  Eigen::Vector3d translation_;
  Eigen::Vector3d rotation_;
  std::string save_path_;
  int mode_;
  double delta_;
  std::string from_;
  std::string to_;


};

}
