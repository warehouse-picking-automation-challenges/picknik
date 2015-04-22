/*
 * Author : Andy McEvoy (mcevoy.andy@gmail.com)
 * Desc   : Allows manual control of a TF through the keyboard
 */

#include <keyboard/Key>

namespace picknik_perception
{

class ManualTFAlignment
{
public:
  ManualTFAlignment();
  ~ManualTFAlignment();

  void keyboardCallback(const keyboard::Key::ConstPtr& msg);
  void updateTF(int mode, double delta);
  void writeTFToFile();

  Eigen::Vector3d translation_;
  Eigen::Vector3d rotation_;

};

}
