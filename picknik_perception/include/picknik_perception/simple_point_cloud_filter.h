/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : A simple filter for a static depth camera
*/

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


namespace picknik_perception
{

class SimplePointCloudFilter
{
public:

  SimplePointCloudFilter();
  ~SimplePointCloudFilter();

private:
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  

}; // class


} // end namespace
  
