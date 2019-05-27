#include <ros/ros.h>
#include "turtlebot_highlevel_controller/WallFollowing.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "WallFollowing");
  ros::NodeHandle nodeH = ros::NodeHandle();

  //namespace::classname object(Übergabewert)
  Following::WallFollowing Wallfollowing(nodeH);
  ros::spin();
  return 0;
}