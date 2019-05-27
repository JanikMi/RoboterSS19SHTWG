#include <ros/ros.h>
//#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "WallFollowing");
  ros::NodeHandle nodeHandle = ros::NodeHandle();

  //HighlevelController::TurtlebotHighlevelController turtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}