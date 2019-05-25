#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>
#include <vector>
#include <time.h>
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nodeHandle = ros::NodeHandle();

  HighlevelController::TurtlebotHighlevelController turtlebotHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}