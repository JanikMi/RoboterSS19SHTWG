#pragma once
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include<visualization_msgs/Marker.h>
namespace HighlevelController {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class TurtlebotHighlevelController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TurtlebotHighlevelController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TurtlebotHighlevelController();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const sensor_msgs::Temperature& message);
  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */

  void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /*
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);
  */
 
   void odomCallback(const nav_msgs::Odometry odomFrame);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;
  ros::Subscriber odom_subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher publisher_;
  ros::Publisher vis_pub_;
  //! Algorithm computation object.
  //Algorithm algorithm_;
};

} /* namespace */
