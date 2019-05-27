#pragma once
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <ros/param.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <string>
#include <stdbool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Spawn.h>

namespace Following {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class WallFollowing
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  WallFollowing(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~WallFollowing();

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

  // Definition der Säule 
  geometry_msgs::PointStamped saeule_robo;
  geometry_msgs::PointStamped saeule_tf_to_odom;

  // Transformation durch tf
  tf::TransformListener tf_transform_robo_to_world;
  tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tfListener;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher publisher_;
  ros::Publisher vis_pub_;
  //! Algorithm computation object.
  //Algorithm algorithm_;
};

} /* namespace */
