#include "turtlebot_highlevel_controller/TurtlebotHighlevelController.hpp"

sensor_msgs::LaserScan Pub_scan;
nav_msgs::Odometry globalOdomFrame;
float SizeOfArray;
double ranges[5];

 
namespace HighlevelController {

TurtlebotHighlevelController::TurtlebotHighlevelController(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
{
  int queue_size;
  std::string topic;
  if(!nodeHandle.getParam("/queue_size", queue_size))
  {
      ROS_ERROR_STREAM("queue_size wurde nicht gelesen.");
      ros::requestShutdown();
  }
  if(!nodeHandle.getParam("/topic", topic))
  {
      ROS_ERROR_STREAM("topic wurde nicht gelesen.");
      ros::requestShutdown();
  }

  
  ros::Subscriber subscriber = nodeHandle.subscribe(topic, queue_size, &TurtlebotHighlevelController::chatterCallback, this);
  odom_subscriber_ = nodeHandle.subscribe("odom", queue_size, &TurtlebotHighlevelController::odomCallback, this);

  publisher_ = nodeHandle.advertise<sensor_msgs::LaserScan>("Pub_scan", queue_size);
  cmd_vel_pub_ = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  vis_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("Marker_Saeule", 0 );

  ROS_INFO("Successfully launched node.");

  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {

    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Shutdown node");
}

TurtlebotHighlevelController::~TurtlebotHighlevelController()
{
}

bool TurtlebotHighlevelController::readParameters()
{
}

void TurtlebotHighlevelController::topicCallback(const sensor_msgs::Temperature& message)
{
  //algorithm_.addData(message.temperature);
}

void TurtlebotHighlevelController::odomCallback(const nav_msgs::Odometry odomFrame)
{
  globalOdomFrame = odomFrame;
}

void TurtlebotHighlevelController::chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  SizeOfArray = msg->ranges.size();
  Pub_scan.ranges.resize(5);
  float AusgabeSubscriber[5];
  unsigned int index;
  float Min_Distanz = msg->ranges[0];
  for (int i=0; i<SizeOfArray;i++)
  {
    if (msg->ranges[i] < Min_Distanz)
    {
      Min_Distanz = msg->ranges[i];
      index = i;
    } 
  }

  if (index < 0) index = 0;
  if (index > 640) index = 640;
  ROS_INFO("Aktueller Index: [%i]", index);
  
  /**/
  if (index < 3)
      {
        Pub_scan.ranges[0] = 0;
        Pub_scan.ranges[1] = 0;
        AusgabeSubscriber[0] = 0;
        AusgabeSubscriber[1] = 0;
      }
      else 
      {
        Pub_scan.ranges[0] = msg->ranges[index-2];
        Pub_scan.ranges[1] = msg->ranges[index-1];
        AusgabeSubscriber[0] = msg->ranges[index-2];
        AusgabeSubscriber[1] = msg->ranges[index-1];
      }
      if (index == 640)
      {
        Pub_scan.ranges[3] = msg->ranges[index];
        Pub_scan.ranges[4] = msg->ranges[index];
        AusgabeSubscriber[3] = msg->ranges[index];
        AusgabeSubscriber[4] = msg->ranges[index];
      }
      else 
      {
        Pub_scan.ranges[3] = msg->ranges[index+1];
        Pub_scan.ranges[4] = msg->ranges[index+2];
        AusgabeSubscriber[3] = msg->ranges[index+1];
        AusgabeSubscriber[4] = msg->ranges[index+2];
      }
      Pub_scan.ranges[2]   = msg->ranges[index];
      AusgabeSubscriber[2] = msg->ranges[index];

  ROS_INFO("Distanzen: [%f], [%f], [%f], [%f], [%f]", AusgabeSubscriber[0],AusgabeSubscriber[1],AusgabeSubscriber[2],AusgabeSubscriber[3],AusgabeSubscriber);


  /*
    ros::Time scan_time = ros::Time::now();
    publisher_msg.header.stamp = scan_time;
    publisher_msg.header.frame_id = "base_laser_link";    
  */   
    Pub_scan.header.stamp = msg->header.stamp;
    Pub_scan.header.frame_id = msg->header.frame_id;

    //Pub_scan.angle_increment = 0.00158417993225;
    Pub_scan.angle_increment = msg->angle_increment;

    Pub_scan.angle_min = msg->angle_min+(index-2)*Pub_scan.angle_increment;
    Pub_scan.angle_max = msg->angle_max+(index+2)*Pub_scan.angle_increment;

    //scan.time_increment = (1 / laser_frequency) / (num_readings);
    Pub_scan.range_min = msg->range_min;
    Pub_scan.range_max = msg->range_max;

  // Position der Säule:
  float alpha;
  float distance;
  alpha = (msg->angle_min + index *msg->angle_increment);
  distance = Pub_scan.ranges[2];

  // Fahrt zur Säule:
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 0.5;
  base_cmd.linear.y = 0.0;
  base_cmd.linear.z = 0.0;
  base_cmd.angular.x = 0.0;
  base_cmd.angular.y = 0.0; 
  if (distance < 5 && distance > 0)
  {
    base_cmd.angular.z = (alpha/2/3.1416*360  * 0.1);
  }
  else
  {
    base_cmd.angular.z = 0.0;
  }

  //Hier werden die Pfeilerposition aus der Robosicht angegeben
  saeule_robo.header.frame_id = "base_laser_link";
  saeule_robo.header.stamp = ros::Time();
  saeule_robo.point.x = cos(alpha)*distance;
  saeule_robo.point.y = sin(alpha)*distance;
  saeule_robo.point.z = 0;

  try
  {
   tf_transform_robo_to_world.transformPoint("odom", saeule_robo, saeule_tf_to_odom);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

//Hier kommen die Marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "HighlevelController";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = saeule_tf_to_odom.point.x;
  marker.pose.position.y = saeule_tf_to_odom.point.y;
  marker.pose.position.z = saeule_tf_to_odom.point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    



  ROS_INFO("alpha, distance, base_cmd.angular.z: [%f], [%f], [%f]", alpha, distance, base_cmd.angular.z);
  
  cmd_vel_pub_.publish(base_cmd);
  publisher_.publish(Pub_scan);
  vis_pub_.publish(marker);
}
}

/*
bool TurtlebotHighlevelController::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}
}
*/
// /* namespace */
