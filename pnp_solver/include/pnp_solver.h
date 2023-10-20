#ifndef pnp_solver_NODE_H
#define pnp_solver_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Int32MultiArray.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

class PNPSolver
{

public:

  PNPSolver(){
    ros::param::get("EllipseNodeTopic", ellipseNodeTopic);
    ros::param::get("OdomTopic", odomTopic);

    ellipseNodeSuber = nodeHandle.subscribe(ellipseNodeTopic, 1, &PNPSolver::Callback, this);
    odomPuber = nodeHandle.advertise<sensor_msgs::Image>(odomTopic, 1);
  }

  void Callback(const std_msgs::Int32MultiArray::ConstPtr& ellipseNodes);

private:

  ros::NodeHandle nodeHandle;

  ros::Subscriber ellipseNodeSuber;
  ros::Publisher odomPuber;

  std::string ellipseNodeTopic;
  std::string odomTopic;

  tf2_ros::TransformBroadcaster tf_broadcaster;

};

#endif