#ifndef SUBPUBER_NODE_H
#define SUBPUBER_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

class PoseEKF
{
private:

  ros::NodeHandle nodeHandle;

  ros::Subscriber topicSuber0;
  ros::Publisher topicPuber0;
  ros::Subscriber topicSuber1;
  ros::Publisher topicPuber1;

  std::string rawPose;
  std::string ekfPose;
  std::string subTopic1;
  std::string pubTopic1;

public:

  PoseEKF(){
    ros::param::get("RawPose", rawPose);
    ros::param::get("EKFPose", ekfPose);

    // topicSuber0 = nodeHandle.subscribe(rawPose, 1, &PoseEKF::Callback0, this);
    topicPuber0 = nodeHandle.advertise<sensor_msgs::Image>(ekfPose, 1);
  }

  void Callback0(const sensor_msgs::ImageConstPtr &cameraImage);
};

#endif