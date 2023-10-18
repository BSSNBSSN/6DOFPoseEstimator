#ifndef TARGET_DETECTOR_NODE_H
#define TARGET_DETECTOR_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

class TargetDetector
{
private:

  ros::NodeHandle nodeHandle;

  ros::Subscriber imageTopicSuber;
  ros::Publisher nodesTopicPuber;

  std::string imageSubTopic;
  std::string nodesPubTopic;

public:

  TargetDetector(){
    ros::param::get("ImageSubTopic", imageSubTopic);
    ros::param::get("NodesPubTopic", nodesPubTopic);

    imageTopicSuber = nodeHandle.subscribe(imageSubTopic, 1, &TargetDetector::TargetDetectCallback, this);
    nodesTopicPuber = nodeHandle.advertise<sensor_msgs::Image>(nodesPubTopic, 1);
  }

  void TargetDetectCallback(const sensor_msgs::ImageConstPtr &cameraImage);
};

#endif