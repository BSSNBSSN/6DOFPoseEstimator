#ifndef TARGET_DETECTOR_NODE_H
#define TARGET_DETECTOR_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

class TargetDetector
{

public:

    TargetDetector()
    {
        ros::param::get("ImageSubTopic", imageSubTopic);
        ros::param::get("ImagePubTopic", imagePubTopic);
        ros::param::get("NodesPubTopic", nodesPubTopic);

        imageTopicSuber = nodeHandle.subscribe(imageSubTopic, 1, &TargetDetector::TargetDetectCallback, this);
        imageTopicPuber = nodeHandle.advertise<sensor_msgs::Image>(imagePubTopic, 1);
        nodesTopicPuber = nodeHandle.advertise<std_msgs::Int32MultiArray>(nodesPubTopic, 1);
    }
    
    void TargetDetectCallback(const sensor_msgs::ImageConstPtr &cameraImage);

private:
    ros::NodeHandle nodeHandle;

    ros::Subscriber imageTopicSuber;
    ros::Publisher imageTopicPuber;
    ros::Publisher nodesTopicPuber;

    std::string imageSubTopic;
    std::string imagePubTopic;
    std::string nodesPubTopic;

};

#endif