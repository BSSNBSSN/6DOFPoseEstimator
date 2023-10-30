#ifndef POSE_EKF_NODE_H
#define POSE_EKF_NODE_H

#include "ekf.h"

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Int32MultiArray.h"

#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

class PoseEKF
{
public:
    void Run();

    PoseEKF()
    {
        ros::param::get("BaseFrame", baseFrame);
        ros::param::get("TargetRawFrame", targetRawFrame);
        ros::param::get("TargetEKFFrame", targetEKFFrame);
    }

private:
    ros::NodeHandle nodeHandle;

    std::string baseFrame;
    std::string targetRawFrame;
    std::string targetEKFFrame;

    ExtendedKalmanFilter EKF;
};

#endif
