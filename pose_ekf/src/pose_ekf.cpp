#include "pose_ekf.h"

void PoseEKF::Run()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;
    
    Eigen::VectorXd x0(7);
    x0 << 1, 0, 0, 0, 0, 0, 0;
    EKF.StateInitialize(x0);

    while (ros::ok())
    {
         try
        {
            // Get TF
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tfBuffer.lookupTransform(this->baseFrame, this->targetRawFrame, ros::Time(0));

            // Get quaternion
            tf2::Quaternion quaternion;
            tf2::convert(transformStamped.transform.rotation, quaternion);
            ROS_INFO("Raw Quaternion (x, y, z, w): %f, %f, %f, %f",
                     quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

            // EKF
            Eigen::VectorXd z(7);
            z << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(), 0, 0, 0;
            EKF.StatePredict();
            EKF.StateUpdate(z);
            EKF.GetState(quaternion);

            // Publish filtered TF
            // std::cout<<"header "<<transformStamped.header.frame_id<<std::endl;
            // std::cout<<"child_frame_id "<<transformStamped.child_frame_id<<std::endl;
            
            transformStamped.child_frame_id = this->targetEKFFrame;
            transformStamped.transform.rotation.w = quaternion.w();
            transformStamped.transform.rotation.x = quaternion.x();
            transformStamped.transform.rotation.y = quaternion.y();
            transformStamped.transform.rotation.z = quaternion.z();

            // Broadcast transform
            tfBroadcaster.sendTransform(transformStamped);

        }
        catch (tf2::TransformException &ex)
        {
            // ROS_WARN("Failed to get TF transform: %s", ex.what());
        }
    }
}