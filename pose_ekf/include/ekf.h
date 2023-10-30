#ifndef EKF_H
#define EKF_H

#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

class ExtendedKalmanFilter
{
public:

    // Set the initial state
    void StateInitialize(const Eigen::VectorXd &x0);
    // Update EKF parameters
    void Updatedt();
    void UpdateF(const Eigen::VectorXd &x);
    void UpdateH();
    void UpdateQ();
    void UpdateR(const Eigen::VectorXd &x);
    // EKF pridict
    Eigen::MatrixXd StatePredict();
    // EKF update
    Eigen::MatrixXd StateUpdate(const Eigen::VectorXd &z);

    void GetState(tf2::Quaternion &quaternion);
    
    ExtendedKalmanFilter()
    {
        F_ = Eigen::MatrixXd::Zero(7, 7);
        H_ = Eigen::MatrixXd::Zero(7, 7);
        Q_ = Eigen::MatrixXd::Zero(7, 7);
        R_ = Eigen::MatrixXd::Zero(7, 7);
        P_pri_ = Eigen::MatrixXd::Zero(7, 7);
        P_post_ = Eigen::MatrixXd::Zero(7, 7);
        K_ = Eigen::MatrixXd::Zero(7, 7);
        I_ = Eigen::MatrixXd::Identity(7, 7);
        x_pri_ = Eigen::VectorXd::Zero(7);
        x_post_ = Eigen::VectorXd::Zero(7);
    }

private:

    ros::Time timeRecorder_;
    double dt_;
    
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_pri_;
    Eigen::MatrixXd P_post_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd I_;
    Eigen::VectorXd x_pri_;
    Eigen::VectorXd x_post_;
};

#endif