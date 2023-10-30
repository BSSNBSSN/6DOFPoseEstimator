#include "ekf.h"

Eigen::MatrixXd ExtendedKalmanFilter::StatePredict()
{
    Updatedt();
    UpdateF(x_post_);
    UpdateQ();

    x_pri_ = F_ * x_post_;
    P_pri_ = F_ * P_post_ * F_.transpose() + Q_;

    // handle the case when there will be no measurement before the next predict
    x_post_ = x_pri_;
    P_post_ = P_pri_;
}

Eigen::MatrixXd ExtendedKalmanFilter::StateUpdate(const Eigen::VectorXd &z)
{
    UpdateH();
    UpdateR(z);

    K_ = P_pri_ * H_.transpose() * (H_ * P_pri_ * H_.transpose() + R_).inverse();
    x_post_ = x_pri_ + K_ * (z - H_ * x_pri_);
    P_post_ = (I_ - K_ * H_) * P_pri_;
    return x_post_;

}

void ExtendedKalmanFilter::GetState(tf2::Quaternion & quaternion)
{
    quaternion.setW(x_post_(0));
    quaternion.setX(x_post_(1));
    quaternion.setY(x_post_(2));
    quaternion.setZ(x_post_(3));
}

void ExtendedKalmanFilter::StateInitialize(const Eigen::VectorXd &x0)
{

    timeRecorder_ = ros::Time::now();
    x_post_ = x0;

    H_ << 1, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0;

    // set P as infinity to ignore the initial orientation
    P_post_ << 1e7, 0, 0, 0, 0, 0, 0,
               0, 1e7, 0, 0, 0, 0, 0,
               0, 0, 1e7, 0, 0, 0, 0,
               0, 0, 0, 1e7, 0, 0, 0,
               0, 0, 0, 0, 10, 0, 0,
               0, 0, 0, 0, 0, 10, 0,
               0, 0, 0, 0, 0, 0, 10;

}

void ExtendedKalmanFilter::Updatedt()
{
    ros::Duration duration;
    ros::Time currentTime = ros::Time::now();
    duration = currentTime - timeRecorder_;
    dt_ = duration.toSec();
    timeRecorder_ = currentTime;
}

void ExtendedKalmanFilter::UpdateF(const Eigen::VectorXd &x)
{
    // F =       
    // [             1, -(dt*omega_x)/2, -(dt*omega_y)/2, -(dt*omega_z)/2, -(dt*q1)/2, -(dt*q2)/2, -(dt*q3)/2]
    // [(dt*omega_x)/2,               1,  (dt*omega_z)/2, -(dt*omega_y)/2,  (dt*q0)/2, -(dt*q3)/2,  (dt*q2)/2]
    // [(dt*omega_y)/2, -(dt*omega_z)/2,               1,  (dt*omega_x)/2,  (dt*q3)/2,  (dt*q0)/2, -(dt*q1)/2]
    // [(dt*omega_z)/2,  (dt*omega_y)/2, -(dt*omega_x)/2,               1, -(dt*q2)/2,  (dt*q1)/2,  (dt*q0)/2]
    // [             0,               0,               0,               0,          1,          0,          0]
    // [             0,               0,               0,               0,          0,          1,          0]
    // [             0,               0,               0,               0,          0,          0,          1]

    F_ <<            1, -(dt_*x(4))/2, -(dt_*x(5))/2, -(dt_*x(6))/2, -(dt_*x(1))/2, -(dt_*x(2))/2, -(dt_*x(3))/2,
          (dt_*x(4))/2,             1,  (dt_*x(6))/2, -(dt_*x(5))/2,  (dt_*x(0))/2, -(dt_*x(3))/2,  (dt_*x(2))/2,
          (dt_*x(5))/2, -(dt_*x(6))/2,             1,  (dt_*x(4))/2,  (dt_*x(3))/2,  (dt_*x(0))/2, -(dt_*x(1))/2,
          (dt_*x(6))/2,  (dt_*x(5))/2, -(dt_*x(4))/2,             1, -(dt_*x(2))/2,  (dt_*x(1))/2,  (dt_*x(0))/2,
                     0,             0,             0,             0,             1,             0,             0,
                     0,             0,             0,             0,             0,             1,             0,
                     0,             0,             0,             0,             0,             0,             1;

}

void ExtendedKalmanFilter::UpdateH()
{
    // No need to update H for this problem
    // H_ << 1, 0, 0, 0, 0, 0, 0,
    //       0, 1, 0, 0, 0, 0, 0,
    //       0, 0, 1, 0, 0, 0, 0,
    //       0, 0, 0, 1, 0, 0, 0,
    //       0, 0, 0, 0, 0, 0, 0,
    //       0, 0, 0, 0, 0, 0, 0,
    //       0, 0, 0, 0, 0, 0, 0;
}

void ExtendedKalmanFilter::UpdateQ()
{
    Q_ << 0.1, 0, 0, 0, 0, 0, 0,
          0, 0.1, 0, 0, 0, 0, 0,
          0, 0, 0.1, 0, 0, 0, 0,
          0, 0, 0, 0.1, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 1;

    Q_ = Q_ * dt_;
}

void ExtendedKalmanFilter::UpdateR(const Eigen::VectorXd &x)
{
    R_ << 0.01, 0, 0, 0, 0, 0, 0,
          0, 0.01, 0, 0, 0, 0, 0,
          0, 0, 0.01, 0, 0, 0, 0,
          0, 0, 0, 0.01, 0, 0, 0,
          0, 0, 0, 0, 1e7, 0, 0,
          0, 0, 0, 0, 0, 1e7, 0,
          0, 0, 0, 0, 0, 0, 1e7;

    // TODO: Times distance to target
}
