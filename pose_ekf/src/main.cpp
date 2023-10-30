#include "pose_ekf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_ekf_node");
    PoseEKF poseEKF;

    poseEKF.Run();
    return 0;
}
