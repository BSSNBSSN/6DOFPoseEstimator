#include "target_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_detector_node");
  TargetDetector targetDetector;
  ros::spin();
  return 0;
}
