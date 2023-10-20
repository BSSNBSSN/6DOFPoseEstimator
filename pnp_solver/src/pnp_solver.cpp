#include "pnp_solver.h"

void PNPSolver::Callback(const std_msgs::Int32MultiArray::ConstPtr& ellipseNodes)
{

  // 3D coordinate
  std::vector<cv::Point3f> objectPoints;
  objectPoints.push_back(cv::Point3f(0, 0, 0.25));
  objectPoints.push_back(cv::Point3f(0, 0.25, 0));
  objectPoints.push_back(cv::Point3f(0, 0, -0.25));
  objectPoints.push_back(cv::Point3f(0, -0.25, 0));

  // 2D coordinate
  std::vector<cv::Point2f> imagePoints;
  imagePoints.push_back(cv::Point2f(ellipseNodes->data.at(0), ellipseNodes->data.at(1)));
  imagePoints.push_back(cv::Point2f(ellipseNodes->data.at(2), ellipseNodes->data.at(3)));
  imagePoints.push_back(cv::Point2f(ellipseNodes->data.at(4), ellipseNodes->data.at(5)));
  imagePoints.push_back(cv::Point2f(ellipseNodes->data.at(6), ellipseNodes->data.at(7)));

  // Inner parameter
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 379.941650390625, 0.0, 316.7630615234375, 0.0, 379.57232666015625, 241.15725708007812, 0.0, 0.0, 1.0);
  cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << -0.05865439027547836, 0.07110843807458878, -0.00029281777096912265, -0.0006561076152138412, -0.022533254697918892);

  // SolvePNP
  cv::Mat rvec, tvec;
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

  // Print Rotation and Translation
  cv::Mat rotationMatrix;
  cv::Rodrigues(rvec, rotationMatrix);
  std::cout << "Rotation Matrix:\n" << rotationMatrix << std::endl;
  std::cout << "Translation Vector:\n" << tvec << std::endl;

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "camera_color_frame";
  transform.child_frame_id = "ring";

  tf2::Matrix3x3 rotation;
  tf2::Quaternion quaternion;
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          rotation[i][j] = rotationMatrix.at<double>(i, j);
      }
  }
  rotation.getRotation(quaternion);

  transform.transform.rotation.w = quaternion.w();
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();

  transform.transform.translation.x = tvec.at<double>(0, 0);
  transform.transform.translation.y = tvec.at<double>(1, 0);
  transform.transform.translation.z = tvec.at<double>(2, 0);

  // 发布变换
  tf_broadcaster.sendTransform(transform);
}