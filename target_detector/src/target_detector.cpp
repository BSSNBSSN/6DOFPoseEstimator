#include "target_detector.h"

void TargetDetector::TargetDetectCallback(const sensor_msgs::ImageConstPtr &cameraImage)
{
  cv::Mat image = cv_bridge::toCvShare(cameraImage, "bgr8")->image;

  if (image.empty())
  {
    std::cerr << "Could not open or find the image." << std::endl;
    return;
  }
  
  std::vector<cv::Mat> channels;
  split(image,channels);

  // 将图像转换为灰度图像
  cv::Mat grayImage = cv::Mat(image.rows, image.cols, CV_8UC1);
  grayImage = 1 * channels.at(2) - 0.5 * channels.at(1) - 0.5 * channels.at(0);

  // 使用阈值化来提取椭圆环
  cv::Mat thresholded;
  cv::threshold(grayImage, thresholded, 200, 255, cv::THRESH_BINARY);

  // 寻找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 遍历轮廓并拟合椭圆
  for (const auto &contour : contours)
  {
    if (contour.size() >= 5)
    {
      cv::RotatedRect ellipse = cv::fitEllipse(contour);
      cv::ellipse(image, ellipse, cv::Scalar(0, 255, 0), 2); // 用绿色画出拟合的椭圆
    }
  }

  // 显示图像
  // cv::imshow("Result", image);
  // cv::waitKey(0);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  nodesTopicPuber.publish(*msg);
}
