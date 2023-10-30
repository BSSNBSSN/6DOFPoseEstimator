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
    split(image, channels);

    // Turn the image to gray and highlight the prefer color (red here)
    cv::Mat grayImage = cv::Mat(image.rows, image.cols, CV_8UC1);
    grayImage = 1 * channels.at(2) - 0.5 * channels.at(1) - 0.5 * channels.at(0);

    // Threshould and get the highlighted ring
    cv::Mat thresholded;
    cv::threshold(grayImage, thresholded, 100, 255, cv::THRESH_BINARY);

    // Erosion and dilation
    int kernelSize = 3;
    int iterations = 1;
    cv::Mat erosionDilationKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(thresholded, thresholded, erosionDilationKernel, cv::Point(-1, -1), iterations);
    cv::dilate(thresholded, thresholded, erosionDilationKernel, cv::Point(-1, -1), iterations);

    // Find the contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresholded, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    // Iterate contours
    int ellipseIndex = -1, ellipseArea = 1000;
    for (int i = 0; i < contours.size(); i++)
    {
        // contour with enough points for ellipse fit; no child contour; not too small
        if (contours[i].size() >= 5 && hierarchy[i][3] != -1)
        {
            if (contourArea(contours[i]) > ellipseArea)
            {
                ellipseIndex = i;
                // std::cout<<ellipseIndex<<" "<<std::endl;
                ellipseArea = contourArea(contours[i]);
            }
        }
    }

    if (ellipseIndex == -1)
    {
        sensor_msgs::ImagePtr dst = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        imageTopicPuber.publish(*dst);
        return;
    }

    std::vector<cv::Point> ellipseContour = contours[ellipseIndex];
    // Fit ellipses
    cv::RotatedRect ellipse = cv::fitEllipse(ellipseContour);
    cv::ellipse(image, ellipse, cv::Scalar(0, 255, 0), 2); // Draw the ellipse with green line

    cv::Point2f squareVertices[4];
    ellipse.points(squareVertices);

    cv::Point2f ellipseVertices[4];
    for (int i = 0; i <= 3; i++)
    {
        ellipseVertices[i].x = (squareVertices[i].x + squareVertices[(i + 1) % 4].x) / 2;
        ellipseVertices[i].y = (squareVertices[i].y + squareVertices[(i + 1) % 4].y) / 2;
    }

    cv::line(image, ellipseVertices[0], ellipseVertices[2], cv::Scalar(0, 255, 0), 2);
    cv::line(image, ellipseVertices[1], ellipseVertices[3], cv::Scalar(0, 255, 0), 2);

    std_msgs::Int32MultiArray msg;
    msg.data.resize(8);

    for (int i = 0; i <= 3; i++)
    {
        msg.data[i * 2] = ellipseVertices[i].x;
        msg.data[i * 2 + 1] = ellipseVertices[i].y;
    }

    sensor_msgs::ImagePtr dst = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    imageTopicPuber.publish(*dst);

    nodesTopicPuber.publish(msg);
}
