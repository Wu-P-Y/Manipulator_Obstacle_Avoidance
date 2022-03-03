#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

void mask_depth(cv::Mat& depth_image, cv::Mat& targt_image, int throld = 1000);
std::vector<std::vector<cv::Point>> getObstacle(cv::Mat& depth_image, int thresh = 20, int max_thresh = 255, int area = 500);