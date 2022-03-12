#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

std::vector<cv::Point> getObstacle(cv::Mat& depth_image, int thresh = 20, int max_thresh = 255, int area = 500);