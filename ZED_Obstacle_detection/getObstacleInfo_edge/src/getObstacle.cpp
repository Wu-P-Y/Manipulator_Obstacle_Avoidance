// 通过canny函数提取边缘

#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;



// 寻找障碍物 返回重心
// image 原图    thresh 当前阈值    max_thresh 最大阈值   area 最小面积
std::vector<cv::Point> getObstacle(Mat& image, int threshold = 127, int max_threshold = 255, int area  =500)
{
	vector<vector<Point>> res;
	vector<Point> centers;

	// 二值化
	Mat src;
	Mat gray;
	Mat canny;
	image.copyTo(src);

	// 转灰度图
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

	// canny边缘检测
	cv::Canny(gray, canny, threshold, max_threshold, 3);
	imshow("canny", canny);

	// 二值化
	//Mat threshold_out;
	//cv::threshold(gray, threshold_out, thresh, max_thresh, cv::THRESH_BINARY);

	// 寻找轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	cv::findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	// 计算凸包
	vector<vector<Point>> hull(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
	}

	 // 绘制轮廓及凸包
	Mat drawing = Mat::zeros(canny.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) < area)	// 如果轮廓包围面积小于area，则忽略当前轮廓
		{
			continue;
		}
		res.push_back(hull[i]);		// 符合条件的凸包加入结果

		RNG rng(12345);		// 生成一个64位int型的随机数
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));		// 随机生成一种颜色，返回Scalar类型

		// 绘制轮廓
		drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		// 绘制凸包
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}

	imshow("contours", drawing);

	// 计算物体位置信息， 返回障碍物重心像素坐标
	Point pt[10000];
	Moments moment;		// 矩

	for (int i = 0; i >= 0; i = hierarchy[i][0])
	{
		Mat temp(contours.at(i));
		Scalar color(0, 0, 255);
		moment = moments(temp, false);
		if (contourArea(contours[i]) < area)		// 如果轮廓包围面积小于area，则忽略当前轮廓
		{
			continue;
		}
		if (moment.m00 != 0)
		{
			pt[i].x = cvRound(moment.m10 / moment.m00);		// 计算重心横坐标
			pt[i].y = cvRound(moment.m01 / moment.m00);		// 计算重心纵坐标
		}
		//cout << pt[i].x << ";" << pt[i].y << endl;
		centers.push_back(pt[i]);
	}
	

	return centers;
}