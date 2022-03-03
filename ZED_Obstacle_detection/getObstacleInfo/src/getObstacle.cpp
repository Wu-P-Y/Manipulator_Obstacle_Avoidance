#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

// 深度图二值化
// deph_image 深度图	target_image 目标图 throld 距离阈值
void mask_depth(Mat& depth_image, Mat& targt_image, int throld = 1000)
{
	int rows = depth_image.rows;
	int cols = depth_image.cols;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (depth_image.at<ushort>(i, j) > throld)
			{
				targt_image.at<ushort>(i, j) = 0;
			}
		}
	}
}

// 寻找障碍物 返回凸集（构成凸包的点集）
// depth_image 深度图    thresh 当前阈值    max_thresh 最大阈值（参考cv::threshold函数）    area 最小面积
vector<vector<Point>> getObstacle(Mat& depth_image, int thresh = 5, int max_thresh = 255, int area  =500)
{
	// 二值化
	Mat dep;
	Mat gray;
	depth_image.copyTo(dep);
	//mask_depth(depth_image, dep, 1000);	// 二值化
	//dep.convertTo(dep, CV_8UC1, 1.0 / 16);
	//cvtColor(dep, gray, COLOR_BGR2GRAY);
	//imshow("gray", gray);	//二值化结果

	// 开运算
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));	// 构建核
	Mat open_output;
	morphologyEx(dep, open_output, MORPH_OPEN, element);		// 开运算去噪声
	//imshow("open", open_output);		//开运算结果

	// threshold函数二值化
	Mat threshold_output;
	threshold(dep, threshold_output, thresh, max_thresh, CV_THRESH_BINARY);
	imshow("output", threshold_output);
	threshold_output.convertTo(threshold_output, CV_8UC1, 1.0 / 16);
	cvtColor(threshold_output, gray, COLOR_BGR2GRAY);
	imshow("gray", gray);	//二值化结果

	// 寻找轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	// 计算凸包
	vector<vector<Point>> hull(contours.size());
	vector<vector<Point>> res;
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
	}

	// 绘制轮廓及凸包
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) < area)	// 如果轮廓包围面积小于area，则忽略当前轮廓
		{
			continue;
		}
		res.push_back(hull[i]);		// 符合条件的凸包加入结果

		RNG rng(-1);		// 生成一个64位int型的随机数
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));		// 随机生成一种颜色，返回Scalar类型

		// 绘制轮廓
		drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		// 绘制凸包
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}

	imshow("contours", drawing);
	return res;
}