#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

// ���ͼ��ֵ��
// deph_image ���ͼ	target_image Ŀ��ͼ throld ������ֵ
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

// Ѱ���ϰ��� ����͹��������͹���ĵ㼯��
// depth_image ���ͼ    thresh ��ǰ��ֵ    max_thresh �����ֵ���ο�cv::threshold������    area ��С���
vector<vector<Point>> getObstacle(Mat& depth_image, int thresh = 5, int max_thresh = 255, int area  =500)
{
	// ��ֵ��
	Mat dep;
	Mat gray;
	depth_image.copyTo(dep);
	//mask_depth(depth_image, dep, 1000);	// ��ֵ��
	//dep.convertTo(dep, CV_8UC1, 1.0 / 16);
	//cvtColor(dep, gray, COLOR_BGR2GRAY);
	//imshow("gray", gray);	//��ֵ�����

	// ������
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));	// ������
	Mat open_output;
	morphologyEx(dep, open_output, MORPH_OPEN, element);		// ������ȥ����
	//imshow("open", open_output);		//��������

	// threshold������ֵ��
	Mat threshold_output;
	threshold(dep, threshold_output, thresh, max_thresh, CV_THRESH_BINARY);
	imshow("output", threshold_output);
	threshold_output.convertTo(threshold_output, CV_8UC1, 1.0 / 16);
	cvtColor(threshold_output, gray, COLOR_BGR2GRAY);
	imshow("gray", gray);	//��ֵ�����

	// Ѱ������
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


	// ����͹��
	vector<vector<Point>> hull(contours.size());
	vector<vector<Point>> res;
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
	}

	// ����������͹��
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) < area)	// ���������Χ���С��area������Ե�ǰ����
		{
			continue;
		}
		res.push_back(hull[i]);		// ����������͹��������

		RNG rng(-1);		// ����һ��64λint�͵������
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));		// �������һ����ɫ������Scalar����

		// ��������
		drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		// ����͹��
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}

	imshow("contours", drawing);
	return res;
}