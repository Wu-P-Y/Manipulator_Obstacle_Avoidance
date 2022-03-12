/***********************************************************************************************
 ** 从ZED2相机获得障碍物信息																								  	      **
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// Opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/cvconfig.h>

#include <getObstacle.hpp>

#include<iostream>

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);		// ZED格式Mat转为OpenCV格式Mat
#ifdef HAVE_CUDA
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);
#endif // HAVE_CUDA

int main(int argc, char** argv) {

	std::cout << "press \'s\' to get obstacle info"<<std::endl;
	std::cout << "press \'q\' to quit"<<std::endl;

	// 新建一个ZED相机对象
	Camera zed;

	// 设置初始化参数
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION::HD1080;		// 分辨率
	init_params.depth_mode = DEPTH_MODE::ULTRA;				// 深度模式
	init_params.coordinate_units = UNIT::MILLIMETER;						// 长度单位
	if (argc > 1) init_params.input.setFromSVOFile(argv[1]);		// 如果在参数中指定了文件，则以SVO格式读入

	// 打开相机
	ERROR_CODE err = zed.open(init_params);
	if (err != ERROR_CODE::SUCCESS)
	{
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1;			// 如果打开相机时发生错误则退出
	}

	// 设置运行参数
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

	// 准备一个新的图片格式来保存原图片一半分辨率的图片
	Resolution image_size = zed.getCameraInformation().camera_resolution;
	int new_width = image_size.width / 2;
	int new_height = image_size.height / 2;

	Resolution new_image_size(new_width, new_height);

	// 使用slMat2cvMat改变图片的类型
	// cv::Mat和sl::Mat在内存中是一样的，改变的只是指针
	Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
	cv::Mat image_ocv = slMat2cvMat(image_zed);

#ifndef HAVE_CUDA // If no cuda, use CPU memory
	Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
#else
	Mat depth_image_zed_gpu(new_width, new_height, MAT_TYPE::U8_C4, sl::MEM::GPU); // alloc sl::Mat to store GPU depth image
	cv::cuda::GpuMat depth_image_ocv_gpu = slMat2cvMatGPU(depth_image_zed_gpu); // create an opencv GPU reference of the sl::Mat
	cv::Mat depth_image_ocv; // cpu opencv mat for display purposes
#endif

	Mat Point_cloud(new_width, new_height, MAT_TYPE::U8_C4 );
	

	// 直到'q'被按下前循环
	char key = ' ';
	while (key != 'q')
	{
		if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
		{
			// 抓取左相机图片，深度图
			zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
#ifndef HAVE_CUDA 
			// retrieve CPU -> the ocv reference is therefore updated
			zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size);
#else
			// retrieve GPU -> the ocv reference is therefore updated
			zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);
#endif

			zed.retrieveMeasure(Point_cloud, MEASURE::XYZRGBA);

			cv::imshow("org", image_ocv);
			//cv::imshow("depth", depth_image_ocv);	

			// 键盘输入's'，保存图片并获取障碍物坐标
			while (key == 's') 
			{
				cv::imwrite("img.jpg", image_ocv);
				
				cv::Mat img;
				img = cv::imread("img.jpg");
				cv::imshow("img", img);

				// 获取障碍并显示重心三维坐标
				vector<cv::Point> centers = getObstacle(img);
				for (int i = 0; i < centers.size(); i++)
				{
					sl::float4 point_cloud_value;
					Point_cloud.getValue(centers[i].x, centers[i].y, &point_cloud_value);
					cout << point_cloud_value.x << ";" << point_cloud_value.y << ";" << point_cloud_value.z << endl;
				}

				std::cout << "-----------------------------------------" << std::endl;

				key = ' ';
			}

			// 处理键盘输入
			key = cv::waitKey(10);
		}
	}
#ifdef HAVE_CUDA
	// sl::Mat GPU memory needs to be free before the zed
	depth_image_zed_gpu.free();
#endif
	zed.close();
	return 0;
}

// MAT_TYPE和CV_TYPE间的映射
int getOCVtype(sl::MAT_TYPE type) {
	int cv_type = -1;
	switch (type) {
	case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	return cv_type;
}

// cv::Mat类型和sl::Mat类型间的转换
cv::Mat slMat2cvMat(Mat& input) {
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

#ifdef HAVE_CUDA
/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input) {
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}
#endif