/***********************************************************************************************
 ** ��ZED2���������ͼ																								  	      **
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// Opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/cvconfig.h>

#include <getObstacle.hpp>


using namespace sl;

cv::Mat slMat2cvMat(Mat& input);		// ZED��ʽMatתΪOpenCV��ʽMat
#ifdef HAVE_CUDA
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);
#endif // HAVE_CUDA

int main(int argc, char** argv) {
	// �½�һ��ZED�������
	Camera zed;

	// ���ó�ʼ������
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION::HD1080;		// �ֱ���
	init_params.depth_mode = DEPTH_MODE::ULTRA;				// ���ģʽ
	init_params.coordinate_units = UNIT::METER;						// ���ȵ�λ
	if (argc > 1) init_params.input.setFromSVOFile(argv[1]);		// ����ڲ�����ָ�����ļ�������SVO��ʽ����

	// �����
	ERROR_CODE err = zed.open(init_params);
	if (err != ERROR_CODE::SUCCESS)
	{
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1;			// ��������ʱ�����������˳�
	}

	// �������в���
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

	// ׼��һ���µ�ͼƬ��ʽ������ԭͼƬһ��ֱ��ʵ�ͼƬ
	Resolution image_size = zed.getCameraInformation().camera_resolution;
	int new_width = image_size.width / 2;
	int new_height = image_size.height / 2;

	Resolution new_image_size(new_width, new_height);

	// ʹ��slMat2cvMat�ı�ͼƬ������
	// cv::Mat��sl::Mat���ڴ�����һ���ģ��ı��ֻ��ָ��
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

	// ֱ��'q'������ǰѭ��
	char key = ' ';
	while (key != 'q')
	{
		if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
		{
			// ץȡ�����ͼƬ�����ͼ
			zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
#ifndef HAVE_CUDA 
			// retrieve CPU -> the ocv reference is therefore updated
			zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size);
#else
			// retrieve GPU -> the ocv reference is therefore updated
			zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);
#endif

			cv::imshow("org", image_ocv);
			cv::imshow("depth", depth_image_ocv);

			// ��ȡ͹������ʾ
			std::vector<std::vector<cv::Point>> hull = getObstacle(depth_image_ocv);

			// �����������
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

// MAT_TYPE��CV_TYPE���ӳ��
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

// cv::Mat���ͺ�sl::Mat���ͼ��ת��
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