#include "../include/cal_calibration.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>


namespace calib 
{
	void cal_extrinsic_par()
	{
		double dist[5] = { -0.1942, 0.0140, -5.358e-4, 0.0016, 0.
		};
		cv::Mat dist_array(5, 1, CV_64F, dist);
		/*double Intrinsic[3][3] = {
			1915.49, 1.311865401806629, 977.23,
			0., 1925.84, 538.86,
			0.,  0.,  1.};*/
		double Intrinsic[3][3] = {
			1.6634617699999999e+03, 0., 9.7235897999999997e+02,
			0., 1.6652231500000000e+03, 5.1716867000000002e+02,
			0.,  0.,  1. };

		cv::Mat Int(3, 3, CV_64F, Intrinsic);

		std::vector <cv::Point3f> vec_3d;
		cv::Mat mat_3d;
		vec_3d.clear();
		vec_3d.push_back(cv::Point3f(3.040, 0.108, 0.401));
		vec_3d.push_back(cv::Point3f(2.878, 0.119, -0.023));
		vec_3d.push_back(cv::Point3f(2.907, -0.286, -0.011));
		vec_3d.push_back(cv::Point3f(3.064, -0.292, 0.398));
		cv::Mat(vec_3d).convertTo(mat_3d, CV_32F);

		// /////////////////////////////////////////////////
		// vec origin_img
		// /////////////////////////////////////////////////
		std::vector <cv::Point2f> vec_2d;
		cv::Mat mat_2d;
		vec_2d.clear();
		vec_2d.push_back(cv::Point2f(887, 279));
		vec_2d.push_back(cv::Point2f(882, 571));
		vec_2d.push_back(cv::Point2f(1168, 569));
		vec_2d.push_back(cv::Point2f(1156, 288));
		cv::Mat(vec_2d).convertTo(mat_2d, CV_32F);

		cv::Mat rvec;
		cv::Mat tvec;
		// 使用opencv提供的PnP算法
		cv::solvePnP(mat_3d, mat_2d, Int, dist_array, rvec, tvec);
		cv::Mat rotM;
		Rodrigues(rvec, rotM);
		std::cout << "rvec = " << rvec << std::endl;
		std::cout << "tvec = " << tvec << std::endl;
		std::cout << "rotM = " << rotM << std::endl;

	};

	 
}