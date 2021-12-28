#include "../include/cal_calibration.h"#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// windows
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   //define something for Windows (32-bit and 64-bit, this part is common)
   #include <io.h>

   #ifdef _WIN64
      //define something for Windows (64-bit only)
   #else
      //define something for Windows (32-bit only)
   #endif
#elif __linux__
	// linux
	#include <sys/io.h>
#endif


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

	 void zhang_zhengyou_calib(const char * folder_address) 
	 {
	 	std::vector<std::vector<cv::Point2f>> img_points_seq;
		  
	 	std::vector<std::string> files_vec;
	 	calib::getFileNames(folder_address, files_vec);
	 	std::cout << "files_vec.shape = " << files_vec.size() << std::endl;
	 	cv::Size image_size;
	 	cv::Size board_size(6, 8); // chessboard corner nums
	 	for (auto file = files_vec.begin(); file != files_vec.end(); ++file)
	 	{
	 		std::cout << *file << std::endl;
	 		cv::Mat origin_img = cv::imread(*file);
	 		cv::Mat gray_img;
	 		cv::cvtColor(origin_img, gray_img, cv::COLOR_RGB2GRAY);
	 		cv::Mat out_img;
			
	 		// 
	 		std::vector<cv::Point2f> img_points_buf;
	 		if (findChessboardCorners(origin_img, board_size, img_points_buf) == 0)
	 		{
	 			std::cout << "can not find chessboard corners!\n"; 
	 			exit(1);
	 		}
	 		else {
	 			std::cout << "success!" << std::endl;
	 			cv::Mat view_gray;
	 			cv::cvtColor(origin_img, view_gray, cv::COLOR_RGB2GRAY);
	 			cv::find4QuadCornerSubpix(view_gray, img_points_buf, cv::Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��
	 			img_points_seq.push_back(img_points_buf);
	 			cv::drawChessboardCorners(view_gray, board_size, img_points_buf, true); //��ͼƬ�б�ǽǵ�
	 			cv::namedWindow("Camera Calibration", cv::WINDOW_NORMAL);
	 			cv::imshow("Camera Calibration", view_gray);
	 			cv::waitKey(1000);
	 			image_size.width = view_gray.cols;
	 			image_size.height = view_gray.rows;
	 		}
			
	 	}
		
	 	std::cout << img_points_seq.size() << std::endl;
		
	 	cv::Size square_size(2.44, 2.44); 
	 	std::vector<std::vector<cv::Point3f>> object_points; 
		cv::Mat intrinsic_mat(3, 3, CV_32FC1, cv::Scalar::all(0)); 
	 	std::vector<int> point_counts;  
	 	cv::Mat distCoeffs(1, 5, CV_32FC1, cv::Scalar::all(0)); 
	 	std::vector<cv::Mat> tvecsMat;  
	 	std::vector<cv::Mat> rvecsMat; 
	 	
	 	int i, j, t;
	 	int img_count = img_points_seq.size();
	 	for (t = 0; t < img_count; t++)
	 	{
	 		std::vector<cv::Point3f> tempPointSet;
	 		for (i = 0; i < board_size.height; i++)
	 		{
	 			for (j = 0; j < board_size.width; j++)
	 			{
	 				cv::Point3f realPoint;
	 				realPoint.x = i * square_size.width;
	 				realPoint.y = j * square_size.height;
	 				realPoint.z = 0;
	 				tempPointSet.push_back(realPoint);
	 			}
	 		}
	 		object_points.push_back(tempPointSet);
	 	}

	 	for (i = 0; i < img_count; i++)
	 	{
	 		point_counts.push_back(board_size.width * board_size.height);
	 	}
	 	cv::calibrateCamera(object_points, img_points_seq, image_size, intrinsic_mat, distCoeffs, rvecsMat, tvecsMat, 0);
	 	double total_err = 0.0; 
	 	double err = 0.0; 
		std::vector<cv::Point2f> image_points2; 
	 	for (i = 0; i < img_count; i++)
	 	{
	 		std::vector<cv::Point3f> tempPointSet = object_points[i];
	 		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], intrinsic_mat, distCoeffs, image_points2);
	 		std::vector<cv::Point2f> tempImagePoint = img_points_seq[i];
	 		cv::Mat tempImagePointMat(1, tempImagePoint.size(), CV_32FC2);
	 		cv::Mat image_points2Mat(1, image_points2.size(), CV_32FC2);
	 		for (int j = 0; j < tempImagePoint.size(); j++)
	 		{
	 			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
	 			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
	 		}
	 		err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
	 		total_err += err /= point_counts[i];
	 	}
	 	std::cout << "intrinsic matrix = " << intrinsic_mat << std::endl;
		
	 };

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
	 // recursively get file name
	 void getFileNames(std::string path, std::vector<std::string>& files)
	 {
	 	intptr_t hFile = 0;
	 	struct _finddata_t fileinfo;
	 	std::string p;
	 	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	 	{
	 		do
	 		{
	 			if ((fileinfo.attrib & _A_SUBDIR))
	 			{
					// 递归遍历
	 				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
	 					getFileNames(p.assign(path).append("\\").append(fileinfo.name), files);
	 			}
	 			else
	 			{
	 				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
	 			}
	 		} while (_findnext(hFile, &fileinfo) == 0);
	 		_findclose(hFile);
	 	}
	 }
#elif
	 // linux cannot use _findnext, _findfirst


#endif
}