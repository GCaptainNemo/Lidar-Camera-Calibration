#include "../include/cal_calibration.h"
#include <io.h>


namespace calib {

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
		cv::solvePnP(mat_3d, mat_2d, Int, dist_array, rvec, tvec);
		cv::Mat rotM;
		Rodrigues(rvec, rotM);
		std::cout << "rvec = " << rvec << std::endl;
		std::cout << "tvec = " << tvec << std::endl;
		std::cout << "rotM = " << rotM << std::endl;

	};

	void zhang_zhengyou_calib(const char * folder_address) 
	{
		std::vector<cv::Point2f> image_points_buf;  

		std::vector<std::string> files_vec;
		calib::getFileNames(folder_address, files_vec);
		std::cout << "files_vec.shape = " << files_vec.size() << std::endl;
		cv::Size board_size(6, 8);
		for (auto file = files_vec.begin(); file != files_vec.end(); ++file)
		{
			std::cout << *file << std::endl;
			cv::Mat origin_img = cv::imread(*file);
			cv::Mat gray_img;
			cv::cvtColor(origin_img, gray_img, cv::COLOR_RGB2GRAY);
			cv::Mat out_img;
			// 
			if (0 == findChessboardCorners(origin_img, board_size, image_points_buf))
			{
				std::cout << "can not find chessboard corners!\n"; //找不到角点
				exit(1);
			}
			else {
				std::cout << "success!" << std::endl;
			}
			//cv::cornerHarris(gray_img, out_img, 10, 3, 0.1);
			//cv::Mat harrisCorner;
			//// binarize
			//cv::threshold(out_img, harrisCorner, 0.00001, 255, cv::THRESH_BINARY);
			//cv::namedWindow("image", cv::WINDOW_NORMAL);
			//cv::imshow("image", harrisCorner);
			//cv::imshow("image", gray_img);
		}


		
	};

	void getFileNames(std::string path, std::vector<std::string>& files)
	{
		//文件句柄
		//注意：我发现有些文章代码此处是long类型，实测运行中会报错访问异常
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		std::string p;
		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//如果是目录,递归查找
				//如果不是,把文件绝对路径存入vector中
				if ((fileinfo.attrib & _A_SUBDIR))
				{
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


}