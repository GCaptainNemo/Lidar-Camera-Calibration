#pragma once

namespace calib {
	// use solvePnP function to calculate extrinsic parameters(rotation matrix and translate vector)
	void cal_extrinsic_par();

	// zhangzhengyou calibration to calculate intrinsic parameters and distortion coefficient.
	void zhang_zhengyou_calib(const char * folder_address);
	
	// get files name in path 
	void getFileNames(std::string path, std::vector<std::string>& files);
	
	
}