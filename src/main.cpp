#include "lidar_visualize.hpp"
#include "../include/cal_calibration.h"


int main(int argc, char **argv)
{
	if (argc != 2){std::cout << "require to enter file name!!"; return -1;}
	std::string file_dir = std::string("../pcds/") + std::string(argv[1]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	read_pcd_template<pcl::PointCloud<pcl::PointXYZI>, pcl::PointXYZI>(file_dir.c_str(), *xyzi_cloud);
	pcl::copyPointCloud(*xyzi_cloud, *cloud);
	interact_visualize(cloud);
	
	// calib::zhang_zhengyou_calib("D:/pg_cpp/calibration/imgs/20210613");
	//calib::cal_extrinsic_par();
	return 0;
}
