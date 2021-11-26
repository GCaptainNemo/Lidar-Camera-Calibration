#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

template<class PointCloud, class Point_Type>
void read_pcd_template(const char * file_dir, PointCloud &point_cloud) 
{
	
	if (pcl::io::loadPCDFile<Point_Type>(file_dir, point_cloud) == -1) 
	{
		std::cout << "cant load file" << std::endl;
	}
}

// linux cannot use _findnext, _findfirst
// void read_pcds_xyz(const std::string & dir, const int & frame_num, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
// 	intptr_t handle;  
// 	struct _finddata_t fileinfo;
// 	std::string p;
// 	handle = _findfirst(p.append(dir).append("/*.pcd").c_str(), &fileinfo);
// 	if (handle == -1) {
// 		return;
// 		std::cout << "handle == -1" << std::endl;
// 	}
// 	std::vector<std::string> files;

// 	do
// 	{
// 		printf("%s\n", fileinfo.name);
// 		files.push_back(p.assign(dir).append("/").append(fileinfo.name));
// 	} while (!_findnext(handle, &fileinfo));
// 	_findclose(handle);
// 	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	for (int frame = 0; frame < frame_num; frame++)
// 	{
// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame(new pcl::PointCloud<pcl::PointXYZ>);
		
// 		if (pcl::io::loadPCDFile<pcl::PointXYZ>(files[frame].c_str(), *cloud_frame) == -1) {
// 			std::cout << "Couldn't read file" << "\n";
// 		}
// 		else {
// 			*cloud += *cloud_frame;
// 		}
// 	}
// }


template <class T>
void interact_visualize(T cloud)
{
	//visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

	cloud_mutex.lock();    // for not overwriting the point cloud
	// Display pointcloud:
	viewer->addPointCloud(cloud, "bunny");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	// Spin until 'Q' is pressed:
	viewer->spin();
	std::cout << "done." << std::endl;

	cloud_mutex.unlock();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
