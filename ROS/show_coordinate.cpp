#include <ros/ros.h>
#include<highgui.h>
#include<cv.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include <vector>
#include "common.h"

string photo_path, output_name, intrinsic_path;
cv::RNG  random_number_generator;
cv::Mat src_img, gray_img;
std::vector<cv::Point2f> corners(0);

void writeData(const string filename, const float x, const float y, uint mode) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    switch(mode) {
        case(0):
            outfile << "photo" << endl;
            outfile << "1" << endl;
            break;
        case(1):
            outfile << "2" << endl;
            break;
        case(2):
            outfile << "3" << endl;
            break;
        case(3):
            outfile << "4" << endl;
            break;
        default:
            cout << "[writeData] - Code error, unknown mode" << endl;
            exit(0);
    }
    outfile << float2str(x) << "        " << float2str(y) << endl;
}

void on_mouse(int event, int x, int y, int flags, void* img)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        IplImage *timg = cvCloneImage((IplImage *)img);
        CvPoint pt = cvPoint(x, y);
        char temp[16];
        ROS_INFO("temp: (%d,%d)", x, y);
        sprintf(temp, "(%d,%d)", x, y);
        cvPutText(timg, temp, pt, &font, CV_RGB(250,0,0));
        cvCircle(timg, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
        cvShowImage("src", timg);
        cvReleaseImage(&timg);

        cv::Point2f p;
        p.x = x;
        p.y = y;
        corners.push_back(p);

    }
    
}


void getParameters() {
    std::cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_photo_path", photo_path)) {
        std::cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("ouput_path", output_name)) {
        std::cout << "Can not get the value of ouput_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        std::cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "drawCoordinate");
    getParameters();
    
    src_img = cv::imread(photo_path);

    if(src_img.empty()) {  // use the file name to search the photo
        cout << "No Picture found by filename: " << photo_path << endl;
        return 0;
    }

    std::vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    std::vector<float> distortion;
    getDistortion(intrinsic_path, distortion);

	// set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion
    // ///////////////////////////////////////////////////////
    // choose corner
    cv::Mat clone_src_img = src_img.clone();
    IplImage *img = new IplImage(clone_src_img);
    cvNamedWindow("src", 1);
    cvSetMouseCallback("src", on_mouse,img);
    cvShowImage("src", img);
    char *obj_key = "q";
    while ((char)cv::waitKey(1) != *obj_key) {}
    // cvWaitKey(0);
    // cvDestroyAllWindows();
    delete img;
    // cvReleaseImage(&img);
    // corners-> std::vector<cv::Point>
    if (!corners.size()) {
        cout << "No input corners, end process" << endl;
        return 0;
    }
    cv::Size winSize = cv::Size(5, 5);
	cv::Size zerozone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
    
    // cv::namedWindow("output", CV_WINDOW_KEEPRATIO);
    ROS_INFO("########## show result image ############\n");
    cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria);

    // cv::Mat result_img = src_img.clone();
    for(uint t = 0; t < corners.size(); ++t) {
        // cv::circle(result_img, corners[t], 3, cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255)), 1, 8, 0);
        printf("(%.3f %.3f)", corners[t].x, corners[t].y);
        writeData(output_name, corners[t].x, corners[t].y, t);
    }
    
    cout << endl << "Result saved, tap a random key to finish the process" << endl;
    // cv::namedWindow("output");
    // imshow("output", result_img);
    // cv::waitKey(0);
    return 0;
}
