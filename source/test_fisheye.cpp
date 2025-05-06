#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp> // 添加这一行以使用 cv::fisheye

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    cv::Mat K = (cv::Mat_<double>(3, 3) << 3.0245305983229298e+02, 0., 4.9664001463163459e+02,
         0., 3.2074618594392325e+02, 3.3119980984361649e+02,
         0.0, 0.0, 1.0);

    cv::Mat D = (cv::Mat_<double>(4, 1) << -4.3735601598704078e-02, 2.1692522970939803e-02, -2.6388839028513571e-02, 8.4123126605702321e-03 );

    // cv::Mat raw_image = imread("/opt/csg/slam/navs/front.png");
    cv::Mat raw_image = imread("/opt/csg/slam/navs/edu_demo.jpg");
    std::cout << raw_image.cols  << " " << raw_image.rows << std::endl;
    int width  =  raw_image.cols / 2.  ;
    int height =  raw_image.rows / 2. ;
    std::cout << width  << " " << height << std::endl;
    cv::Mat map1, map2;
    cv::Mat undistortImg;
    cv::Size imageSize(width, height);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K, imageSize, CV_16SC2, map1, map2);
    // cout << "map1:" << map1 << "\nmap2:" << map2 << endl;
    std::cout << map1.cols  << " " << map1.rows << std::endl;
    std::cout << map2.cols  << " " << map2.rows << std::endl;
    cv::remap(raw_image, undistortImg, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::imwrite("/opt/csg/slam/navs/edu_undistorted.jpg", undistortImg);


    return 0;
}
