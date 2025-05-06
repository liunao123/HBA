#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int main() {
    // 读取灰度图像
    Mat image = imread("/home/1_equa.png", IMREAD_GRAYSCALE);
    
    if (image.empty()) {
        std::cerr << "Error: Unable to read image file." << std::endl;
        return -1;
    }

    // 定义核（结构元素）用于膨胀和腐蚀
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

    // 膨胀处理
    Mat dilated;
    dilate(image, dilated, element);

    // 提取边缘
    Mat dilated_edges;
    Canny(dilated, dilated_edges, 150, 200);
    
    Mat edges;
    Canny(image, edges, 150, 200);

    // 腐蚀处理
    // Mat eroded;
    // erode(edges, eroded, element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( dilated_edges ,contours, hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE  ,cv::Point());  
    std::cout << "159 contours.size: " << contours.size() << std::endl;

    cv::Mat imageContours=Mat::zeros(edges.size(),CV_8UC1);  
    cv::Mat S_Contours=Mat::zeros(edges.size(),CV_8UC1);  //绘制  
    for(int i=0;i<contours.size();i++)  
    {  
        if ( contours[i].size() < 100 )
        {
            continue;
        }
        
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
        for(int j=0;j<contours[i].size();j++)   
        {  
            //绘制出contours向量内所有的像素点  
            Point P=Point(contours[i][j].x,contours[i][j].y);  
            S_Contours.at<uchar>(P)=255;  
        }  
        //输出hierarchy向量内容  
        // char ch[256];  
        // sprintf(ch,"%d",i);  
        // string str=ch;  
        // cout<<"向量hierarchy的第" << i <<" 个元素内容为：" << hierarchy[i] <<endl<<endl;  
        //绘制轮廓  
        drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);  
    } 
    // imshow("Contours Image",imageContours); //轮廓  
    imshow("Point of Contours",S_Contours);   //向量contours内保存的所有轮廓点集  


    // 显示原始图像、膨胀后图像、腐蚀后图像和边缘图像
    // imshow("Original Image", image);
    // imshow("Dilated Image", dilated);
    imshow("Edges", edges);
    imshow("dilated_edges Image", dilated_edges);

    waitKey(0);

    return 0;
}