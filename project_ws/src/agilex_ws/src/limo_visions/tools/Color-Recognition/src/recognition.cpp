#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // int thre = std::stod(argv[1]);
    // std::cout << thre << std::endl;

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cv::Mat frame;

    if(!cap.isOpened())
    {
        return -1;
    }

    cv::Mat img, gray;
    while (true)
    {
        cap >> frame;
        img = frame.clone();
        cv::cvtColor(img,  gray, cv::COLOR_RGB2GRAY);
        
         // 颜色空间转换
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // 颜色阈值  //使用inRange过滤像素
    Mat mask;
    // 黄色
    // inRange(hsv, Scalar(5, 110, 90), Scalar(50, 207, 220), mask);
    inRange(hsv, Scalar(124, 45, 100), Scalar(140, 255, 255), mask);
 

    // 形态学操作  开闭操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // 轮廓检测
    vector<vector<Point>> contours;
    findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 形状匹配
    for (size_t i = 0; i < contours.size(); i++)
    {
        double match = matchShapes(contours[i], contours[0], CONTOURS_MATCH_I1, 0);
        double area = cv::contourArea(contours[i], false);  //计算轮廓的面积
        if (area < 100)
            continue;
        if (match < 0.1)
        {
            Rect rect = boundingRect(contours[i]);
            cv::Point p(rect.x, rect.y-10);
            rectangle(img, rect, Scalar(0, 255, 0), 2);
            cv::putText(img, "purple", p, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2, 8);
        }
    }
        
        cv::imshow("img", img);
        
        char k = cv::waitKey(30);
        if(k == 'q')
            break;
    }

    return 0;

}
