#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    int thre = std::stod(argv[1]);
    std::cout << thre << std::endl;

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
        
        cv::Mat binary;
        cv::threshold(gray, binary, thre, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::imshow("binary", ~binary);
        // 查找轮廓
    
        std::vector<std::vector<cv::Point> > contours;
	    std::vector<cv::Vec4i> hiearachy;
	    cv::findContours(~binary, contours, hiearachy,  cv::RETR_EXTERNAL,  cv::CHAIN_APPROX_SIMPLE);
 
	    // cv::Point2f center;
	    // float radius;
	    // minEnclosingCircle(contours[0], center, radius);

        // cv::circle(img, center, radius, cv::Scalar(0, 0, 255), 2, 8);


        // 查找凸包
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::convexHull(contours[i], hull[i]);
    }

    // 查找最小外接矩形
    std::vector<cv::RotatedRect> min_rects(hull.size());
    for (size_t i = 0; i < hull.size(); i++)
    {
        min_rects[i] = cv::minAreaRect(hull[i]);
    }

    // 绘制最小外接矩形
    cv::Mat rect_image = img.clone();
    for (size_t i = 0; i < min_rects.size(); i++)
    {
        cv::Point2f vertices[4];
        min_rects[i].points(vertices);
        for (int j = 0; j < 4; j++)
        {
            line(rect_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
    }
    //     cv::imshow("view", rect_image);
    //     cv::imshow("binary", bin_img);
        
        
        cv::imshow("img", rect_image);
        
        char k = cv::waitKey(30);
        if(k == 'q')
            break;
    }

    return 0;

}
