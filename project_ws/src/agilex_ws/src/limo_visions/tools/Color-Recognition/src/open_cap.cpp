#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cv::Mat frame;

    if(!cap.isOpened())
    {
        return -1;
    }

    while (true)
    {
        cap >> frame;
        cv::imshow("view", frame);
        char k = cv::waitKey(30);
        if(k == 'q')
            break;
    }

    return 0;

}
