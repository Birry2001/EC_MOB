#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main()
{

	cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);

	Mat img;
	Mat imgHSV, mask, imgColor;
	int hmin = 0, smin = 0, vmin = 0;
	int hmax = 179, smax = 255, vmax = 255;

	namedWindow("Trackbars", (640, 200)); // Create Window
	createTrackbar("Hue Min", "Trackbars", &hmin, 179);
	createTrackbar("Hue Max", "Trackbars", &hmax, 179);
	createTrackbar("Sat Min", "Trackbars", &smin, 255);
	createTrackbar("Sat Max", "Trackbars", &smax, 255);
	createTrackbar("Val Min", "Trackbars", &vmin, 255);
	createTrackbar("Val Max", "Trackbars", &vmax, 255);

	while (true) {

		cap.read(img);
		cvtColor(img, imgHSV, COLOR_BGR2HSV);

		Scalar lower(hmin, smin, vmin);
		Scalar upper(hmax, smax, vmax);

		inRange(imgHSV, lower, upper, mask);
		// hmin, smin, vmin, hmax, smax, vmax;
		cout << "hsv_min: " << hmin << ", " << smin << ", " << vmin << "  "; 
		cout << "hsv_max: " << hmax << ", " << smax << ", " << vmax << endl;
		imshow("Image", img);
		imshow("Mask", mask);
		if (waitKey(30) == 'q')
			break;
	}
}
