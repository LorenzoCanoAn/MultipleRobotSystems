#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> // drawing shapes
#include <string>
#include <unistd.h>
#include <iostream>


const float HRVO_TWO_PI = 6.283185307179586f;

class graphicsHandler
{
private:
	cv::Scalar background_;
	cv::Mat image_;
	size_t height_, width_, botSize_;
	std::string windowName_;
	float scale_factor_;

public:
	graphicsHandler(size_t height, size_t width, double scale_fator, size_t botSize_);
	~graphicsHandler();
	void resetImage(void);
	void drawRobot(double x, double y, cv::Scalar color);
	void showAndClearImage();
	std::string get_window_name(void);
};