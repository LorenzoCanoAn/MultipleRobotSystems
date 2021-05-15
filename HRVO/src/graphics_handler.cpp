#include <graphics_handler.hpp>



graphicsHandler::graphicsHandler(size_t height, size_t width, double scale_factor, size_t botSize_): 
    background_(20,20,20),
    image_(height, width, CV_8UC3, background_),
    height_(height),
    width_(width),
    botSize_(botSize_),
    windowName_("Display"),
    scale_factor_(scale_factor)
{
     
}

graphicsHandler::~graphicsHandler()
{
    cv::destroyAllWindows();
}

void graphicsHandler::drawRobot(double x, double y, cv::Scalar color)
{
    double x_d, y_d;
    x_d = x*scale_factor_ + ((double)width_)/2;
    y_d = - y*scale_factor_ + ((double)height_)/2;
    cv::Point2d point(x_d,y_d);
    cv::circle(image_,point, botSize_, color, -1);
    
}

void graphicsHandler::showAndClearImage(void)
{
    cv::imshow(windowName_,image_);
    cv::waitKey(0);
    image_ = cv::Mat(height_, width_, CV_8UC3, background_);
}

std::string graphicsHandler::get_window_name(void)
{
    return windowName_;
}