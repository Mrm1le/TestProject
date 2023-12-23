#include "has2_get_next_state_offline/next_state_visualizer.h"


namespace msquare
{

NextStateVisualizer::NextStateVisualizer()
{
    inited_ = false;
}
NextStateVisualizer::~NextStateVisualizer()
{
}

bool NextStateVisualizer::init(int canvas_width, int canvas_height, double meter_per_pixel)
{
    canvas_width_ = canvas_width;
    canvas_height_ = canvas_height;

    canvas_origin_x_ = canvas_width/2;
    canvas_origin_y_ = canvas_height/2;

    meter_per_pixel_ = meter_per_pixel;
    inited_ = true;

    canvas_ = cv::Mat(canvas_height, canvas_width, CV_8UC3);
    
    canvas_.setTo(cv::Scalar(0,0,0));

    return inited_;
}

cv::Mat NextStateVisualizer::getCanvas()
{
    return canvas_;
}

void NextStateVisualizer::drawStatePose(std::shared_ptr<SearchNode> state) 
{
    cv::Scalar color(0,255,0);
    double arrow_length = 0.05;
    //double origin_radius = 0.02;

    cv::Point2d p0 = coordToCanvas(state->x, state->y);
    cv::Point2d p1 = coordToCanvas(state->x+arrow_length*std::cos(state->theta), state->y+arrow_length*std::sin(state->theta));

    cv::line(canvas_, p0, p1, color);
    cv::circle(canvas_, p0, 5, color);
}

void NextStateVisualizer::drawStateInfo(int idx, std::shared_ptr<SearchNode> state)
{
    cv::Scalar color(0,128,128);

    cv::Point2d p0 = coordToCanvas(state->x, state->y);

    std::string str = state-> getNodeInfoStr();
    str = std::string("idx:") + std::to_string(idx) + " " + str;
    cv::putText(canvas_, str, p0, cv::FONT_HERSHEY_SIMPLEX, 1, color);

}

void NextStateVisualizer::drawGrid(double res)
{
    cv::Scalar color(128,128,128);
    int grid_max = 10;
    for(int gx = -grid_max; gx<=grid_max; gx++)
    {
        double x = res*gx;
        
        cv::Point2d p0 = coordToCanvas(x,-100);
        cv::Point2d p1 = coordToCanvas(x,+100);
        
        cv::line(canvas_, p0, p1, color, gx==0? 2:1);
    }

    for(int gy = -grid_max; gy<=grid_max; gy++)
    {
        double y = res*gy;
        cv::Point2d p0 = coordToCanvas(-100,y);
        cv::Point2d p1 = coordToCanvas(+100,y);
        
        cv::line(canvas_, p0, p1, color, gy==0? 2:1);
    }


}


cv::Point2d NextStateVisualizer::coordToCanvas(double x, double y)
{
    return cv::Point2d(y/meter_per_pixel_ + canvas_origin_y_, -x/meter_per_pixel_ + canvas_origin_x_);
}


}