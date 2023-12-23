#include <opencv2/opencv.hpp>
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"

namespace msquare
{

class NextStateVisualizer
{

    cv::Mat canvas_;

    int canvas_width_;
    int canvas_height_;
    double meter_per_pixel_;      //

    int canvas_origin_x_;
    int canvas_origin_y_;

    bool inited_;
   


public:
    NextStateVisualizer();
    ~NextStateVisualizer();
    bool init(int canvas_width, int canvas_height, double meter_per_pixel_);
    cv::Mat getCanvas();

    void drawStatePose(std::shared_ptr<SearchNode> state);

    void drawStateInfo(int idx, std::shared_ptr<SearchNode> state);

    void drawGrid(double res);

private:
    cv::Point2d coordToCanvas(double x, double y);
    

};



}