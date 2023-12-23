#pragma once

#include <opencv2/opencv.hpp>
class PolygonVisualizer
{
    cv::Mat canvas_;

    int rows_;
    int cols_;

    double res_;      
    double inv_res_;

    double origin_x_;
    double origin_y_;

public:
    PolygonVisualizer() {};
    ~PolygonVisualizer() {};
  bool init(int rows, int cols, double res, double origin_x, double origin_y) {
      if (rows <= 0 || cols <= 0 || res <= 0) {
        return false;
      }
      canvas_.create(rows, cols, CV_8UC3);
      canvas_.setTo(cv::Scalar(0,0,0));

      
      origin_x_ = origin_x;
      origin_y_ = origin_y;
      res_ = res;
      inv_res_ = 1.0 / res_;
      rows_ = rows;
      cols_ = cols;



      return true;
  }

  cv::Mat getCanvas(){ return canvas_; }

  void clearCanvas() { canvas_.setTo(cv::Scalar(0,0,0));}

  void drawLine(const cv::Scalar& color, cv::Point2f p0, cv::Point2f p1) {
      cv::Point2d p0_in_canvas_ = coordToCanvas(p0.x, p0.y);
      cv::Point2d p1_in_canvas_ = coordToCanvas(p1.x, p1.y); 
      cv::line(canvas_, p0_in_canvas_, p1_in_canvas_, color, 1);
  }

  void drawPolygon(const cv::Scalar& color, const std::vector<cv::Point2f>& points) {
    for(std::size_t i=0; i<points.size(); i++){
      drawLine(color, points[i], points[(i+1)%points.size()]);
      cv::Point2d p = coordToCanvas(points[i].x, points[i].y); 
      cv::circle(canvas_, p, 5, color, -1);

      char str[512];
      sprintf(str, "[%d]%.3f,%.3f", (int)i, points[i].x, points[i].y);
      cv::putText(canvas_, str, cv::Point2d(p.x, p.y-5) ,0 ,0.5, color);

    }
  }

  void drawDotsInsidePolygon(const cv::Scalar& color, const std::vector<cv::Point2i>& points){
    for(std::size_t i=0; i<points.size(); i++){
      
      cv::Point2d p0_in_canvas_ = coordToCanvas(points[i].x, points[i].y); 
      cv::circle(canvas_, p0_in_canvas_, 5, color, -1);
    }
  }

  void drawGrid(const cv::Scalar& color, double grid_tick = 1.0){
      double grid_max = 100;
     
      //int grid_style = 0;

      for(double x = -grid_max + origin_x_; x<=grid_max + origin_x_; x+=grid_tick)
      {
          cv::Point2d p0 = coordToCanvas(x, origin_y_ - 1000);
          cv::Point2d p1 = coordToCanvas(x, origin_y_ + 1000);
          cv::line(canvas_, p0, p1, color, std::abs(x - origin_x_)<1e-5? 2:1);
      }

      for(double y = -grid_max + origin_y_; y<=grid_max + origin_y_;  y+=grid_tick)
      {

          cv::Point2d p0 = coordToCanvas(origin_x_ - 1000, y);
          cv::Point2d p1 = coordToCanvas(origin_y_ + 1000, y);
          
          cv::line(canvas_, p0, p1, color, std::abs(y - origin_y_)<1e-5? 2:1);
      }

      for(double x = -grid_max + origin_x_; x<=grid_max + origin_x_; x+=grid_tick)
      {
          for(double y = -grid_max + origin_y_; y<=grid_max + origin_y_; y+=grid_tick)
          {
              cv::Point2d p = coordToCanvas(x, y);
              cv::circle(canvas_, p, 2, color, -1);

              char str[512];
              sprintf(str, "%d,%d", (int)std::round(x), (int)std::round(y));
              cv::putText(canvas_, str, cv::Point2d(p.x, p.y-10) ,0 ,0.4, color);

          }

          
      }

  }

private:
    cv::Point2d coordToCanvas(double x, double y)
    {
      return cv::Point2d((x - origin_x_) * inv_res_ + cols_/2, (y - origin_y_) * inv_res_ + rows_ / 2);
    }


};
