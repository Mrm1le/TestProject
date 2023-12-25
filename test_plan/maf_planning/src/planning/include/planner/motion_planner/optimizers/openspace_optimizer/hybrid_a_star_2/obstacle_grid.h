#pragma once

// TODO: Jinwei: refactor this file with access control and format
// only init grid_data_f32_buffer_ in constructFromSbpObstacleMultiHeights now
// so only this function is safety

#define OBSTACLE_GRID_DEBUG_DUMP_IMAGE

#include <vector>

#include <opencv2/opencv.hpp>

#include "../sbp_obstacle_interface.h"
#include "common/sbp_obstacle_point.h"
#include "multi_circle_footprint_model.h"
#include "planning/common/timer.h"
#include "search_node.hpp"
#include "tmp_usage_only_global_variables.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

struct MultiModelMinDistances {
  enum : uint8_t {
    LOW_WHEELS = 0,
    NOT_ALLOW_CLOSE = 1,
    FULL_BODY = 2,
    MODEL_NUM
  };

  float distances[MODEL_NUM];
};

class ObstacleGrid {
public:
  bool inited_;

  // enabling height layer
  std::vector<ObstacleHeightType> enabling_heights_;

  float res_;
  float inv_res_;
  float max_error_;
  float origin_x_, origin_y_;

  // grid_data_u16_[y * cols_ + x] * grid_data_scale_ = minimun distance from
  // grid center to closest obstacle

  std::vector<std::uint16_t>
      grid_data_u16_[(int)ObstacleHeightType::HEIGHT_TYPE_NUM];
  float grid_data_scale_;

  //This buffer is only used for debugging voroni index
  // so a buffer of voroni-center index is required.  (will be removed in the future)
  cv::Mat voroni_buffer_;

  cv::Mat grid_data_f32_buffer_;
  std::vector<cv::Point2f> voroni_centers_buffer_;
  std::vector<std::vector<cv::Point2f> > voroni_faces_buffer_;
  std::vector<cv::Point2l> facei_converted_to_point2i_buffer_;

  int rows_;
  int cols_;
  // gx, gy = math.floor((x-origin_x_)/res), math.floor((y-origin_y_)/res)
  //      (x,y) : coordinate in meters from local coordinate
  //      (gx,gy) : grid coordinate

public:
  double debug_sum_abs_diff = 0.0;
  double debug_max_positive_diff = 0.0;
  double debug_max_minus_diff = 0.0;
  int debug_calcdis_function_called_num = 0;
  int debug_obs_point_num = 0;

public:
  ObstacleGrid() {
    inited_ = false;
    res_ = 0.0f;
    inv_res_ = 0.0f;
    max_error_ = 0.0f;
    origin_x_ = 0.0f;
    origin_y_ = 0.0f;
    rows_ = 0;
    cols_ = 0;
    grid_data_scale_ = 0.001f;
  }

  ~ObstacleGrid() {}

  bool isInited() const { return inited_; }
  float max_error() const { return max_error_; }

  bool init(int rows, int cols, float res, float origin_x, float origin_y,
            bool debug = false) {
    if (rows <= 0 || cols <= 0 || res <= 0.0f) {
      return false;
    }
    if (debug) {
      voroni_buffer_.create(rows, cols, CV_32S);
    }

    origin_x_ = origin_x;
    origin_y_ = origin_y;
    res_ = res;
    inv_res_ = 1.0f / res;
    max_error_ = std::sqrt(2.0f) * 0.5f * res;
    rows_ = rows;
    cols_ = cols;

    //only enable these two heights
    enabling_heights_.clear();
    enabling_heights_.push_back(ObstacleHeightType::LOW);
    enabling_heights_.push_back(ObstacleHeightType::HIGH);
    for (const auto &height : enabling_heights_) {
      grid_data_u16_[(int)height].resize(rows_ * cols_);
    }

    inited_ = true;
    return true;
  }

  bool constructFromSbpObstacleMultiHeights(
      const std::vector<SbpObstaclePoint> &discrete_obs) {

    // divide obstacles into multiple heights
    std::vector<SbpObstaclePoint>
        discrete_obs_by_heights[(int)ObstacleHeightType::HEIGHT_TYPE_NUM];
    for (std::size_t i = 0; i < discrete_obs.size(); i++) {
      ObstacleHeightType height = discrete_obs[i].getHeightType();
      if (height == ObstacleHeightType::UNKNOWN) {
        height = ObstacleHeightType::HIGH;
      }
      discrete_obs_by_heights[(int)height].emplace_back(discrete_obs[i]);
    }

    for (std::size_t i = 0; i < enabling_heights_.size(); i++) {
      ObstacleHeightType height = enabling_heights_[i];
      constructFromSbpObstacle(discrete_obs_by_heights[(int)height], height);
    }
    return true;
  }

  bool constructFromSbpObstacleMultiHeights(
      const std::vector<SbpObstaclePoint> &discrete_obs,
      MultiCircleFootprintModel &footprint, const SearchNode &pose,
      float inflation_high, float inflation_low) {
    // divide obstacles into multiple heights
    std::vector<SbpObstaclePoint>
        discrete_obs_by_heights[(int)ObstacleHeightType::HEIGHT_TYPE_NUM];
    for (std::size_t i = 0; i < discrete_obs.size(); i++) {
      ObstacleHeightType height = discrete_obs[i].getHeightType();
      if (height == ObstacleHeightType::UNKNOWN) {
        height = ObstacleHeightType::HIGH;
      }
      discrete_obs_by_heights[(int)height].emplace_back(discrete_obs[i]);
    }

    footprint.updatePose(pose.x(), pose.y(), pose.cos_theta(),
                         pose.sin_theta());

    grid_data_f32_buffer_.create(rows_, cols_, CV_32F);
    for (std::size_t i = 0; i < enabling_heights_.size(); i++) {
      ObstacleHeightType height = enabling_heights_[i];
      float inflation = 0.0f;
      int footprint_model_index = -1;
      if (height == ObstacleHeightType::HIGH) {
        inflation = inflation_high;
        footprint_model_index = MultiModelMinDistances::FULL_BODY;
      } else if (height == ObstacleHeightType::LOW) {
        inflation = inflation_low;
        footprint_model_index = MultiModelMinDistances::LOW_WHEELS;
      } else {
        continue;
      }
      constructFromSbpObstacle(discrete_obs_by_heights[int(height)], height,
                               footprint, footprint_model_index, pose,
                               inflation);
    }
    grid_data_f32_buffer_.release();
    return true;
  }

  bool constructFromSbpObstacle(
      const std::vector<SbpObstaclePoint> &discrete_obs,
      ObstacleHeightType height, const MultiCircleFootprintModel &footprint,
      int footprint_model_index, const SearchNode &pose, float inflation) {
    const auto &model = footprint.models()[footprint_model_index];
    std::vector<int> circle_used(footprint.circles().size(), 0);
    for (int circle_index : model.circle_index) {
      circle_used[circle_index] = 1;
    }
    for (int model_index : model.include_model) {
      for (int circle_index : footprint.models()[model_index].circle_index) {
        circle_used[circle_index] = 1;
      }
    }

    std::vector<MultiCircleFootprintModel::Circle> circles;
    circles.reserve(footprint.circles().size());
    for (std::size_t i = 0; i < circle_used.size(); i++) {
      if (circle_used[i] == 1) {
        circles.push_back(footprint.circles()[i]);
      }
    }
    std::vector<float> needed_r_square(circles.size());
    for (std::size_t i = 0; i < circles.size(); i++) {
      float needed_r = circles[i].radius + inflation;
      needed_r_square[i] = needed_r * needed_r;
    }

    std::vector<cv::Point2f> obstacle_points_in_grid;
    for (std::size_t i = 0; i < discrete_obs.size(); i++) {
      const std::vector<planning_math::Vec2d> &obs_points =
          discrete_obs[i].getPoints();
      for (const auto &point : obs_points) {
        bool valid = true;
        for (std::size_t k = 0; k < circles.size(); k++) {
          float dx = float(point.x()) - circles[k].center_x;
          float dy = float(point.y()) - circles[k].center_y;
          valid = valid && (dx * dx + dy * dy > needed_r_square[k]);
        }
        if (valid) {
          float obs_x_in_grid = (point.x() - origin_x_) * inv_res_;
          float obs_y_in_grid = (point.y() - origin_y_) * inv_res_;
          obstacle_points_in_grid.emplace_back(obs_x_in_grid, obs_y_in_grid);
        }
      }
    }
    return constructFromPointsInGrid(obstacle_points_in_grid, height);
  }

  void
  constructGridFillVoroniConvexPolyWithDistanceFloat( cv::Mat& img, cv::Point2f* vf, int npts, float voroni_center_x, float voroni_center_y, int debug_only_voroni_id)
  {
      //[Code Explanation By Fenix.Shi]
      //这个函数是重写了OpenCV的FillConvexPoly函数，原函数的功能为将图像中的一块凸多边形区域着固定颜色。效率非常高。
      //    改造后变为将图像（实际上是obstacle_grid）中的多边形区域，per-peixel地填充为该grid到voroni-center的距离
      //
      // 为了满足构建Obstacle Grid的需求对FillConvexPoly做了以下改造
      //      1. （重要）FillConvexPoly 计算完每行的[x_left,x_right]后，使用memset填充固定的像素值，这里将“填充颜色”替换成逐grid（pixel）计算到最近障碍物的distance。
      //            这样可以一步到位，避免先填充voroni-center-id，再计算距离的2次计算。
      //            也能通过少量的重复计算，规避掉在2个多边形交界的区域只填充单个veroni-center-id可能造成的误差。（本方法在2个多边形的重叠区域，会计算多次distance，取最小值）
      //      2. （重要）原函数只支持由整数点构成的多边形的计算，实际我们的需求是填充浮点数构成的多边形的“内整点”，这里修改了实现使得支持浮点输入。
      //      3. （重要）FillConvexPoly 除了填充多边形内部外，还有填充多变形的边缘的算法（同时支持多种线型包括AntiAlis），我们不需要，予以删除，在边缘直接用填充算法取代
      //      4. shift功能不需要直接删掉
      //      5. 其他细节修改，不一一说明
      // 以下是填充Polygon的流程解释 =====================================================================================
      //
      //
      // Opencv convex polygon的填充采用的是水平行扫描的方式，即从miny（多边形的最小y）坐标开始，按顺序递增y，逐行扫描直到maxy
      //    对于每一个y行，都计算的到该行与多边形的2个交点的x坐标。然后填充介于[x_left,x_right]之间像素即达成结果。由于任意convex polygon，任一水平行至多与2条边相交，所以该方法可以成立。
      //    在扫描的过程中，高效实现的关键是维护2个“edge”。即与当前的y行相交的2条边的信息。edge中只有非常有限的信息在必要的时候更新计算，因此速度可以非常快。
      //
      // edge所维护的信息内容
      //
      //     - idx：edge的“目标顶点“的index。常规理解一条边必须有一个起点和一个终点。但Edge的”起点信息“ 始终被维护在（x，y）里，所以只需要明确edge的终点即可
      //
      //     - x：第y行（当前扫描行），与edge相交的交点x，也可以认为是当前edge的起点位置。 (x,y)就是当前在该edge上扫描所在的起点坐标。由于y已经在外层循环里维护了，这里只需要记录x即可。
      //
      //     - dx：y递增1时，x的增量。只有当edge的idex更新时，dx才需要被重新计算。每次y递增1时，x只要递增dx即可
      //
      //     - ye：edge的终点的y坐标（也就是v[idx].y）。一旦扫描y达到了ye，说明这条edge被耗尽了，需要被更新到下一条
      //
      //     - di：edge的更新方向。2个edge的di固定为1和npts-1。
      //          在初始化时，edge的idx都被设定为y最小的顶点。
      //          在edge耗尽（y>=ye）的时候，其中一条edge朝着idx +1 的方向更新，另一条edge朝着idx-1（也就是 (idx + npts-1)%npts）的方向更新到下一个目标顶点。
      //
      // 填充时刻：
      //     1. 常规填充 ： y递增1时，计算左edge和右edge的x，然后填充介于介于[x_left,x_right]的grid
      //     2. 水平边填充： 当出现水平边（v[i].y == v[i+1].y）时，会在y>edige[i-1].y，edge由i-1更新到i的时候，跳过i，直接更新到i+1，，因此此时需要额外填充介于 y[v[i].x，v[i+1].x]的像素


    struct
    {
      int idx, di;
      double x, dx;
      float ye;
      int ye_floor;

      std::string getInfo()
      {
          char buf[512];
          sprintf(buf, "idx = %d di = %d x = %f dx = %f ye = %f(%d)", idx, di, 
              x,  dx, ye, ye_floor);
          return buf;
      }
    }
    edge[2];

    int i, y, imin = 0;
    int edges = npts;
    cv::Size size = img.size();

    float xmin, xmax, ymin, ymax;

    xmin = xmax = vf[0].x;
    ymin = ymax = vf[0].y;

    for( i = 0; i < npts; i++ ) {
      cv::Point2f p = vf[i];
      if( p.y < ymin )
      {
        ymin = p.y;
        imin = i;
      }
      ymax = std::max( ymax, p.y );
      xmax = std::max( xmax, p.x );
      xmin = std::min( xmin, p.x );
    }

  
    if( npts < 3 || (int)xmax < 0 || (int)ymax < 0 || (int)xmin >= size.width || (int)ymin >= size.height )
      return;
    
    if(ymax > size.height-1) ymax = size.height - 1;

    int ymin_ceil = std::ceil(ymin);
    int ymax_floor = std::floor(ymax);

    edge[0].idx = edge[1].idx = imin;
    edge[0].ye = edge[1].ye = ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;
    edge[0].x = edge[1].x = -1.0f;
    edge[0].dx = edge[1].dx = 0;

    edge[0].ye_floor = edge[1].ye_floor = std::floor(ymin);

    y = ymin_ceil;


    for(y=ymin_ceil; y<=ymax_floor; y++){
      float *ptr = (float*)img.ptr(y);

      for( i = 0; i < 2; i++ ){
        if( y > edge[i].ye_floor || y == ymin_ceil) {  //ready to find next available edge
          int idx0 = edge[i].idx, di = edge[i].di;
          int idx = idx0 + di;
          if (idx >= npts) idx -= npts;
          float ty;
          int ty_floor;


          for (; edges-- > 0; ) {
            ty = vf[idx].y;
            ty_floor = std::floor(vf[idx].y);
            if (ty_floor > y){
              float xs = vf[idx0].x;
              float xe = vf[idx].x;

              float ye = vf[idx].y;
              float ys = vf[idx0].y;

              edge[i].ye = ty;
              edge[i].ye_floor = std::floor(ye);
              edge[i].dx = (xe - xs) / (ye - ys);
              edge[i].x = xs + edge[i].dx * (y - ys);
              edge[i].idx = idx;

              break;
            } else if(ty_floor == y && y != ymin) { //[Key Modification 2] extra fill for horizontal edgs
            
              int xx1, xx2;
              if(vf[idx0].x > vf[idx].x) {
                  xx1 = std::ceil(vf[idx].x - 0.1f);
                  xx2 = std::floor(vf[idx0].x + 0.1f);
              } else {
                  xx1 = std::ceil(vf[idx0].x - 0.1f);
                  xx2 = std::floor(vf[idx].x + 0.1f);
              }
              //----  FILL row y bettwin grids between [xx1, xx2]

              if( xx2 >= 0 && xx1 < size.width && y>=0 && y <= size.height-1) {
                if( xx1 < 0 ) xx1 = 0;
                if( xx2 >= size.width ) xx2 = size.width - 1;
              
                float dx, dy, dis;
                int x;
                // dy does not need to recalculate in the same row
                dy = voroni_center_y - y;                 

                //[Fenix] ALL grids are compared because edge grids need to be calculated in all possible polygons
                //   this is a simple implementation that does not care the difference between edge grids and body grids, but works faster
                            
                for(x=xx1; x<=xx2; x++) {                   
                  dx = voroni_center_x - x;

                  dis = std::sqrt(dx*dx+dy*dy) * res_; 
                  if(dis < ptr[x]) {
                    ptr[x] = dis;
                  }
                }
              }
            }
            idx0 = idx;
            idx += di;
            if (idx >= npts) idx -= npts;
          }
        }
      }

      // if (edges < 0)
      //   break;

      if (y >= 0)
      {
        int left = 0, right = 1;
        if (edge[0].x > edge[1].x)
        {
            left = 1, right = 0;
        }
        // -1 && +1 are for edge compensation (might have better ways)
        int xx1 = std::ceil(edge[left].x - 0.1f);
        int xx2 = std::floor(edge[right].x + 0.1f);
        
        if( xx2 >= 0 && xx1 < size.width )
        {
          if( xx1 < 0 ) xx1 = 0;
          if( xx2 >= size.width ) xx2 = size.width - 1;

          //[Key Modification 1]
          //per pixel fill with distance to voroni-center

          float dx, dy, dis;
          int x;
          // dy does not need to recalculate in the same row
          dy = voroni_center_y - y;                 

          //[Fenix] ALL grids are compared because edge grids need to be calculated in all possible polygons
          //   this is a simple implementation that does not care the difference between edge grids and body grids, but works faster
          
     
          for(x=xx1; x<=xx2; x++)
          {                   
            dx = voroni_center_x - x;

            dis = std::sqrt(dx*dx+dy*dy) * res_;
            if (dis < ptr[x]) {
              ptr[x] = dis;
            }
          }
        }
      }
      edge[0].x += edge[0].dx;
      edge[1].x += edge[1].dx;
    }


  }

  inline bool convertSbpObstacleToPointsInGrid(
      const std::vector<SbpObstaclePoint> &discrete_obs,
      std::vector<cv::Point2f> &out_obstacle_points_in_grid) {
    out_obstacle_points_in_grid.clear();
    for (std::size_t i = 0; i < discrete_obs.size(); i++) {
      const std::vector<planning_math::Vec2d> &obs_points =
          discrete_obs[i].getPoints();
      for (std::size_t j = 0; j < obs_points.size(); j++) {
        float obs_x_in_grid = (obs_points[j].x() - origin_x_) * inv_res_;
        float obs_y_in_grid = (obs_points[j].y() - origin_y_) * inv_res_;
        out_obstacle_points_in_grid.emplace_back(
            cv::Point2f(obs_x_in_grid, obs_y_in_grid));
      }
    }
    return true;
  }

  bool __test_constructFromObstaclePoints(const std::vector<planning_math::Vec2d>& points){
    std::vector<cv::Point2f> points_in_grid(points.size());
    for(std::size_t i=0; i<points.size(); i++){
      points_in_grid[i].x = (points[i].x() - origin_x_) * inv_res_;
      points_in_grid[i].y = (points[i].y() - origin_y_) * inv_res_;
    }
    return constructFromPointsInGrid(points_in_grid, ObstacleHeightType::HIGH);
  }

  bool
  constructFromSbpObstacle(const std::vector<SbpObstaclePoint> &discrete_obs,
                           ObstacleHeightType height) {
    std::vector<cv::Point2f> obstacle_points_in_grid;
    convertSbpObstacleToPointsInGrid(discrete_obs, obstacle_points_in_grid);

    return constructFromPointsInGrid(obstacle_points_in_grid, height);
  }

  bool constructFromPointsInGrid(const std::vector<cv::Point2f> &points_in_grid,
                                 ObstacleHeightType height) {
    if (points_in_grid.size() == 0) {
      grid_data_u16_[(int)height].clear();
      grid_data_u16_[(int)height].resize(rows_ * cols_,
                            std::numeric_limits<std::uint16_t>::max());
      return true;
    }
    cv::Subdiv2D subdiv(cv::Rect(0, 0, cols_, rows_));
    for (std::size_t i = 0; i < points_in_grid.size(); i++) {
      if(points_in_grid[i].x >=0 && points_in_grid[i].y >=0 && points_in_grid[i].x <=cols_-1 && points_in_grid[i].y <=rows_-1){
        subdiv.insert(cv::Point2f(points_in_grid[i].x, points_in_grid[i].y));
      }
    }
    
    
    // auto get_voroni_timer =  msd_planning::utils::Timer<true>("get_voroni");
    // auto fill_polygon_timer =  msd_planning::utils::TotalTimer<true>("fill_convex_poly");
    
    //get_voroni_timer.Tic();
    subdiv.getVoronoiFacetList(std::vector<int>(), voroni_faces_buffer_, voroni_centers_buffer_);
    //get_voroni_timer.Toc();

    std::vector<cv::Point2l> facei_converted_to_point2i;

    grid_data_f32_buffer_.setTo(10000.0);

    for (size_t i = 0; i < voroni_faces_buffer_.size(); i++) {
      facei_converted_to_point2i.resize(voroni_faces_buffer_[i].size());
      for (size_t j = 0; j < voroni_faces_buffer_[i].size(); j++) {
        facei_converted_to_point2i[j] =
            cv::Point2l(std::round(voroni_faces_buffer_[i][j].x),
                        std::round(voroni_faces_buffer_[i][j].y));
      }
      // constructGridFillVoroniConvexPolyWithDistance2(grid_data, ( cv::Point2l*)facei_converted_to_point2i.data(), facei_converted_to_point2i.size(),
      //   voroni_centers_buffer_[i].x, voroni_centers_buffer_[i].y, (int)i);
      //fill_polygon_timer.Tic();
      constructGridFillVoroniConvexPolyWithDistanceFloat(grid_data_f32_buffer_, ( cv::Point2f*)voroni_faces_buffer_[i].data(), voroni_faces_buffer_[i].size(),
        voroni_centers_buffer_[i].x, voroni_centers_buffer_[i].y, (int)i);
      // fill_polygon_timer.Toc();
    }

    grid_data_u16_[(int)height].resize(rows_ * cols_);
    float *grid_data_ptr = (float *)grid_data_f32_buffer_.data;
    for (std::size_t i = 0; i < grid_data_u16_[(int)height].size(); i++) {
      grid_data_u16_[(int)height][i] = std::min(
          std::max(std::round(grid_data_ptr[i] / grid_data_scale_), 0.0f),
          float(std::numeric_limits<std::uint16_t>::max()));
    }
    return true;
  }

  float getDistance(const MultiCircleFootprintModel::Circle &circle,
                    ObstacleHeightType height) const {
    int gx = std::round((circle.center_x - origin_x_) * inv_res_);
    int gy = std::round((circle.center_y - origin_y_) * inv_res_);
    float radius = circle.radius;
    if (gx < 0 || gx >= cols_ || gy < 0 || gy >= rows_) {
      return -radius;
    }
    return float(grid_data_u16_[int(height)][gy * cols_ + gx]) *
               grid_data_scale_ -
           radius;
  }

  MultiModelMinDistances calcMultiModelMinDistance(
      const MultiCircleFootprintModel &footprint_model) const {
    MultiModelMinDistances result;
    for (std::size_t hi = 0; hi < enabling_heights_.size(); hi++) {
      ObstacleHeightType height = enabling_heights_[hi];
      for (std::size_t mi = 0;
           mi < footprint_model.modelsByHeight(height).size(); mi++) {
        const MultiCircleFootprintModel::Model &model =
            footprint_model.modelsByHeight(height)[mi];

        float min_distance;
        if (model.include_model.empty()) {
          min_distance = std::numeric_limits<float>::infinity();
        } else {
          min_distance =
              result.distances[model.include_model[0]]; // max one include
        }
        for (int ci : model.circle_index) {
          float distance = getDistance(footprint_model.circles()[ci], height);
          if (distance < min_distance) {
            min_distance = distance;
          }
        }
        result.distances[model.index] = min_distance;
      }
    }
    return result;
  }

  bool dumpGridImage(const std::string &filename, ObstacleHeightType height) {
    cv::Mat debug_image_grayscale;
    cv::Mat(rows_, cols_, CV_16U, grid_data_u16_[int(height)].data())
        .convertTo(debug_image_grayscale, CV_32F, grid_data_scale_);
    cv::threshold(debug_image_grayscale, debug_image_grayscale, 0.0f, 0,
                  cv::THRESH_TOZERO);
    cv::threshold(debug_image_grayscale, debug_image_grayscale, 1.0f, 0,
                  cv::THRESH_TRUNC);
    cv::Mat((1.0f - debug_image_grayscale) * 255)
        .convertTo(debug_image_grayscale, CV_8U);

    cv::Mat debug_image_colored(rows_, cols_, CV_8UC3);
    cv::applyColorMap(debug_image_grayscale, debug_image_colored,
                      cv::COLORMAP_JET);
    cv::resize(debug_image_colored, debug_image_colored, cv::Size(1600, 1600));
    return cv::imwrite(filename, debug_image_colored);
  }

  bool
  dumpcalcMinDistanceImage(const std::string &filename,
                           const MultiCircleFootprintModel &footprint_model,
                           ObstacleHeightType height, std::string debug_txt) {
    cv::Mat debug_image_grayscale(rows_, cols_, CV_8UC1);
    cv::Mat debug_image_colored(rows_, cols_, CV_8UC3);

    float max_val_visual = 2.0f;
    float min_val_visual = 0.0f;
    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        float value_normalized =
            (float(grid_data_u16_[(int)height][(rows_ - 1 - gy) * cols_ + gx]) *
                 grid_data_scale_ -
             min_val_visual) /
            (max_val_visual - min_val_visual);
        value_normalized = std::min(value_normalized, 1.0f);
        value_normalized = std::max(value_normalized, 0.0f);
        debug_image_grayscale.at<uchar>(gy, gx) =
            255 - (uchar)(value_normalized * 255);
      }
    }

    cv::applyColorMap(debug_image_grayscale, debug_image_colored,
                      cv::COLORMAP_JET);

    for (const auto &circle : footprint_model.circles()) {
      int gx = std::round((circle.center_x - origin_x_) / res_);
      int gy = std::round((circle.center_y - origin_y_) / res_);
      cv::circle(debug_image_colored, cv::Point2i(gx, rows_ - 1 - gy),
                 circle.radius / res_, cv::Scalar(0, 255, 255), -1);
    }
    cv::resize(debug_image_colored, debug_image_colored, cv::Size(1600, 1600));

    cv::putText(debug_image_colored, debug_txt, cv::Point2d(50, 50), 0, 1.0,
                cv::Scalar(255, 255, 255));
    return cv::imwrite(filename, debug_image_colored);
  }
};

} // namespace hybrid_a_star_2

} // namespace msquare
