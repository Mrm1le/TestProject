#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/obstacle_grid.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/obstacle_grid_test.h"
#include "planning/common/timer.h"
#include "slog_macro_impl.h"

using namespace cv;
using namespace std;

using msd_planning::utils::Timer;
using msd_planning::utils::TotalTimer;

using namespace msquare;
using namespace msquare::hybrid_a_star_2;

constexpr int DEBUG_POLYGON = 29;

std::vector<cv::Vec3b> generateRandColors(int num){
    std::vector<cv::Vec3b> colors(num);

    for(int i=0; i<num; i++){
        cv::Vec3b color;

        color(0) = rand() & 255;
        color(1) = rand() & 255;
        color(2) = rand() & 255;
        
        if(i==10000){
            colors[i] = cv::Vec3b(255,255,255);
        }
        else if(i == DEBUG_POLYGON){
            colors[i] = cv::Vec3b(0,0,255);
        }
        else
        {
            colors[i] = color ;
        }


    }
    return colors;
}

cv::Mat drawVoroniIdxDiffImage(const cv::Mat& voroni_idx1, const cv::Mat& voroni_idx2, const std::vector<cv::Vec3b>& colors){
    cv::Mat canvas(voroni_idx1.rows, voroni_idx1.cols, CV_8UC3);
    for(int y=0; y<voroni_idx1.rows;y++){
        for(int x=0; x<voroni_idx1.cols; x++){
            int idx1 = voroni_idx1.at<int>(y,x);
            int idx2 = voroni_idx2.at<int>(y,x);
            if(idx1 != idx2)
            {
                canvas.at<cv::Vec3b>(y,x) = cv::Vec3b(255,255,255);
                SLOG_DEBUG(" vroni diff coord (x,y) = (%d %d) idx1=%d idx2=%d\n", x,y ,idx1, idx2);
            }
            else
            {
                canvas.at<cv::Vec3b>(y,x) = colors[idx1];
            }
        }
    }
    return canvas;
}

cv::Mat drawVoroniIdxImage(const cv::Mat& voroni_idx, const std::vector<cv::Vec3b>& colors){
    cv::Mat canvas(voroni_idx.rows, voroni_idx.cols, CV_8UC3);
    for(int y=0; y<voroni_idx.rows; y++){
        for(int x=0; x<voroni_idx.cols; x++){
            canvas.at<cv::Vec3b>(voroni_idx.rows-1-y,x) = colors[voroni_idx.at<int>(y,x)];
        }
    }
    return canvas;
}

bool drawVoroniCenters(cv::Mat& canvas, const std::vector<cv::Point2f>& points){
    for(size_t i=0; i<points.size(); i++){
        cv::circle(canvas, cv::Point2f(points[i].x, canvas.rows-1-points[i].y), 1, cv::Scalar(255,255,255), -1);

    }
    return true;
}

bool dumpPolygon(std::string file_name, const std::vector<cv::Point2f>& polygon){
    ofstream ofs(file_name);
    if(!ofs.is_open()){
        SLOG_ERROR("[dumpPolygon] open file failed: %s", file_name.c_str());
        return false;
    }
    char str[512];

    ofs << polygon.size() << endl;
    for(size_t i=0; i<polygon.size(); i++){
        sprintf(str, "%f %f", polygon[i].x, polygon[i].y);
        ofs << str << endl;
    }

    ofs.close();
    return true;
}

void testOnGivenPoints(std::string test_data_file_name, ObstacleHeightType height){
    if (!TmpGlobals::CONFIG_OBSTACLE_GRID_DEBUG) {
        SLOG_ERROR("[testOnGivenPoints] must be run with "
                   "CONFIG_OBSTACLE_GRID_DEBUG = true");
        return;
    }

    ObstacleGridTest obs_grid_test_;
    ObstacleGrid obs_grid_;

    if(!obs_grid_test_.loadTestDataFromFile(test_data_file_name)){
        SLOG_ERROR("load test data failed");
    }
    obs_grid_test_.constructGridGTWithoutVoroniFromObstaclePoints(obs_grid_test_.test_buffer_obstacle_points_);

    obs_grid_.init(obs_grid_test_.rows_,obs_grid_test_.cols_, obs_grid_test_.res_, obs_grid_test_.origin_x_, obs_grid_test_.origin_y_);
    obs_grid_.__test_constructFromObstaclePoints(obs_grid_test_.test_buffer_obstacle_points_);

    if (obs_grid_.voroni_faces_buffer_.size() > DEBUG_POLYGON) {
        dumpPolygon("test_obstacle_grid_polygon.txt",
                    obs_grid_.voroni_faces_buffer_[DEBUG_POLYGON]);
        SLOG_INFO("polygon %d dumped (%d vertex) ", DEBUG_POLYGON,
                  (int)obs_grid_.voroni_faces_buffer_[DEBUG_POLYGON].size());
    }

    SLOG_INFO("[Obstacle Grid Test] Running testGridDiff on obs_grid_ at height %d", (int)height);
    obs_grid_test_.testGridDiff(obs_grid_.grid_data_u16_[(int)height].data(),
                                obs_grid_.grid_data_scale_);
    obs_grid_test_.getTestGridDiffResult();
    



    obs_grid_test_.constructGridAndVoroniGTFromVoroniCenterPoints(obs_grid_.voroni_centers_buffer_);
    SLOG_INFO("[Obstacle Grid Test] Running testGridDiff on obs_grid_");
    obs_grid_test_.testGridDiff(obs_grid_.grid_data_u16_[(int)height].data(),
                                obs_grid_.grid_data_scale_);
    obs_grid_test_.getTestGridDiffResult();

    
    std::vector<cv::Vec3b> rand_colors = generateRandColors(10001);

    cv::Mat voroni_idx_gt_image = drawVoroniIdxImage(obs_grid_test_.voroni_index_gt_, rand_colors);
    drawVoroniCenters(voroni_idx_gt_image,obs_grid_.voroni_centers_buffer_ );
    cv::Mat voroni_idx_image = drawVoroniIdxImage(obs_grid_.voroni_buffer_, rand_colors);
    drawVoroniCenters(voroni_idx_image,obs_grid_.voroni_centers_buffer_ );

    cv::Mat voroni_idx_diff_image = drawVoroniIdxDiffImage(obs_grid_.voroni_buffer_, obs_grid_test_.voroni_index_gt_, rand_colors);
    
    cv::imshow("test_obstacle_grid_voroni_idx_gt_image", voroni_idx_gt_image);
    cv::imshow("test_obstacle_grid_voroni_idx_image", voroni_idx_image);
    cv::imshow("test_obstacle_grid_voroni_idx_diff_image", voroni_idx_diff_image);


    cv::imwrite("test_obstacle_grid_voroni_idx_gt_image.png", voroni_idx_gt_image);
    cv::imwrite("test_obstacle_grid_voroni_idx_image.png", voroni_idx_image);
    cv::imwrite("test_obstacle_grid_voroni_idx_diff_image.png", voroni_idx_diff_image);
    obs_grid_.dumpGridImage("test_obstacle_grid_image.png", height);


    cv::waitKey();
}

void testOnRandomPoints(int test_num, int rand_n, double xmin, double xmax, double ymin, double ymax, 
    int grid_rows, int grid_cols, double grid_origin_x, double grid_origin_y, double grid_res, ObstacleHeightType height,
    double integer_points_only_res = -1.0, bool verbose = true){
    ObstacleGridTest obs_grid_test_;
    ObstacleGrid obs_grid_;


    SLOG_INFO("Test ON rand_n = %d, x[%f, %f]y[%f,%f] integer_points_only_res = %f\n",
        rand_n, xmin, xmax, ymin, ymax, integer_points_only_res);

    SLOG_INFO("   grid rows, cols = %d %d, origin = %f %f, res = %f\n",
        grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res);
    
   
    obs_grid_.init(grid_rows, grid_cols, grid_res, grid_origin_x, grid_origin_y);
    obs_grid_test_.init(grid_rows, grid_cols, grid_res, grid_origin_x, grid_origin_y);

        
    constexpr bool enable_timer = true;

    auto construct_timer =
        TotalTimer<enable_timer, 1>("obstacle_grid_construct");

    auto gt_construct_timer =
        TotalTimer<enable_timer, 1>("obstacle_grid_gt_construct");

    int passed_count = 0;

    double max_positive_diff = 0.0;
    double max_negative_diff = 0.0;

    int avg_point_num = 0;;
    for(int ti=0; ti<test_num; ti++){
        std::vector<planning_math::Vec2d> point_list;
        for(int pi=0; pi<rand_n; pi++)
        {
            double x = (rand() / (double)RAND_MAX) * (xmax-xmin) + xmin;
            double y = (rand() / (double)RAND_MAX) * (ymax-ymin) + ymin;

            if(integer_points_only_res > 0.001){
                x = std::round(x / (double)integer_points_only_res) * integer_points_only_res;
                y = std::round(y / integer_points_only_res) * integer_points_only_res;
            }
            
            double gx = (x - grid_origin_x) / grid_res;
            double gy = (y - grid_origin_y) / grid_res;
            if(gx >=0 && gy >=0 && gx <=grid_cols-1 && gy <=grid_rows -1 ){
                point_list.push_back(planning_math::Vec2d(x,y));
                if(verbose){
                    SLOG_DEBUG("Generated  points : [%d] %f %f", pi, point_list.back().x(), point_list.back().y());
                }
            }
        }
        avg_point_num += point_list.size();

        
        obs_grid_test_.test_buffer_obstacle_points_ = point_list;

        gt_construct_timer.Tic();
        obs_grid_test_.constructGridGTWithoutVoroniFromObstaclePoints(point_list);
        gt_construct_timer.Toc();


        construct_timer.Tic();
        obs_grid_.__test_constructFromObstaclePoints(point_list);
        construct_timer.Toc();
        

        SLOG_DEBUG("[Obstacle Grid Test] Running testGridDiff on obs_grid_ --- [%d]", ti);
        obs_grid_test_.testGridDiff(obs_grid_.grid_data_u16_[(int)height].data(),
                                    obs_grid_.grid_data_scale_);
        bool test_passed = obs_grid_test_.getTestGridDiffResult();

        max_positive_diff = std::max(max_positive_diff, obs_grid_test_.test_result_max_grid_positive_diff_);
        max_negative_diff = std::min(max_negative_diff, obs_grid_test_.test_result_max_grid_negative_diff_);

        if(test_passed) passed_count ++;

        if(!test_passed){
            obs_grid_test_.dumpTestDataToFile("obstacle_grid_test_data_failed.txt");
            //break;
        }
    }

    
    if(passed_count == test_num)
    {
        SLOG_INFO("       finished TOTAL %d PASSED %d max_positive_diff = %f max_negative_diff = %f avg point num = %f\n", test_num, passed_count,max_positive_diff, max_negative_diff, avg_point_num/(float)test_num);
    }
    else 
    {
        SLOG_WARN("       finished TOTAL %d PASSED %d max_positive_diff = %f max_negative_diff = %f avg point num = %f\n", test_num, passed_count,max_positive_diff, max_negative_diff, avg_point_num/(float)test_num);
    }

}


int main( int argc, char** argv)
{
    int test_mode = 1;

    if(test_mode == 0)
    {
        std::string test_data_file_name = "obstacle_grid_test_data_0.txt";
        //std::string test_data_file_name = "obstacle_grid_test_data_failed.txt";

        testOnGivenPoints(test_data_file_name, ObstacleHeightType::HIGH);
    }
    else if(test_mode == 1)
    {
        int grid_rows = 600;
        int grid_cols = 600;
        double grid_origin_x = 0.0 - 15.0;
        double grid_origin_y = 0.0 - 15.0;
        double grid_res = 0.05;

        bool verbose = false;
        
        ObstacleHeightType height = ObstacleHeightType::HIGH;
        testOnRandomPoints(100, 100, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 200, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 500, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 1000, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 2000, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);


        testOnRandomPoints(100, 0, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 1, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 2, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 3, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 4, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);
        testOnRandomPoints(100, 5, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);

        testOnRandomPoints(100, 500, -50, 50, -50, 50, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, -1, verbose);

       

        testOnRandomPoints(100, 100, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 200, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 500, -15, 15, -15, 15, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 0, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 1, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 2, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 3, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 4, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
        testOnRandomPoints(100, 5, 0, 10, 0, 10, grid_rows, grid_cols, grid_origin_x, grid_origin_y, grid_res, height, 0.05, verbose);
       
        
    }
    return 0;
}


