#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "has2_obstacle_grid_voroni/polygon_visualizer.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/obstacle_grid.h"
#include "planning/common/timer.h"
#include "slog_macro_impl.h"
#include "utils/polygon.hpp"

using namespace cv;
using namespace std;

using msd_planning::utils::Timer;
using msd_planning::utils::TotalTimer;

using namespace putils;



enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1<<12) - 256 };

/*
static void
Line( Mat& img, Point pt1, Point pt2,
      const void* _color, int connectivity = 8 )
{
    if( connectivity == 0 )
        connectivity = 8;
    else if( connectivity == 1 )
        connectivity = 4;

    LineIterator iterator(img, pt1, pt2, connectivity, true);
    int i, count = iterator.count;
    int pix_size = (int)img.elemSize();
    const uchar* color = (const uchar*)_color;

    for( i = 0; i < count; i++, ++iterator )
    {
        uchar* ptr = *iterator;
        if( pix_size == 1 )
            ptr[0] = color[0];
        else if( pix_size == 3 )
        {
            ptr[0] = color[0];
            ptr[1] = color[1];
            ptr[2] = color[2];
        }
        else
            memcpy( *iterator, color, pix_size );
    }
}
*/

static inline void ICV_HLINE_X(uchar* ptr, int64_t xl, int64_t xr, const uchar* color, int pix_size)
{
    uchar* hline_min_ptr = (uchar*)(ptr) + (xl)*(pix_size);
    uchar* hline_end_ptr = (uchar*)(ptr) + (xr+1)*(pix_size);
    uchar* hline_ptr = hline_min_ptr;
    if (pix_size == 1)
      memset(hline_min_ptr, *color, hline_end_ptr-hline_min_ptr);
    else//if (pix_size != 1)
    {
      if (hline_min_ptr < hline_end_ptr)
      {
        memcpy(hline_ptr, color, pix_size);
        hline_ptr += pix_size;
      }//end if (hline_min_ptr < hline_end_ptr)
      size_t sizeToCopy = pix_size;
      while(hline_ptr < hline_end_ptr)
      {
        memcpy(hline_ptr, hline_min_ptr, sizeToCopy);
        hline_ptr += sizeToCopy;
        sizeToCopy = std::min(2*sizeToCopy, static_cast<size_t>(hline_end_ptr-hline_ptr));
      }//end while(hline_ptr < hline_end_ptr)
    }//end if (pix_size != 1)
}
//end ICV_HLINE_X()

static inline void ICV_HLINE(uchar* ptr, int64_t xl, int64_t xr, const void* color, int pix_size)
{
  ICV_HLINE_X(ptr, xl, xr, reinterpret_cast<const uchar*>(color), pix_size);
}

/*
static void
FillConvexPoly2( Mat& img, const Point2l* v, int npts, const void* color, int line_type, int shift )
{
    //[Code Explanation By Fenix]
    //理解edge的维护方式是理解这个算法里的关键
    // Opencv convex polygon的填充采用的是水平行扫描的方式，即从miny（多边形的最小y）坐标开始，按顺序递增y，逐行扫描直到maxy
    //    对于每一个y行，都计算的到该行与多边形的2个交点的x坐标。然后填充介于[x_left,x_right]之间像素即达成结果。由于任意convex polygon，任一水平行至多与2条边相交，所以该方法可以成立。
    //    在扫描的过程中，高效实现的关键是维护2个“edge”。即与当前的y行相交的2条边的信息。edge中只有非常有限的信息在必要的时候更新计算，因此速度可以非常快。

    //edge所维护的信息内容
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
    //          在edge耗尽（y>=ye）的时候，其中一条edge朝着 idx = idx +1 的方向更新，另一条edge朝着 idx = idx-1（也就是 (idx + npts-1)%npts）的方向更新到下一个目标顶点。
    //

    //XY_SHIFT的含义：
    //      显然dx（y递增1时，x的增量）是浮点数，但是Opencv在这里可能为了计算更高效，将x用定点数取代，
    //      定点的bit位置是 XY_SHIFT = 65536（16bit），即x的低16位存储的是小数部分，高48位（OpenCV设定x是int64，根据实际情况应当可以设成int32）是整数部分
    //      delta1 = 32768 = XY_SHIFT/2  起到的是最终在取回整数的时候的“round”作用，即：
    //             取整时，使用(x + 32768) >> 16 即 (x + 32768)/65536 等同与浮点形式的  (int)(x+0.5) ，即等同于取 std::round(x)

    //
    //为了实现Obstacle Grid对FillConvexPoly做了哪些改造
    //      1. FillConvexPoly 计算完每行的[x_left,x_right]后，直接使用memset填充固定的像素值，这里直接替换成逐grid计算distance
    //      2. FillConvexPoly 除了填充多边形内部外，还有填充多变形的边缘的算法（同时支持多种线型包括AntiAlis），我们不需要，予以删除，在边缘直接用填充算法取代
    //      3. shift功能不需要直接删掉
    //      4. 根据实际数据范围，将int64改为int32


    struct
    {
        int idx, di;
        int64 x, dx;
        int ye;

        std::string getInfo()
        {
            char buf[512];
            sprintf(buf, "idx = %d di = %d x = %ld(%ld) dx = %ld ye = %d", idx, di,  x,x>>XY_SHIFT, dx, ye);
            return buf;
        }
    }
    edge[2];


    //imin = index of miny
    int i, y, imin = 0;

    int edges = npts;
    
    //xmin ymin, xmax, ymax   are min&&max coordinate of all vectors
    int64 xmin, xmax, ymin, ymax;
    uchar* ptr = img.ptr();
    Size size = img.size();
    int pix_size = (int)img.elemSize();
    

    //delta1 == delta2 == 32768
    int delta1, delta2;

    if( line_type < CV_AA )
        
        delta1 = delta2 = XY_ONE >> 1;
    else
        delta1 = XY_ONE - 1, delta2 = 0;


    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for( i = 0; i < npts; i++ )
    {
        Point2l p = v[i];
        if( p.y < ymin )
        {
            ymin = p.y;
            imin = i;
        }
        ymax = std::max( ymax, p.y );
        xmax = std::max( xmax, p.x );
        xmin = MIN( xmin, p.x );
    }

    if( npts < 3 || (int)xmax < 0 || (int)ymax < 0 || (int)xmin >= size.width || (int)ymin >= size.height )
        return;

    ymax = MIN( ymax, size.height - 1 );

    //start from the min y
    edge[0].idx = edge[1].idx = imin;
    edge[0].ye = edge[1].ye = y = (int)ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    edge[0].x = edge[1].x = -XY_ONE;
    edge[0].dx = edge[1].dx = 0;

    printf("[before iter]edge[0] = %s \n", edge[0].getInfo().c_str());
    printf("             edge[1] = %s \n", edge[1].getInfo().c_str());


    ptr += (int64_t)img.step*y;

    int iter_num = 0;
    do
    {

        printf("[iter %03d]edge[0] = %s \n", iter_num, edge[0].getInfo().c_str());
        printf("          edge[1] = %s \n",  edge[1].getInfo().c_str());
        printf("   y = %d\n", y);

        if( line_type < CV_AA || y < (int)ymax || y == (int)ymin )
        {
            for( i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx0 = edge[i].idx, di = edge[i].di;
                    int idx = idx0 + di;
                    if (idx >= npts) idx -= npts;
                    int ty = 0;

                    printf("  [1]edge[0] = %s \n", edge[0].getInfo().c_str());
                    printf("     edge[1] = %s \n",  edge[1].getInfo().c_str());
                    printf("     i=%d idx0=%d idx=%d ty=%d edges = %d\n", i, idx0, idx, ty, edges);

                    for (; edges-- > 0; )
                    {
                        ty = (int)(v[idx].y);
                        if (ty > y)
                        {
                            int64 xs = v[idx0].x;
                            int64 xe = v[idx].x;
                            
                            xs <<= XY_SHIFT;
                            xe <<= XY_SHIFT;
                        

                            edge[i].ye = ty;
                            edge[i].dx = ((xe - xs)*2 + ((int64_t)ty - y)) / (2 * ((int64_t)ty - y));
                            edge[i].x = xs;
                            edge[i].idx = idx;

                            printf("  [2]edge[0] = %s \n", edge[0].getInfo().c_str());
                            printf("     edge[1] = %s \n",  edge[1].getInfo().c_str());
                            printf("     i=%d idx0=%d idx=%d ty=%d edges = %d\n", i, idx0, idx, ty, edges);

                            break;
                        }
                        else if(ty == y && y != ymin) 
                        {
                            int xx1, xx2;
                            if(v[idx0].x > v[idx].x)
                            {
                                xx1 = v[idx].x;
                                xx2 = v[idx0].x;
                            }
                            else
                            {
                                xx1 = v[idx0].x;
                                xx2 = v[idx].x;
                            }
                          
                            ICV_HLINE( ptr, xx1, xx2, color, pix_size );

                            printf("hline y = %d xl = %d xr = %d \n", y, xx1, xx2);
                        }
                        idx0 = idx;
                        idx += di;
                        if (idx >= npts) idx -= npts;

                        printf("  [3]edge[0] = %s \n", edge[0].getInfo().c_str());
                        printf("     edge[1] = %s \n",  edge[1].getInfo().c_str());
                        printf("     i=%d idx0=%d idx=%d ty=%d edges = %d\n", i, idx0, idx, ty, edges);

                    }
                }
            }
        }

        if (edges < 0)
            break;

        if (y >= 0)
        {
            int left = 0, right = 1;
            if (edge[0].x > edge[1].x)
            {
                left = 1, right = 0;
            }

            int xx1 = (int)((edge[left].x + delta1) >> XY_SHIFT);
            int xx2 = (int)((edge[right].x + delta2) >> XY_SHIFT);

            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 )
                    xx1 = 0;
                if( xx2 >= size.width )
                    xx2 = size.width - 1;
                ICV_HLINE( ptr, xx1, xx2, color, pix_size );

                printf("hline y = %d xl = %d xr = %d \n", y, xx1, xx2);
            }
        }
        else
        {
            // TODO optimize scan for negative y
        }

        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
        ptr += img.step;

        iter_num ++;
    }
    while( ++y <= (int)ymax );
}
*/

int DEBUG_X = 1053;
int DEBUG_Y = 4;

std::vector<cv::Point2i> fillConvextPolyGT( cv::Mat& img, cv::Point2f* vf, int npts, const uchar* color)
{   

    std::vector<cv::Point2i> filled_points;
    std::vector<Eigen::Vector2d> polygon;

    for(int i=0; i<npts; i++){
        polygon.push_back(Eigen::Vector2d(vf[i].x, vf[i].y));
    }

    for(int y=0; y<img.rows; y++){
        for(int x=0; x<img.rows; x++){
            Eigen::Vector2d p(x,y);
            if(Polygon::pointInPolygon(p, polygon)){
                img.at<uchar>(y,x) = color[0];
                filled_points.push_back(cv::Point2i(x,y));
            }
            if(x == DEBUG_X && y == DEBUG_Y){
                SLOG_WARN("x = %d y = %d pointsInPolygonGT = %d", x,y,Polygon::pointInPolygon(p, polygon));
            }            
        } 
    } 



    return filled_points;
}



std::vector<cv::Point2i> 
fillConvexPolyFloat( cv::Mat& img, cv::Point2f* vf, int npts, const uchar* color)
{
    std::vector<cv::Point2i> filled_points;

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
        return filled_points;

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

    printf("xmin = %f ymin = %f xmax = %f ymax = %f ymin_ceil = %d ymax_floor = %d\n",
        xmin, ymin, xmax, ymax, ymin_ceil, ymax_floor);

    for(y=ymin_ceil; y<=ymax_floor; y++){
        float *ptr = (float*)img.ptr(y);

        printf("[y= %03d]edge[0] = %s \n", y, edge[0].getInfo().c_str());
        printf("          edge[1] = %s \n",  edge[1].getInfo().c_str());

        for( i = 0; i < 2; i++ ){
            if( y > edge[i].ye_floor || y == ymin_ceil) {  //ready to find next available edge
                int idx0 = edge[i].idx, di = edge[i].di;
                int idx = idx0 + di;
                if (idx >= npts) idx -= npts;
                float ty;
                int ty_floor;

                printf("[1][y= %03d]edge[0] = %s \n", y, edge[0].getInfo().c_str());
                printf("          edge[1] = %s \n",  edge[1].getInfo().c_str());
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

                        printf("[2][y= %03d]edge[0] = %s \n", y, edge[0].getInfo().c_str());
                        printf("          edge[1] = %s \n",  edge[1].getInfo().c_str()); 
                        printf("     i=%d idx0=%d idx=%d ty=%f edges = %d\n", i, idx0, idx, ty, edges);
                        printf("xs xe ys ye = %f %f %f %f \n", xs ,xe ,ys ,ye);


                        break;
                    } else if(ty_floor == y && y != ymin) { //[Key Modification 2] extra fill for horizontal edgs
                        int xx1, xx2;
                        if(vf[idx0].x > vf[idx].x) {
                            xx1 = std::ceil(vf[idx].x);
                            xx2 = std::floor(vf[idx0].x);
                        } else {
                            xx1 = std::ceil(vf[idx0].x);
                            xx2 = std::floor(vf[idx].x);
                        }

                        printf("[3][y= %03d]edge[0] = %s \n", y, edge[0].getInfo().c_str());
                        printf("          edge[1] = %s \n",  edge[1].getInfo().c_str()); 
                        printf("     i=%d idx0=%d idx=%d ty=%f edges = %d\n", i, idx0, idx, ty, edges);
                        

                        //----  FILL row y grids between [xx1, xx2]
                        printf("FILL1 y=%d, x=[%d %d] \n", y, xx1, xx2);
                        if( xx2 >= 0 && xx1 < size.width && y>=0 && y <= size.height-1) {
                            if( xx1 < 0 ) xx1 = 0;
                            if( xx2 >= size.width ) xx2 = size.width - 1;
                            
                            for(int x=xx1; x<=xx2; x++) {
                                filled_points.push_back(cv::Point2i(x,y));
                                img.at<uchar>(y,x) = color[0];
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
        //     break;

        if (y >= 0) {
            int left = 0, right = 1;
            if (edge[0].x > edge[1].x)
            {
                left = 1, right = 0;
            }
            // -1 && +1 are for edge compensation (might have better ways)
            int xx1 = std::ceil(edge[left].x);
            int xx2 = std::floor(edge[right].x);
            
            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 ) xx1 = 0;
                if( xx2 >= size.width ) xx2 = size.width - 1;
                printf("FILL2 y=%d, x=[%d %d] \n", y, xx1, xx2);

                for(int x=xx1; x<=xx2; x++) {
                        filled_points.push_back(cv::Point2i(x,y));
                        img.at<uchar>(y,x) = color[0];
                    }
            }
        }

        printf("[4][y= %03d]edge[0] = %s \n", y, edge[0].getInfo().c_str());
        printf("          edge[1] = %s \n",  edge[1].getInfo().c_str());

        
        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
    }

    return filled_points;
}

vector<Point2f> readPoints(std::string file_name){
    vector<Point2f> points;
    ifstream ifs(file_name);
    

    if(!ifs.is_open()){
        return points;
    }
    int num;
    ifs >> num;
    points.resize(num);
    for(int i=0; i<num; i++){
        ifs >> points[i].x >> points[i].y;
    }

    ifs.close();
    return points;
}

 
int main( int argc, char** argv)
{

    constexpr bool enable_timer = true;

    int rows = 1200;
    int cols = 1200;

    Mat img(rows,cols, CV_8U);
    img.setTo(0);

    Mat img_gt(rows,cols, CV_8U);
    img_gt.setTo(0);


    // Create a vector of points.
    
    vector<Point2f> points;
    // //points.resize(5);
    // points.push_back(cv::Point2f(20,20));
    // points.push_back(cv::Point2f(10,50));
    // points.push_back(cv::Point2f(30,70));
    // points.push_back(cv::Point2f(90.1,70));
    // points.push_back(cv::Point2f(40,20));


    points.push_back(cv::Point2f(608.897156, 75.531372));
    points.push_back(cv::Point2f(608.122253, 81.689568));
    points.push_back(cv::Point2f(607.898071, 82.625984));
    points.push_back(cv::Point2f(560.573975, 75.129112));
    points.push_back(cv::Point2f(560.306580, 73.079163));
    points.push_back(cv::Point2f(561.325378, 67.713684));
    points.push_back(cv::Point2f(561.596130, 66.974136));


    //points = readPoints("test_obstacle_grid_polygon.txt");
    
    cv::Point2f center = cv::Point2f(0,0);
    for (size_t j = 0; j < points.size(); j++) {
        SLOG_INFO("        vertex[%zd] = %f %f ", j, points[j].x, points[j].y);
        center += points[j];
    }
    center *= (1.0 / points.size());

    PolygonVisualizer pvis;
    PolygonVisualizer pvis_gt;
    
    double origin_x = std::round(center.x);
    double origin_y = std::round(center.y);
    // origin_x = 1048;
    // origin_y = 33;

    pvis.init(6400,6400,0.01, origin_x, origin_y);
    pvis_gt.init(6400,6400,0.01, origin_x, origin_y);

    

    pvis.drawGrid(cv::Scalar(128,128,128), 1.0);
    pvis_gt.drawGrid(cv::Scalar(128,128,128));

    pvis.drawPolygon(cv::Scalar(0,255,0), points);
    pvis_gt.drawPolygon(cv::Scalar(0,255,0), points);



    uchar color_buf[4];
    color_buf[0] = 255;
    color_buf[1] = 255;
    color_buf[2] = 255;
    std::vector<cv::Point2i> filled_points = fillConvexPolyFloat(img, points.data(), (int)points.size(), color_buf);
    

    std::vector<cv::Point2i> filled_points_gt = fillConvextPolyGT(img_gt, points.data(), (int)points.size(), color_buf);
    

    pvis.drawDotsInsidePolygon(cv::Scalar(0,0,255), filled_points);
    pvis_gt.drawDotsInsidePolygon(cv::Scalar(0,0 ,255), filled_points_gt);
    
    cv::imwrite("polygon_vis.png", pvis.getCanvas());
    cv::imwrite("polygon_vis_gt.png", pvis_gt.getCanvas());


    
    //test identical
    cv::Mat img_diff(rows, cols, CV_8UC3);
    for(int y=0; y<rows; y++){
        for(int x=0; x<cols; x++){
            int v = img.at<uchar>(y,x);
            int v_gt = img_gt.at<uchar>(y,x);

            if(v == 0 && v_gt != 0){
                img_diff.at<Vec3b>(y,x) = cv::Vec3b(0,0,255); 
            } else if(v != 0 && v_gt == 0){
                img_diff.at<Vec3b>(y,x) = cv::Vec3b(0,255,255); 
            } else {
                img_diff.at<Vec3b>(y,x) = cv::Vec3b(0,0,0);
            }
           
        }
    }




    cv::imshow("polygon", img);
    cv::imwrite("polygon.png", img);
    cv::imshow("polygon_gt", img_gt);
    cv::imshow("diff", img_diff); 

    // Show results.
    
    waitKey(0);
 
    return 0;
}