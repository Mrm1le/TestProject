#ifndef CLOTHOID_BIND_HEADER
#define CLOTHOID_BIND_HEADER

#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_solver.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/curve.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_shape.h"

#include "parking_scenario/parking_scenario.h"

namespace clothoid{

ClothoidPoint getPointByS(ClothoidSolver& clo_sol, double s);
ClothoidPoint getPointByRadius(ClothoidSolver& clo_sol, double radius);

ClothoidCurve getCurveByS(ClothoidSolver& clo_sol, double s, double step);
std::vector<double> getMuOffset(ClothoidSolver& clo_sol, double radius);

ClothoidCurve transform2DPy(const ClothoidCurve& c1, double x, double y, double theta);
ClothoidCurve transformS2Py(const ClothoidCurve& c1);
ClothoidCurve transformS3Py(const ClothoidCurve& c1);
ClothoidCurve transformS4Py(const ClothoidCurve& c1);


class CollisionShapeGeneratorPy{
public:
    CollisionShapeGeneratorPy(){
    }

    std::vector<parking_scenario::Point2d> getRawShape(const parking_scenario::Point2d& ego_pose, double lat, double lon); 
    std::vector<parking_scenario::Point2d> getWheelBaseShape(const parking_scenario::Point2d& ego_pose, double lat, double lon);
    std::vector<parking_scenario::Point2d> getRotateShape(const parking_scenario::Point2d& ego_pose, double lat, double lon);
    std::vector<parking_scenario::Point2d> getRectShape(const parking_scenario::Point2d& ego_pose, double lat, double lon);
    std::vector<parking_scenario::Point2d> getOctagonShape(const parking_scenario::Point2d& ego_pose, double lat, double lon);

private:
    CollisionShapeGenerator csg_;
};

}


#endif
