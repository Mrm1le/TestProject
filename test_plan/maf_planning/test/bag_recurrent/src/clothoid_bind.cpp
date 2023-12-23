#include "clothoid_bind.h"

namespace clothoid{
using namespace msquare;

ClothoidPoint getPointByRadius(ClothoidSolver& clo_sol, double radius){
    ClothoidPoint clo_p;
    clo_sol.getPointByRadius(radius, clo_p);
    return clo_p;
}
ClothoidPoint getPointByS(ClothoidSolver& clo_sol, double s){
    ClothoidPoint clo_p;
    clo_sol.getPointByS(s, clo_p);
    return clo_p;
}
ClothoidCurve getCurveByS(ClothoidSolver& clo_sol, double s, double step){
    ClothoidCurve clo_curve;
    clo_sol.getCurveByS(s,step, clo_curve);
    return clo_curve;
}
std::vector<double> getMuOffset(ClothoidSolver& clo_sol, double radius){
    std::vector<double> res(3);
    clo_sol.getMuOffset(radius, res[0], res[1], res[2]);
    return res;
}

ClothoidCurve transform2DPy(const ClothoidCurve& c1, double x, double y, double theta){
    ClothoidCurve c2;
    transform2D(c1, c2, x, y, theta);
    return c2;
}
ClothoidCurve transformS2Py(const ClothoidCurve& c1){
    ClothoidCurve c2;
    transformS2(c1, c2);
    return c2;
}
ClothoidCurve transformS3Py(const ClothoidCurve& c1){
    ClothoidCurve c2;
    transformS3(c1, c2);
    return c2;
}
ClothoidCurve transformS4Py(const ClothoidCurve& c1){
    ClothoidCurve c2;
    transformS4(c1, c2);
    return c2;
}

std::vector<parking_scenario::Point2d> CollisionShapeGeneratorPy::getRawShape(const parking_scenario::Point2d& ego_pose, double lat, double lon){
        Pose2D pose(ego_pose.x, ego_pose.y, ego_pose.theta);
        std::vector<planning_math::Vec2d> corners;
        std::vector<parking_scenario::Point2d> res;

        csg_.getRawShape(pose, corners, lat, lon);
        for(auto p : corners){
            res.emplace_back(p.x(), p.y(), 0.0);
        }
        return res;

    }
    std::vector<parking_scenario::Point2d> CollisionShapeGeneratorPy::getWheelBaseShape(const parking_scenario::Point2d& ego_pose, double lat, double lon){
        Pose2D pose(ego_pose.x, ego_pose.y, ego_pose.theta);
        std::vector<planning_math::Vec2d> corners;
        std::vector<parking_scenario::Point2d> res;

        csg_.getWheelBaseShape(pose, corners, lat, lon);
        for(auto p : corners){
            res.emplace_back(p.x(), p.y(), 0.0);
        }
        return res;
    }
    std::vector<parking_scenario::Point2d> CollisionShapeGeneratorPy::getRotateShape(const parking_scenario::Point2d& ego_pose, double lat, double lon){
        Pose2D pose(ego_pose.x, ego_pose.y, ego_pose.theta);
        std::vector<planning_math::Vec2d> corners;
        std::vector<parking_scenario::Point2d> res;

        csg_.getRotateShape(pose, corners, lat, lon);
        for(auto p : corners){
            res.emplace_back(p.x(), p.y(), 0.0);
        }
        return res;
    }

    std::vector<parking_scenario::Point2d> CollisionShapeGeneratorPy::getRectShape(const parking_scenario::Point2d& ego_pose, double lat, double lon){
        Pose2D pose(ego_pose.x, ego_pose.y, ego_pose.theta);
        std::vector<planning_math::Vec2d> corners;
        std::vector<parking_scenario::Point2d> res;

        csg_.getRectShape(pose, corners, lat, lon);
        for(auto p : corners){
            res.emplace_back(p.x(), p.y(), 0.0);
        }
        return res;
    }
    std::vector<parking_scenario::Point2d> CollisionShapeGeneratorPy::getOctagonShape(const parking_scenario::Point2d& ego_pose, double lat, double lon){
        Pose2D pose(ego_pose.x, ego_pose.y, ego_pose.theta);
        std::vector<planning_math::Vec2d> corners;
        std::vector<parking_scenario::Point2d> res;

        csg_.getOctagonShape(pose, corners, lat, lon);
        for(auto p : corners){
            res.emplace_back(p.x(), p.y(), 0.0);
        }
        return res;
    }
    

}