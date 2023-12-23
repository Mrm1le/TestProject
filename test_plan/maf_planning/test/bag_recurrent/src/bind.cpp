#include "util.h"
#include "planner_interface.h"
#include "perfect_scene.h"
#include "sbp_planner_wrapper.h"
#include "parking_scenario/parking_scenario.h"
#include "parking_scenario/pose_adjuster.h"
#include "clothoid_bind.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_solver.h"
#include "speed_planner/speed_planner_interface.h"
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
namespace pc = parking_scenario;


// // python binding module
PYBIND11_MODULE(py_parking_plotter, m) {
    py::class_<BagAnalysis>(m, "BagAnalysis")
        .def(py::init<>())
        .def("getBagResults", &BagAnalysis::getBagResults)
        .def("getBagEnvironmentInfos", &BagAnalysis::getBagEnvironmentInfos)
        .def("loadBagAndAnalyze", &BagAnalysis::loadBagAndAnalyze)
        .def("getPlannerParams", &BagAnalysis::getPlannerParams)
        .def("getCarSizeParams", &BagAnalysis::getCarSizeParams)      
        .def("plan", py::overload_cast<std::string, msquare::parking::SearchProcessDebug* >(&BagAnalysis::plan))
        .def("getBagResDebugs", &BagAnalysis::getBagResDebugs)
        .def("getParamString", &BagAnalysis::getParamString);

    py::class_<msquare::parking::SearchDebugNode>(m, "SearchDebugNode")
        .def(py::init<const double &, const double &, const double &,
                        const double &, const double & >())
        .def_readwrite("x", &msquare::parking::SearchDebugNode::x)
        .def_readwrite("y", &msquare::parking::SearchDebugNode::y)
        .def_readwrite("theta", &msquare::parking::SearchDebugNode::theta)
        .def_readwrite("traj_cost", &msquare::parking::SearchDebugNode::traj_cost)
        .def_readwrite("heuristic_cost", &msquare::parking::SearchDebugNode::heuristic_cost);

    py::class_<msquare::parking::SearchDebugEdge>(m, "SearchDebugEdge")
        .def(py::init<const msquare::parking::SearchDebugNode&,
                        const msquare::parking::SearchDebugNode& >())
        .def_readwrite("start_node", &msquare::parking::SearchDebugEdge::start_node)
        .def_readwrite("end_node", &msquare::parking::SearchDebugEdge::end_node)
        .def_readwrite("edge_cost", &msquare::parking::SearchDebugEdge::edge_cost);

    py::class_<msquare::parking::SearchProcessDebug>(m, "SearchProcessDebug")
        .def(py::init<>())
        .def(py::init<const std::vector<msquare::parking::SearchDebugNode>&, 
                        const std::vector<std::vector<msquare::parking::SearchDebugNode>>&, 
                        const std::vector<std::vector<msquare::parking::SearchDebugEdge>>& >())
        .def_readwrite("searched_node", &msquare::parking::SearchProcessDebug::searched_node)
        .def_readwrite("added_nodes", &msquare::parking::SearchProcessDebug::added_nodes)
        .def_readwrite("added_edges", &msquare::parking::SearchProcessDebug::added_edges);

    // scenario perf frontier
    py::class_<pc::Point2d>(m, "Point2d")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("x", &pc::Point2d::x)
        .def_readwrite("y", &pc::Point2d::y)
        .def_readwrite("theta", &pc::Point2d::theta);
    
    py::class_<pc::PlanResult>(m, "PlanResult")
        .def(py::init<>())
        .def_readwrite("point", &pc::PlanResult::point)
        .def_readwrite("is_success", &pc::PlanResult::is_success)
        .def_readwrite("debug_info", &pc::PlanResult::debug_info);
    py::class_<pc::ScenarioResult>(m, "ScenarioResult")
        .def(py::init<>())
        .def_readwrite("br", &pc::ScenarioResult::br)
        .def_readwrite("debug_info", &pc::ScenarioResult::debug_info);

    py::enum_<pc::ScenarioType>(m, "ScenarioType")
        .value("PARALLEL", pc::ScenarioType::PARALLEL)
        .value("VERTICAL_BASE", pc::ScenarioType::VERTICAL_BASE)
        .value("VERTICAL_SINGLE_CAR", pc::ScenarioType::VERTICAL_SINGLE_CAR)
        .value("OBLIQUE_SLOT", pc::ScenarioType::OBLIQUE_SLOT)
        .value("PARALLEL_OUT", pc::ScenarioType::PARALLEL_OUT)
        .value("VERTICAL_BASE_OUT", pc::ScenarioType::VERTICAL_BASE_OUT)
        .value("VERTICAL_SINGLE_CAR_OUT",
               pc::ScenarioType::VERTICAL_SINGLE_CAR_OUT)
        .value("OBLIQUE_OUT", pc::ScenarioType::OBLIQUE_OUT)
        .export_values();

    py::class_<pc::PlanParams>(m, "PlanParams")
        .def(py::init<>())
        .def_readonly("BOUNDARY_WIDTH", &pc::PlanParams::BOUNDARY_WIDTH)
        .def_readonly("BOUNDARY_MARGIN", &pc::PlanParams::BOUNDARY_MARGIN)
        .def_readonly("SLOT_DEPTH_MARGIN", &pc::PlanParams::SLOT_DEPTH_MARGIN)
        .def_readwrite("apa_file", &pc::PlanParams::apa_file)
        .def_readwrite("step", &pc::PlanParams::step)
        .def_readwrite("channel_width", &pc::PlanParams::channel_width)
        .def_readwrite("vehicle_param_file", &pc::PlanParams::vehicle_param_file)
        .def_readwrite("slot_margin", &pc::PlanParams::slot_margin)
        .def_readwrite("scenario_type", &pc::PlanParams::scenario_type)
        .def_readwrite("left_space", &pc::PlanParams::left_space)
        .def_readwrite("right_space", &pc::PlanParams::right_space)
        .def_readwrite("top_space", &pc::PlanParams::top_space)
        .def_readwrite("bottom_space", &pc::PlanParams::bottom_space)
        .def_readwrite("oblique_angle", &pc::PlanParams::oblique_angle)
        .def_readwrite("theta", &pc::PlanParams::theta)
        .def_readwrite("left_blocked", &pc::PlanParams::left_blocked)
        .def_readwrite("right_blocked", &pc::PlanParams::right_blocked)
        .def_readwrite("opposite_width", &pc::PlanParams::opposite_width);

    m.def("scenarioPlan", &pc::scenarioPlan);

    py::class_<pc::PoseAdjuster>(m, "PoseAdjuster")
        .def(py::init<const pc::PlanParams&>())
        // .def("planOnce", &pc::PoseAdjuster::planOnce, "get plan",py::arg("point"), py::arg("sp_debug") = nullptr)
        .def("planOnce", &pc::PoseAdjuster::planOnce, "get plan")
        .def("getOdo", &pc::PoseAdjuster::getOdo)
        .def("getMinX", &pc::PoseAdjuster::getMinX)
        .def("getMinY", &pc::PoseAdjuster::getMinY)
        .def("getMaxX", &pc::PoseAdjuster::getMaxX)
        .def("getMaxY", &pc::PoseAdjuster::getMaxY);
    
    m.def("initSingletonParams", &planner_interface::initSingletonParams);
    m.def("planInterfaceSerialize", &planner_interface::planInterfaceSerialize);
    m.def("planInterfaceSerializeParams", &planner_interface::planInterfaceSerializeParams);
    m.def("planInterface", &planner_interface::planInterface);
    m.def("getCarSizeParams", &planner_interface::getCarSizeParams);
    m.def("getAstarPlannerParams", &planner_interface::getAstarPlannerParams);


    m.def("getEgoCorners", &planner_interface::getEgoCorners);
    m.def("getEgoMinObsDistance", &planner_interface::getEgoMinObsDistance);


    py::enum_<perfect_scene::ObstacleType>(m, "ObstacleType")
        .value("CAR", perfect_scene::ObstacleType::CAR)
        .value("WALL", perfect_scene::ObstacleType::WALL)
        .export_values();

    py::enum_<perfect_scene::ParkingSlotType>(m, "ParkingSlotType")
        .value("PARALLEL", perfect_scene::ParkingSlotType::PARALLEL)
        .value("VERTICAL", perfect_scene::ParkingSlotType::VERTICAL)
        .value("OBLIQUE", perfect_scene::ParkingSlotType::OBLIQUE)
        .value("PARKOUT_PARALLEL",
               perfect_scene::ParkingSlotType::PARKOUT_PARALLEL)
        .value("PARKOUT_VERTICAL",
               perfect_scene::ParkingSlotType::PARKOUT_VERTICAL)
        .value("PARKOUT_OBLIQUE",
               perfect_scene::ParkingSlotType::PARKOUT_OBLIQUE)
        .value("UNKNOWN", perfect_scene::ParkingSlotType::UNKNOWN)
        .export_values();

    py::enum_<perfect_scene::ParkingSlotSide>(m, "ParkingSlotSide")
        .value("SIDE_LEFT", perfect_scene::ParkingSlotSide::SIDE_LEFT)
        .value("SIDE_RIGHT", perfect_scene::ParkingSlotSide::SIDE_RIGHT)
        .value("SIDE_UNKNOWN", perfect_scene::ParkingSlotSide::SIDE_UNKNOWN)
        .export_values();
    
    py::class_<perfect_scene::LabelPoint>(m, "LabelPoint")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("x", &perfect_scene::LabelPoint::x)
        .def_readwrite("y", &perfect_scene::LabelPoint::y);
    
    py::class_<perfect_scene::ObstacleLabelData>(m, "ObstacleLabelData")
        .def(py::init<perfect_scene::ObstacleType, std::vector<perfect_scene::LabelPoint> >())
        .def_readonly("label_points", &perfect_scene::ObstacleLabelData::label_points)
        .def_readonly("obstacle_type", &perfect_scene::ObstacleLabelData::obstacle_type);
    m.def("perfectScenePlan", &perfect_scene::perfectScenePlan);
    m.def("perfectScenePlanSimple", &perfect_scene::perfectScenePlanSimple);
    m.def("getSlotType", &perfect_scene::getSlotType);
    m.def("getSlotTypeFromSlotCorners", &perfect_scene::getSlotTypeFromSlotCorners);
    m.def("convertLidarPose2EgoPose", &perfect_scene::convertLidarPose2EgoPose);

    py::class_<clothoid::ClothoidPoint>(m, "ClothoidPoint")
        .def(py::init<>())
        .def_readwrite("x", &clothoid::ClothoidPoint::x)
        .def_readwrite("y", &clothoid::ClothoidPoint::y)
        .def_readwrite("theta", &clothoid::ClothoidPoint::theta)
        .def_readwrite("k", &clothoid::ClothoidPoint::k)
        .def_readwrite("s", &clothoid::ClothoidPoint::s);
    
    py::class_<clothoid::ClothoidSolver>(m, "ClothoidSolver")
        .def(py::init<double>())
        .def("getAlpha", &clothoid::ClothoidSolver::getAlpha)
        .def("getPointByS", &clothoid::ClothoidSolver::getPointByS)
        .def("getCurveByS2", &clothoid::ClothoidSolver::getCurveByS2);
    
    py::class_<clothoid::CollisionShapeGeneratorPy>(m, "CollisionShapeGenerator")
        .def(py::init<>())
        .def("getRawShape", &clothoid::CollisionShapeGeneratorPy::getRawShape)
        .def("getWheelBaseShape", &clothoid::CollisionShapeGeneratorPy::getWheelBaseShape)
        .def("getRectShape", &clothoid::CollisionShapeGeneratorPy::getRectShape)
        .def("getRotateShape", &clothoid::CollisionShapeGeneratorPy::getRotateShape)
        .def("getOctagonShape", &clothoid::CollisionShapeGeneratorPy::getOctagonShape);
    
    m.def("getPointByS", &clothoid::getPointByS);
    m.def("getCurveByS", &clothoid::getCurveByS);
    m.def("getMuOffset", &clothoid::getMuOffset);
    m.def("getPointByRadius", &clothoid::getPointByRadius);
    m.def("transform2DPy", &clothoid::transform2DPy);
    m.def("transformS2Py", &clothoid::transformS2Py);
    m.def("transformS3Py", &clothoid::transformS3Py);
    m.def("transformS4Py", &clothoid::transformS4Py);

    m.def("marginVelocityLimit", &speed_planner_interface::marginVelocityLimit);
    // m.def("marginVelocityLimitDynamic", &speed_planner_interface::marginVelocityLimitDynamic);
    // m.def("getSVByT", &speed_planner_interface::getSVByT);
    // m.def("solveSpeed", &speed_planner_interface::solveSpeed);
    // m.def("solveSpeedByConfig", &speed_planner_interface::solveSpeedByConfig);

    py::class_<speed_planner_interface::MarginDebugWithDebugPara>(m, "MarginDebugWithDebugPara")
        .def(py::init<>())
        .def_readwrite("debug_vec_sv",&speed_planner_interface::MarginDebugWithDebugPara::debug_vec_sv)
        .def_readwrite("vec_sv", &speed_planner_interface::MarginDebugWithDebugPara::vec_sv)
        .def_readwrite("polygons", &speed_planner_interface::MarginDebugWithDebugPara::polygons);

    m.def("getSpeedMarginByParas", &speed_planner_interface::getSpeedMarginByParas);
    m.def("getSVByT", &speed_planner_interface::getSVByT);
    m.def("solveSpeed", &speed_planner_interface::solveSpeed);
    m.def("solveSpeedByConfig", &speed_planner_interface::solveSpeedByConfig);

    
}