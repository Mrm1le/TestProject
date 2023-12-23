# -*- coding: utf-8 -*-

import os
import sys
import json
from easydict import EasyDict

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../lib'))
import py_parking_plotter as apa_plan

gt_dir = "/home/ros/Downloads/gts"

gt_file = "PLAA60351_recording_RVIZ_parking_61_20220316-225730_20220317-000206_11_g.json"

PLANNING_GT_SET = "planning_gt"
PLANNING_DT_SET = "planning_dt"
CAR_PARAM_EP = "/home/ros/catkin_ws/src/maf_planning/test/bag_recurrent/resources/vehicle_param_l.yaml"
APA_PARALLEL = "/home/ros/catkin_ws/src/maf_planning/test/bag_recurrent/resources/apa_parallel.yaml"  # apa file for parallel scenario
APA_VERTICAL = "/home/ros/catkin_ws/src/maf_planning/test/bag_recurrent/resources/apa_vertical.yaml"  # apa file for vertical scenario

def caculate_min_obstacle_dist(target_point, odo_data):
    point = apa_plan.Point2d()
    point.x = target_point.x
    point.y = target_point.y
    point.theta = target_point.theta
    dist = apa_plan.getEgoMinObsDistance(point, json.dumps(odo_data))
    return round(dist, 2)

def update_odo_data(point, odo_data):
    odo_data.target_state.path_point.x = point.x
    odo_data.target_state.path_point.y = point.y
    odo_data.target_state.path_point.theta = point.theta

def get_once_apa_plan_result(slot_type, odo_data):
    if slot_type == "ParkingSlotType.PARALLEL":
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)
    else:
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_VERTICAL)
    odo_str = json.dumps(odo_data)
    res_str = apa_plan.planInterfaceSerialize(odo_str, 0)
    return EasyDict(json.loads(res_str))

def calculate_apa_planning_result(gt):
    dt = {}
    dt['plan_results'] = []
    dt['min_obs_distances'] = []
    dt['name'] = gt['name']
    dt['md5'] = gt['md5']
    dt['dataset'] = PLANNING_DT_SET
    odo_data = EasyDict(gt['odo'])

    is_first = True
    init_speed = -1
    for init_point in gt['init_points']:
        if is_first:
            odo_data.target_state.v = 0
            is_first = False
        else:
            odo_data.target_state.v = init_speed
        init_speed = -init_speed

        update_odo_data(EasyDict(init_point), odo_data)
        apa_plan_result = get_once_apa_plan_result(gt['slot_type'], odo_data)
        dt['plan_results'].append(apa_plan_result)
        min_dist = caculate_min_obstacle_dist(
            EasyDict(gt['target_point']), odo_data)
        dt['min_obs_distances'].append(min_dist)
    return dt

for file_path in os.listdir(gt_dir):
    #if file_path != gt_file:
    #    continue
    file_path = os.path.join(gt_dir, file_path)
    
    gt = dict()
    with open(file_path, 'r') as f:
        gt = json.load(f)

    dt = calculate_apa_planning_result(gt)
    print(dt)

