# -*- coding: utf-8 -*-

import os
import sys
import json
import copy
from easydict import EasyDict
from evaluation.reporter import one_case_html

GT_PATH = '/home/ros/Downloads/real_example_gt.json'
HTML_PATH = os.path.dirname(GT_PATH)

gt_data = dict()
with open(GT_PATH, 'r') as f:
    gt_data = json.load(f)

odo_list = list()
expert_list = list()
plan_list = list()
title_list = list()

for index in range(len(gt_data["init_points"])):

    #process odo
    cur_odo = EasyDict(gt_data["odo"])
    if index != 0 and \
        "replan_odos" in gt_data and \
        len(gt_data["replan_odos"]) >= index:

        cur_odo = EasyDict(gt_data["replan_odos"][index - 1])
    
    odo_list.append(cur_odo)

    #process expert
    expert_result = dict()
    init_point = gt_data["init_points"][index]
    for i in range(len(gt_data["expert_result"]["x"])):
        if init_point["x"] == gt_data["expert_result"]["x"][i] and \
            init_point["y"] == gt_data["expert_result"]["y"][i] and \
            init_point["theta"] == gt_data["expert_result"]["phi"][i]:
            print(i)

            expert_result["x"] = copy.deepcopy(gt_data["expert_result"]["x"][0:i])
            expert_result["y"] = copy.deepcopy(gt_data["expert_result"]["y"][0:i])
            expert_result["phi"] = copy.deepcopy(gt_data["expert_result"]["phi"][0:i])
            
            break

    if len(expert_result) == 0:
        expert_list.append(EasyDict(gt_data["expert_result"]))
    else:
        expert_list.append(EasyDict(expert_result))
    
    plan_result = EasyDict({"x":[], "y": [], "phi": []})
    plan_list.append(plan_result)
    title_list.append(gt_data["name"])

print(len(odo_list), len(expert_list), len(plan_list))


one_case_html(HTML_PATH, title_list, odo_list, expert_list, plan_list, gt_data["car_size_params"])
