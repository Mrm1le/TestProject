# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod
# import rosbag
import time
import math
import numpy as np
import json
import sys
import os
import argparse

from evaluation.cp_evaluator import CPEvaluator, PlanPathOutput, PathPlanInput,EnvInfo
from cp_visualization import CPVisualizer,visualize,build_result_table
from plotter.load_json_cp import JsonRefline,JsonQuadGenerator,JsonPlanPathGenerator
from bokeh.layouts import column, row


import matplotlib
import matplotlib.pyplot as plt
import numpy as np

html_file = "/home/ros/Downloads/visu/simu_visu_bag/cp_evaluation.html"


class CPEvent(object):
    def __init__(self, event_name, dt_jsons, gt_jsons, result_table, input_type):
        self.event_name = event_name
        # print("input_type = ", input_type)
        # Formulate Input, Output, Env
        path_plan_input  = PathPlanInput(gt_jsons, "path_plan_input", event_name)
        env_info         = EnvInfo(gt_jsons, "env_info", event_name)
        plan_path_output = PlanPathOutput(dt_jsons,"plan_path_output", event_name)

        self.path_plan_input = path_plan_input
        self.env_info = env_info
        self.plan_path_output = plan_path_output

        # Build Evaluator and Visualizer
        evaluate_init  = 10        # the frame_number when evaluation initiates
        cp_evaluator   = CPEvaluator(plan_path_output,result_table.keys(), env_info, path_plan_input,evaluate_init)
        cp_visualizer  = CPVisualizer(path_plan_input, plan_path_output, env_info, input_type)
        
        # Get Evaluation and Visualization results 
        self.res_table = cp_evaluator.res_table
        self.layout    = cp_visualizer.event_layout
        

def printHelp():
    print('''\n
USAGE:
    1. <evaluate base version>                 python3 cp_evaluation.py 
    2. <evaluate optimized version>            python3 cp_evaluation.py opt
    3. <evaluate real-input version>           python3 cp_evaluation.py real
    3. <evaluate optimized real-input version> python3 cp_evaluation.py opt_real
    
\n''')


if __name__ == "__main__":
    print(">> Starting to run CP Evaluation...")
    print(">> load json data...")

    gt_dir = '/home/ros/Downloads/Lat_Evaluation_Bags/0509_3.0_bag_gt'
    dt_dir = '/home/ros/Downloads/Lat_Evaluation_Bags/0509_3.0_bag_dt'
    
    if len(sys.argv) >= 2 and sys.argv[-1] not in ["real","opt","opt_real"]:
        print("Error! Invalid input...")
        printHelp()
        sys.exit()

    elif len(sys.argv) >= 2:
        print("   ======",sys.argv[-1],"dt ======")
        planner_debug_dir = dt_dir+'_' + sys.argv[-1] + '/' 
        planner_input_dir = gt_dir+'/' 
        html_file = "/home/ros/Downloads/Lat_Evaluation_Bags/visu/cp_evaluation_" + sys.argv[-1] + ".html"   

    else:
        print("   ====== baseline dt ======")
        planner_debug_dir = dt_dir+'/' 
        planner_input_dir = gt_dir+'/' 
        html_file = "/home/ros/Downloads/Lat_Evaluation_Bags/visu/cp_evaluation.html"
        # planner_input_dir = '/home/ros/Downloads/CP_Evaluation/Ramp_json/All_3.0_replan_bag_gt/others/'
        # planner_debug_dir = '/home/ros/Downloads/CP_Evaluation/Ramp_json/All_3.0_replan_bag_dt/others/'
    
    events       = list()    # here, all events are stored here, to evluate and visualize
    layouts      = list()
    gt_json_list = list()

    if not os.path.exists(planner_input_dir):
        print("Cannot find gt directory!")
        sys.exit()
    if not os.path.exists(planner_debug_dir):
        print("Cannot find dt directory!")
        sys.exit()

    for file_name in os.listdir(planner_input_dir):
        if os.path.splitext(file_name)[1] == '.json':
            gt_json_list.append(file_name)
            print("   find {}".format(file_name))
    
    if gt_json_list == []:
        sys.exit()
    # ----------------------------------------------------------------#
    result_table                           = dict()
    result_table['test case']              = list()
    result_table['summary score']          = list()
    result_table['PLANNING SUCCESS']       = list()
    result_table['min dis2RE (PvE)']       = list()
    result_table['min TTC Lane']           = list()
    result_table['min TTC RE']             = list()
    result_table['MAX CURV RATE']          = list()
    result_table['MAX LAT JERK']           = list()
    result_table['mean re TTC']            = list()
    result_table['more infomation']        = list()

    print(">> Evaluating...")
    for file_name in gt_json_list:
        event_name_f = os.path.splitext(file_name)[0]
        event_name = event_name_f.split('_gt')[0]
        input_type = ""
        gt_jsons = json.load(open(planner_input_dir + event_name + "_gt.json"))
        if len(sys.argv) >= 2 and sys.argv[-1] in ['real','opt_real']:
            input_type = "real"
            dt_jsons = json.load(open(planner_debug_dir + event_name + "_real_dt.json"))
        else:
            dt_jsons = json.load(open(planner_debug_dir + event_name + "_dt.json"))
        # print("   load",event_name,"from", event_name_f)
        print("   load", file_name)
        cp_event = CPEvent(event_name, dt_jsons, gt_jsons, result_table, input_type)  # Main Entrance

        # quad_x_list = []
        # quad_y_list = []        
        # data_jsons = cp_event.path_plan_input.data.origin_data
        # for data_json in data_jsons:
        #     for path_segment in data_json['path_planner_input']['path_segments']:
        #         # print("quad point number: ",len(path_segment['quad_point_info']))
        #         for quad_point_info in path_segment['quad_point_info']:
        #             quad_x_list.append(quad_point_info['refline_info']['x'])
        #             quad_y_list.append(quad_point_info['refline_info']['y'])
        # # real_env
        # re_enu_x_list = []
        # re_enu_y_list = []
        # refline_x_list = []
        # refline_y_list = []

        # for rre_point in cp_event.env_info.data["real_env"]["right_road_edge"]:
        #     re_enu_x_list.append(rre_point["x"])
        #     re_enu_y_list.append(rre_point["y"])
        # for refpoint in cp_event.env_info.data["real_env"]["reference_line_points"]:
        #     refline_x_list.append(refpoint['enu_point_x'])
        #     refline_y_list.append(refpoint['enu_point_y'])

        # plt.scatter(re_enu_x_list, re_enu_y_list, color = 'red')
        # plt.scatter(refline_x_list, refline_y_list, color = 'purple', s = 5)
        # plt.scatter(quad_x_list, quad_y_list, color = 'green', s = 5)
        # plt.show()

        events.append(cp_event)
        for k, v in cp_event.res_table.items():
            result_table[k].append(v)
        # break
    
    # for k in cp_event.res_table.keys():
    #     v = "aaa"
    #     print("k = ", k ,   "  v = ", v)
    #     result_table[k].append(v)
    # result_table['test case'].append("111")
    # result_table['summary score'].append("111")
    # result_table['PLANNING SUCCESS'].append("111")
    # result_table['min dis2RE (PvE)'].append("111")
    # result_table['min TTC Lane'].append("111")
    # result_table['min TTC RE'].append("111")
    # result_table['MAX CURV RATE'].append("111")
    # result_table['MAX LAT JERK'].append("111")
    # result_table['more infomation'].append("111")

    # ----------------------------------------------------------------#
    layouts = build_result_table(layouts,result_table)
    print(">> Visualizing...")
    col_num = 2
    layout_temp  = []
    index = 0
    for event in events:
        layout_temp.append(event.layout)
        index+=1
        if (index == len(events))or(index % col_num == 0 and index >0): 
            layouts.append(layout_temp)
            layout_temp = []
            # print("here!")
    print(">> Producing html...")   
    visualize(html_file,layouts)
