# -*- coding: utf-8 -*-
import os
import sys
import json
from easydict import EasyDict
import math
import copy

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as pp
from plotter.utils.common import GEO_TYPE

# +
CAR_PARAM_RX5 = "../resources/vehicle_param_rx5.yaml"
CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
APA_VERTICAL_2ND = "../resources/apa_vertical_2nd.yaml"                  # apa file for vertical scenario 2
APA_OBLIQUE = "../resources/apa_oblique.yaml"

HTML_FILE_PATH = "/home/yee/Documents/momenta/20220730_spiral/perf_test_spiral_thetam1.html"
SCENARIO = {
    pp.ScenarioType.PARALLEL: "parallel",
    pp.ScenarioType.VERTICAL_BASE: "vertical_base",
    pp.ScenarioType.VERTICAL_SINGLE_CAR: "vertical_single_car",
    pp.ScenarioType.OBLIQUE_SLOT: "oblique_slot"
}
WIDTH = 700
HEIGHT = 600
USE_MULTI_PROCESS = True         # use multi process
GEO_TYPE[0] = "14"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon
MAX_OBLIQUE_ANGLE = math.pi * 50.0 / 180.0

# +
tuned_params = {
    "car_params":{
#         "rx5":CAR_PARAM_RX5,
        "ep":CAR_PARAM_EP
    },
    "step":0.4,                                                 # 采样间隔
    "theta": 0.0,                                               # 默认采样点theta
    "left_space": 1.0,                                          # 通道上下左右不采样距离
    "right_space": 1.0,
    "top_space": 0.2,
    "bottom_space": 0.2,
    "scenario":[                                                # 评测场景
         {
            "type": pp.ScenarioType.PARALLEL,                   # 平行泊车
            "channel_width": [8],                             # 通道宽度
            "slot_margin":[1.8],#[1.2, 1.4, 1.6, 1.8] ,     # 平行库位长度减去车长后的长度
            "apa_file": APA_PARALLEL
        }
#          {
#             "type": pp.ScenarioType.VERTICAL_BASE,              # 垂直泊
#             "channel_width": [5.4],                             # 通道宽度
#             "slot_margin":[2.5],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
#             "apa_file": APA_VERTICAL
#         }
#         {
#             "type": pp.ScenarioType.OBLIQUE_SLOT,              # 斜列车位
#             "oblique_angle": [30, 45, 60],                 # 斜列角度
#             "channel_width": [5.4],                             # 通道宽度
#             "slot_margin":[2.7],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
#             "apa_file": APA_OBLIQUE
#         }
#          {
#             "type": pp.ScenarioType.VERTICAL_BASE,              # 垂直泊
#             "channel_width": [5.4, 5.3, 5.2, 5.1, 5.0],                             # 通道宽度
#             "slot_margin":[2.5, 2.6],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
#             "apa_file": APA_VERTICAL
#         }
#         {
#             "type": pp.ScenarioType.VERTICAL_SINGLE_CAR,        # 垂直泊车对面车伸出库位
#             "channel_width": [5.0],                             # 对面车所形成的通道宽度， 原始通道宽度为5.5 
#             "slot_margin":[2.6],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
#             "apa_file": APA_VERTICAL_2ND
#         }
    ]
}


def getParams(tuned_params):
    perf_all = {}
    for car in tuned_params['car_params'].keys(): 
        perf_all[car]=[[],[]]
        params = []
        names = []
        for scenario in tuned_params['scenario']:
            for slot_margin in scenario['slot_margin']:
                for channel_width in scenario['channel_width']:
                    name =car.upper() + " " + SCENARIO[scenario['type']] + " slot_margin="+str(slot_margin) + " channel_width="+str(channel_width)

                    plan_param = pp.PlanParams()
                    plan_param.vehicle_param_file = tuned_params['car_params'][car]
                    plan_param.scenario_type = scenario['type']
                    plan_param.step = tuned_params['step']
                    plan_param.slot_margin = slot_margin
                    plan_param.channel_width = channel_width
                    plan_param.apa_file = scenario["apa_file"]
                    
                    plan_param.theta = tuned_params['theta']
                    plan_param.left_space = tuned_params['left_space']
                    plan_param.right_space = tuned_params['right_space']
                    plan_param.top_space = tuned_params['top_space']
                    plan_param.bottom_space = tuned_params['bottom_space']
                    if scenario['type'] != pp.ScenarioType.OBLIQUE_SLOT:
                        params.append(plan_param)
                        names.append(name)
                        continue

#                     oblique_min = tuned_params["oblique_min"]
#                     oblique_max = tuned_params["oblique_max"]
#                     oblique_angle_step = tuned_params["oblique_angle_step"]
#                     if abs(oblique_max - oblique_min) < 1e-2:
#                         plan_param.oblique_angle = oblique_min
#                         params.append(plan_param)
#                         names.append(name + " oblique_angle="+str(oblique_min))
#                     angle_nums = (oblique_max - oblique_min) // oblique_angle_step 
#                     real_angle_step = (oblique_max - oblique_min) / angle_nums
                    oblique_angles = scenario["oblique_angle"]
                    for oblique_angle in oblique_angles:
                        new_plan_params = pp.PlanParams()
                        new_plan_params.vehicle_param_file = tuned_params['car_params'][car]
                        new_plan_params.scenario_type = scenario['type']
                        new_plan_params.step = tuned_params['step']
                        new_plan_params.slot_margin = slot_margin
                        new_plan_params.channel_width = channel_width
                        new_plan_params.apa_file = scenario["apa_file"]

                        new_plan_params.theta = tuned_params['theta']
                        new_plan_params.left_space = tuned_params['left_space']
                        new_plan_params.right_space = tuned_params['right_space']
                        new_plan_params.top_space = tuned_params['top_space']
                        new_plan_params.bottom_space = tuned_params['bottom_space']
                        new_plan_params.oblique_angle = oblique_angle * math.pi / 180.0
                        params.append(new_plan_params)
                        names.append(name + " oblique_angle="+str(oblique_angle)+"deg")

            perf_all[car][0]=params
            perf_all[car][1]=names
        
    
    return perf_all
perf_all = getParams(tuned_params)
# -



# +
# 
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, Title
from bokeh.io import output_notebook, push_notebook,output_file,export_png
from bokeh.layouts import layout, column
from bokeh.plotting import figure, output_file, show
from plotter.composite_layers import PerfPlotLayer
from plotter.event_tools import MeasureTools

def getOneFig(scenario_res, name):
    success = 0
    fail = 0
    for pr in scenario_res.br:
        if pr.is_success:
            success += 1
        else:
            fail += 1
    # set layers
    fig = bkp.figure(title=name,
                     x_axis_label='x',
                     y_axis_label='y',
                     match_aspect=True,
                     width=WIDTH,
                     height=HEIGHT)
    fig.add_layout(Title(text="blue: failed", text_font_size="10pt"), 'above')
    fig.add_layout(Title(text="green: success with gear change <=9", text_font_size="10pt"), 'above')
    fig.add_layout(Title(text="red: success with gear change > 9", text_font_size="10pt"), 'above')
    fig.add_layout(Title(text="all:{}   success:{}   fail:{}".format(success+fail, success, fail), text_font_size="10pt"), 'above')
    MeasureTools(fig)
    fig.title.align = 'center'
#     fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

    perf_layer = PerfPlotLayer(fig, [{},{},{},{'legend_label': 'target_car'}, {}])
    perf_layer.updateUtil(scenario_res)

    fig.legend.click_policy="hide"
    fig.legend.location = "bottom_left"
#     export_png（fig, name+".png"）
    return perf_layer.getLayout()

def planOnce(param):
    print(param.oblique_anle)
    return pp.scenarioPlan(param, USE_MULTI_PROCESS)

def getScenarios(perf_all):
    scenario_all = [[] for i in range(len(perf_all))]
    for index, car in enumerate(perf_all.keys()):
        for j, param in enumerate(perf_all[car][0]):
            print("process {}-{} ...".format(index, j))
            scenario_res = pp.scenarioPlan(param, USE_MULTI_PROCESS)
            scenario_all[index].append(scenario_res)
    
    return scenario_all

def getFigs(scenario_all, perf_all):
    fig_all = [[] for i in range(len(perf_all))]
    for index, car in enumerate(perf_all.keys()):
        for j, param in enumerate(perf_all[car][0]):
            fig = getOneFig(scenario_all[index][j], perf_all[car][1][j])
            fig_all[index].append(fig)
    
    return fig_all
    


# +
# plan
import time
start_time = time.process_time()
start_clock = time.perf_counter()

# perf_all = getParams(tuned_params)
scenario_all = getScenarios(perf_all)
end_time = time.process_time()
end_clock = time.perf_counter()

print("process time usage: {:.4} min".format((end_time - start_time)/60.0))
print("    cpu time usage: {:.4} min".format((end_clock - start_clock)/60.0))
# -



# +
# get figs
# print(dir(scenario_all[0][0].br))
fig_all =  getFigs(scenario_all, perf_all)
figs = list(map(list, zip(*fig_all)))  # transpose

output_file(HTML_FILE_PATH)
output_notebook()
fig_handle = bkp.show(layout(figs), notebook_handle=True)

# -





