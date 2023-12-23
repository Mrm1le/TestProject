# -*- coding: utf-8 -*-
# %load_ext autoreload
# %autoreload 2

import sys
import os
import numpy as np
import yaml, json
import copy
try:
    import rosbag
except:
    print("There is error when importing rosbag, while it does not make a difference.")
from easydict import EasyDict

import ipywidgets
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout
from plotter.load_bag import DataLoader
from plotter.utils.common import GEO_TYPE

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from IPython.core.display import display, HTML
display(HTML("<style>.container { width:95% !important;  }</style>"))
display(
  HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
output_notebook()

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan

CAR_PARAM_RX5 = "../resources/vehicle_param_rx5.yaml"
CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
APA_VERTICAL_2ND = "../resources/apa_vertical_2nd.yaml"                  # apa file for vertical scenario 2
GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon



odo_json_file = "/home/ros/Downloads/apa_bag/0419/PLAA60351_recording_RVIZ_dx_cc_sp_67_7_20220326-141134_20220326-141235_0_g_gt.json"  

if not os.path.exists(odo_json_file):
    raise Exception('json file does not exist:{}'.format(odo_json_file))

# +
json_data = None
with open(odo_json_file) as json_file:
    json_data = json.load(json_file)
if not json_data or 'odo' not in json_data:
    raise Exception("failed to load json data from {}".format(odo_json_file))
odo_json = json_data['odo']


init_states = []
if "init_points" not in json_data:
    init_states.append((
        odo_json['init_state']['path_point']['x'],
        odo_json['init_state']['path_point']['y'],
        odo_json['init_state']['path_point']['theta']
    ))
else:
    for p in json_data['init_points']:
        init_states.append((
            p['x'], p['y'], p['theta']
        ))


slot_type = apa_plan.ParkingSlotType.UNKNOWN
if 'slot_type' in json_data  and json_data['slot_type'] == "ParkingSlotType.PARALLEL":
    print("planning in paralell slot")
    apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)
else:
    print("planning in vertical slot")
    apa_plan.initSingletonParams(CAR_PARAM_EP, APA_VERTICAL)

odo_str = json.dumps(odo_json)
# res_str = apa_plan.planInterfaceSerialize(odo_str)
# # -

odo_dict = EasyDict(odo_json)
# res_dict = EasyDict(json.loads(res_str))


astar_planner_params_dict = EasyDict(json.loads(apa_plan.getAstarPlannerParams()))
car_size_params = EasyDict(json.loads(apa_plan.getCarSizeParams()))

odo_list = []
res_list = []
search_edges = []
astar_planner_params = []
param_string_array = []
for i in range(len(init_states)):
    cur_odo = copy.deepcopy(odo_dict)
    cur_odo['target_state']['path_point']['x'] = init_states[i][0]
    cur_odo['target_state']['path_point']['y'] = init_states[i][1]
    cur_odo['target_state']['path_point']['theta'] = init_states[i][2]
    odo_list.append(cur_odo)
    odo_str = json.dumps(cur_odo)
    res_str = apa_plan.planInterfaceSerialize(odo_str)
    res_dict = EasyDict(json.loads(res_str))
    res_list.append(res_dict)
    search_edges.append([])
    astar_planner_params.append(astar_planner_params_dict)
    param_string_array.append("")

# +

# set layers
fig = bkp.figure(title='lane plot',
                 x_axis_label='x',
                 y_axis_label='y',
                 match_aspect=True,
                 width=1000,
                 height=600,
                 tools='box_select,reset,wheel_zoom,pan,crosshair')
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer
from plotter.composite_layers import SearchTreeLayer
planner_input = PlannerInputLayer(fig,[{},{},{},{},{},{},{'legend_label': 'target_car'}, {'legend_label': 'init_car'}])
planner_output = PlannerOutputLayer(fig,  {'line_color': 'green', 'legend_label':'planner_path (base/devcar)'}, {'line_color': 'pink', 'legend_label':'planner_box_base'})
planner_output_tuned = PlannerOutputLayer(fig, {'line_color': 'red', 'legend_label':'planner_path (tuned/epcar)'}, {'line_color': 'purple', 'legend_label':'planner_box_tuned', 'alpha':0.3})
search_tree = SearchTreeLayer(fig, {'line_color': 'grey', 'legend_label': "search_tree_base"})
search_tree_tuned = SearchTreeLayer(fig, {'line_color': 'brown', 'legend_label': "search_tree_tuned"})
fig.legend.click_policy = 'hide'

# callback
debug_output = ipywidgets.Output(layout={'border': '1px solid black'})
# @debug_output.capture(clear_output=True)
def slider_callback(data):
    odo_dict, result_dict, car_size_params = data
    planner_input.updateUtil((odo_dict, car_size_params))
    planner_output.updateUtil((result_dict, car_size_params))

    push_notebook() # force to update

@debug_output.capture(clear_output=True)
def replan_callback(data):
    logger.info("replan ...")
    res_str = None
    try:
        res_str = apa_plan.planInterfaceSerializeParams(data)
    except Exception:
        logger.info("failed to get result")
        return None
    
    if res_str is None:
        logger.info("failed to get result")
        return None
    
    res_dict = None
    try:
        res_dict = EasyDict(json.loads(res_str))
    except Exception:
        logger.info("failed to get result")
        return None
    
    if res_dict is None:
        logger.info("failed to get result")
    else:
        logger.info("get result, path size:{}".format(len(res_dict.x)))
        
    return res_dict, car_size_params, []

# +
# @debug_output.capture(clear_output=True)
def search_tree_callback(data):
    pass 
    
@debug_output.capture(clear_output=True)
def search_tree_tuned_callback(data):
    search_tree_tuned.updateUtil(data)
    push_notebook() # force to update  

def path_callback(data):
    result_dict, car_size_params = data
    planner_output.updateUtil((result_dict, car_size_params))
    push_notebook()
    
def tuned_path_callback(data):
    result_dict, car_size_params = data
    planner_output_tuned.updateUtil((result_dict, car_size_params))
    push_notebook()


# -

from plotter.event_tools import MeasureTools
MeasureTools(fig)
# display plots
fig_handle = bkp.show(layout([fig]), notebook_handle=True)
from plotter.board_manager import BoardManager
board_manager = BoardManager(odo_list, res_list,search_edges, astar_planner_params, car_size_params, param_string_array)
board_manager.setCallbacks(replan_callback, slider_callback, search_tree_callback, search_tree_tuned_callback, tuned_path_callback, path_callback)
board_manager.showWidgets()
display(debug_output)

len(astar_planner_params)
# astar_planner_params[0:4]
# print(board_manager.planner_out_copy)


















