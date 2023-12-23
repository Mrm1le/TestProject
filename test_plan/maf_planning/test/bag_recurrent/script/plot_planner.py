# -*- coding: utf-8 -*-
# %load_ext autoreload
# %autoreload 2

import sys
import os
import numpy as np
import yaml, json
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
from plotter.utils.common import getCurvature

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from IPython.core.display import display, HTML
display(HTML("<style>.container { width:95% !important;  }</style>"))
display(
  HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
output_notebook()

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
bag_file = "/home/yee/Downloads/apa_bag/0515_parallel/2023-05-15-16-23-22.bag" 
bag_file = "/home/yee/Downloads/apa_bag/0518/2023-05-18-19-32-48.bag"
SBP_REQUEST = "/msd/sbp_request"
GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon




if not os.path.exists(bag_file):
    raise Exception('bag file does not exist:{}'.format(bag_file))

request_topic = "/msd/sbp_request"
dataloader = DataLoader(bag_file)
request_msg = dataloader.getTopic(SBP_REQUEST)
param_strs = []
for msg in request_msg:
    if msg[0].task_info.goal_id.id != "search_problem":
        continue
    param_strs.append(msg[0].task_config.problem_config.params_string)
print("read request size:", len(param_strs))

import py_parking_plotter as bag_reader
bag_analyzer = bag_reader.BagAnalysis()
def readPlanFromBag(bag_file):
    bag_analyzer.loadBagAndAnalyze(param_strs)

    odo_string = bag_analyzer.getBagEnvironmentInfos()
    result_string = bag_analyzer.getBagResults()
    bag_debug = bag_analyzer.getBagResDebugs()
    params_array = bag_analyzer.getPlannerParams()
    
    odo_list = [EasyDict(json.loads(x)) for x in odo_string ]
    result_list = [EasyDict(json.loads(x)) for x in result_string ]
    search_edges = [x.added_edges for x in bag_debug]
    astar_planner_params = [ EasyDict(x) for x in  json.loads(params_array)] 
    car_size_params = EasyDict(json.loads(bag_analyzer.getCarSizeParams()))
    param_string_array = bag_analyzer.getParamString()
    
    return (odo_list, result_list, search_edges, astar_planner_params, car_size_params, param_string_array)

odo_list, res_list, search_edges, astar_planner_params, car_size_params, param_string_array = readPlanFromBag(bag_file)
print("params: ",param_string_array )


# set layers
fig = bkp.figure(title='lane plot',
                 x_axis_label='x',
                 y_axis_label='y',
                 match_aspect=True,
                 width=1000,
                 height=600,
                 tools='box_select,reset,wheel_zoom,pan,crosshair')
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, EnvironmentSlotLayer
from plotter.composite_layers import SearchTreeLayer
planner_input = PlannerInputLayer(fig,[{},{},{},{},{},{},{'legend_label': 'target_car'}, {'legend_label': 'init_car'}])
planner_output = PlannerOutputLayer(fig,  {'line_color': 'green', 'legend_label':'planner_path (base/devcar)'}, {'line_color': 'pink', 'legend_label':'planner_box_base'})
planner_output_tuned = PlannerOutputLayer(fig, {'line_color': 'red', 'legend_label':'planner_path (tuned/epcar)'}, {'line_color': 'purple', 'legend_label':'planner_box_tuned', 'alpha':0.3})
environment_slot_output = EnvironmentSlotLayer(fig,  {'line_color': 'blue', 'legend_label':'environment_slot'})
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
    environment_slot_output.updateUtil(result_dict)

    push_notebook() # force to update

@debug_output.capture(clear_output=True)
def replan_callback(data):
    print("replan ...")
    logger.info("replan ...")
    search_process_debug=bag_reader.SearchProcessDebug()
    res_str = bag_analyzer.plan(data, search_process_debug)
    res_dict = EasyDict(json.loads(res_str))
    if res_dict is None:
        print("failed to get result")
    else:
        print("get result")
    return res_dict.sbp_result, res_dict.car_params, search_process_debug.added_edges

# +
# @debug_output.capture(clear_output=True)
def search_tree_callback(data):
    search_tree.updateUtil(data)
    push_notebook() # force to update   
    
@debug_output.capture(clear_output=True)
def search_tree_tuned_callback(data):
    search_tree_tuned.updateUtil(data)
    push_notebook() # force to update  

def path_callback(data):
    result_dict, car_size_params = data
    planner_output.updateUtil((result_dict, car_size_params))
    environment_slot_output.updateUtil(result_dict)
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

# +
xs = res_list[1].x[::-1]
ys = res_list[1].y[::-1]

ks = getCurvature(xs, ys)
fig2 = bkp.figure(title="curvature with spiral planner",
                 x_axis_label='path sample pose',
                 y_axis_label='kappa',
#                  match_aspect=True,
                 width=1000,
                 height=600)
# MeasureTools(fig)
fig2.title.align = 'center'
fig2.toolbar.active_scroll = fig.select_one(WheelZoomTool)
output_notebook()
fig2.line(range(len(ks)),ks,line_width= 3, line_color = 'blue', alpha = 0.5)


# bkp.show(layout(fig2), notebook_handle=True)
# -
car_size_params











