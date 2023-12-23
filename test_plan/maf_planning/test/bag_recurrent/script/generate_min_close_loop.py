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
from bokeh.io import output_notebook, push_notebook, output_file
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
import py_parking_plotter as apa_plan

bag_file = "/home/ros/Downloads/apa_bag/min_close_loop"
html_path = "/home/ros/Downloads/apa_bag/min_close_loop/min_loop.html"
SBP_REQUEST = "/msd/sbp_request"
GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon
CAR_PARAM = "../resources/vehicle_param_l.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
min_loop_folder = "/home/ros/Downloads/apa_bag/min_loop_trejectory"
CAR_TYPE = 'BCAR'
CAR_PARAMS = {
    'BCAR':"../resources/vehicle_param_bcar.yaml",
    'LCAR':"../resources/vehicle_param_l.yaml"
}
CAR_PARAM = CAR_PARAMS[CAR_TYPE]




def getFirstOdoFromBag(bag_path):
    dataloader = DataLoader(bag_path)
    request_msg = dataloader.getTopic(SBP_REQUEST)
    for msg in request_msg:
        if msg[0].task_info.goal_id.id != "search_problem":
            continue
        param_string = msg[0].task_config.problem_config.params_string
        param_json = json.loads(param_string)
        
        if 'OpenspaceDeciderOutput' not in param_json or 'HybridAstarConfig' not in param_json:
            continue
        
        odo = EasyDict(param_json['OpenspaceDeciderOutput'])
        hybrid_astar_config = EasyDict(param_json['HybridAstarConfig'])
        planning_core = hybrid_astar_config.planning_core
        return True, odo, planning_core
    
    return False, None, None

def getAllBags(bag_file):
    if not os.path.exists(bag_file):
        raise Exception('bag file does not exist:{}'.format(bag_file))
    all_bag_files = []
    if os.path.isdir(bag_file):
        for f in os.listdir(bag_file):
            if f.split('.')[-1] != 'bag':
                continue
            all_bag_files.append(os.path.join(bag_file, f))
    else:
        all_bag_files.append(bag_file)
    print("found {} files".format(len(all_bag_files)))
    return all_bag_files

def getAllOdos(all_bag_files):
    odos = []
    planning_cores = []
    for bag in all_bag_files:
        is_exist, odo, planning_core = getFirstOdoFromBag(bag)
        if not is_exist:
            print("read no odos from {}".format(bag))
            continue
        
        if planning_core not in set([1,3]):
            print("planning_core is not in [1,3], which is {}".format(planning_core))
            continue
        
        odos.append(odo)
        planning_cores.append(planning_core)

    return odos, planning_cores

def planAll(odos, planning_cores):
    if not odos or not planning_cores:
        print("odos or planning_cores is none")
        return []
    
    if len(odos) != len(planning_cores):
        print("len(odos) = {}, len(planning_cores) = {}, which are not equal".format(len(odos), len(planning_cores)))
    
    ress = []
    for i in range(len(odos)):
        if planning_cores[i] == 1:
            apa_plan.initSingletonParams(CAR_PARAM, APA_VERTICAL)
        else:
            apa_plan.initSingletonParams(CAR_PARAM, APA_PARALLEL)
        res_str = apa_plan.planInterfaceSerialize(json.dumps(odos[i]))
        ress.append(EasyDict(json.loads(res_str)))
    
    return ress

all_bag_files = getAllBags(bag_file)
odos, planning_cores = getAllOdos(all_bag_files) 
ress = planAll(odos, planning_cores)
astar_planner_params_dict = EasyDict(json.loads(apa_plan.getAstarPlannerParams()))
car_size_params = EasyDict(json.loads(apa_plan.getCarSizeParams()))

len(all_bag_files)

# set layers
fig = bkp.figure(title='lane plot',
                 x_axis_label='x',
                 y_axis_label='y',
                 match_aspect=True,
                 width=1000,
                 height=600,
                 tools='box_select,reset,wheel_zoom,pan,crosshair')
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, PlannerFrameLayer
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
    pass
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
    pass
    search_tree_tuned.updateUtil(data)
    push_notebook() # force to update  

def path_callback(data):
    pass
    result_dict, car_size_params = data
    planner_output.updateUtil((result_dict, car_size_params))
    push_notebook()
    
def tuned_path_callback(data):
    pass
    result_dict, car_size_params = data
    planner_output_tuned.updateUtil((result_dict, car_size_params))
    push_notebook()


# -

from plotter.event_tools import MeasureTools
MeasureTools(fig)
# display plots
fig_handle = bkp.show(layout([fig]), notebook_handle=True)
from plotter.board_manager import BoardManager
board_manager = BoardManager(
    odos, ress, 
    [ [] for i in range(len(odos))], 
    [astar_planner_params_dict for i in range(len(odos))], 
    car_size_params, 
    ["" for i in range(len(odos))])
board_manager.setCallbacks(replan_callback, slider_callback, search_tree_callback, search_tree_tuned_callback, tuned_path_callback, path_callback)
board_manager.showWidgets()
display(debug_output)

len(ress)

ress[-1]
print(car_size_params)

ress[4]

# +
from plotter.utils.common import global2local

ress_at_origin = []
for res in ress:
    x0 = res.x[-1]
    y0 = res.y[-1]    
    phi0 = res.phi[-1]
    
    res_at_origin = {
        "x":[],
        "y":[],
        "phi":[],
        "wheel_base_offset":[]
    }
    
    x_len = len(res.x)
    for i in range(x_len):
        x1 = res.x[x_len-1-i]
        y1 = res.y[x_len-1-i]    
        phi1 = res.phi[x_len-1-i]
        
        x2,y2 = global2local(x1, y1, x0, y0, phi0)
        res_at_origin['x'].append(x2)
        res_at_origin['y'].append(y2)
        res_at_origin['phi'].append(phi1-phi0)
        res_at_origin['wheel_base_offset'].append(0)
    ress_at_origin.append(EasyDict(res_at_origin))

def saveTrajectory(ress_at_origin, min_loop_folder):
    for index,res in enumerate(ress_at_origin):
        file_path = os.path.join(min_loop_folder, "APA_TFT_{}_{}.json".format(CAR_TYPE, index))
        file_path = os.path.join(min_loop_folder, "APA_TFT_{}.json".format(37+index))
        with open(file_path, 'w') as json_file:
            json_file.write(json.dumps(res))
saveTrajectory(ress_at_origin, min_loop_folder)


# +

from bokeh.io import output_notebook, push_notebook, output_file
from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, PlannerFrameLayer

def displayBatchRes(all_bag_files, odos, ress_at_origin, car_size_params, html_path):
    figs = []
    column_num = 2
    
    output_file(html_path)
    plot_num = 0
    for index in range(len(odos)):
        if plot_num % column_num ==0:
            figs.append([])
        
        plot_num += 1
        plot_title = all_bag_files[index]

        # set layers
        fig = bkp.figure(title=plot_title,
                        x_axis_label='x',
                        y_axis_label='y',
                        match_aspect=True,
                        width=700,
                        height=500)

        # fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
        MeasureTools(fig)
        
        frame_layer = PlannerFrameLayer(fig)
        uss_frame = None
        frame_layer.update(None, ress_at_origin[index],uss_frame, car_size_params)
        fig.legend.click_policy = 'hide'
        
        figs[-1].append(frame_layer.getLayout())

        
        # -
    # fig_all = list(map(list, zip(*figs)))  # transpose
    bkp.show(layout(figs), notebook_handle=True)
    
displayBatchRes(all_bag_files, odos, ress_at_origin, car_size_params, html_path)
    
# -


