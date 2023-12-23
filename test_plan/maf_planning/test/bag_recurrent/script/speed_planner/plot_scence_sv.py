# +
import sys, os
import json
sys.path.insert(0, os.path.join(os.getcwd(), "../../lib"))
sys.path.insert(0, os.path.join(os.getcwd(), "../"))
import csv
import numpy as np
import py_parking_plotter as apa_plan
from easydict import EasyDict 

from plotter.event_tools import MeasureTools
from plotter.entity_layers import ObsPoints,PlannerPathLine,PlannerBox
from plotter.composite_layers import SceneSpeedMarginLayer, SpeedCurveLayer,MultiSpeedCurveLayer
from plotter.utils.common import GEO_TYPE, getCurvature
from bokeh.io import output_notebook,push_notebook,show
from bokeh.models import WheelZoomTool,ColumnDataSource, CustomJS
from bokeh.layouts import row
from bokeh.plotting import figure
import ipywidgets
from ipywidgets.widgets.widget_box import HBox
import collection_data_config as config

#plot
output_notebook()

from plotter.load_bag import DataLoader,DataGeneratorBase
try:
    import rosbag
except:
    print("There is error when importing rosbag, while it does not make a difference.")
bag_file="/home/mm/Downloads/bag/1502.bag"
bag_file = "/home/yee/Downloads/apa_bag/0607/PL99044_recording_dfdi_console_manual_recording_20230606-183022_20230606-183113_0.bag"


PATH_REQUEST="/msd/planning/plan"
SPEED_REQUEST="/mla/egopose"
ENV_REQUEST="/perception/fusion/ground_line_uss"
GEO_TYPE[0] = "6" 
SBP_REQUEST = "/msd/sbp_request"
# -


APA_PARALLEL = "../../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../../resources/apa_vertical.yaml"                  # apa file for vertical scenario
CAR_TYPE = 'BCAR'
CAR_PARAMS = {
    'BCAR':"../../resources/vehicle_param_bcar.yaml",
    'LCAR':"../../resources/vehicle_param_l.yaml",
    'ETCAR':"../../resources/vehicle_param_etcar.yaml"
}
car_param = CAR_PARAMS[CAR_TYPE]
print(car_param)
apa_plan.initSingletonParams(car_param, APA_VERTICAL)

# +
'''
生成plot的所需数据
'''
dataloader = DataLoader(bag_file)
planning_data_bag = dataloader.getTopic(PATH_REQUEST)
path_gen=config.PathGenerator(planning_data_bag)
path_debugs=path_gen.xys

v_data_bag= dataloader.getTopic(SPEED_REQUEST)
v_gen=config.EgoinfoGenerator(v_data_bag)
speed_margin_paras=path_gen.get_margin_paras()

env_data_bag= dataloader.getTopic(ENV_REQUEST)
env_gen=config.EnvGenerator(env_data_bag)

request_msg = dataloader.getTopic(SBP_REQUEST)
param_strs = []
real_odo = []
for msg in request_msg:
    if msg[0].task_info.goal_id.id != "search_problem":
        continue
    param_str=json.loads(msg[0].task_config.problem_config.params_string)
    temp_odo = EasyDict(param_str["OpenspaceDeciderOutput"])
    if msg[0].task_info.goal_id.id == "search_problem":
        real_odo.append(temp_odo)
    param_strs.append(msg[0].task_config.problem_config.params_string)
print("read request size:", len(param_strs))

# +
#sceence plot
WIDTH = 600
HEIGHT = 600
from plotter.planner_params import SliderParams
slider=SliderParams('frame',0,0,len(path_debugs)-1,step=1)
# slider.slider.layout.width = '400'
ts_res=[path_gen.get_this_ts(i) for i in range(len(path_debugs))]
env_debugs=[env_gen.atT(ts_res[i]) for i in range(len(path_debugs)) ]
scene_plot=figure(width=WIDTH, height=HEIGHT,
                  match_aspect=True,
                  title="planning trajectory",
           x_axis_label='x',y_axis_label='y',toolbar_location="below")
scene_plot.toolbar.active_scroll=scene_plot.select_one(WheelZoomTool)#滚轮缩放工具
MeasureTools(scene_plot)



speed_plot=figure(title="vs",
                  #match_aspect=True,
            x_axis_label='s/m',
            y_axis_label='velocity/(m/s)',
            width=WIDTH + 200,
            height=HEIGHT)
speed_plot.toolbar.active_scroll=speed_plot.select_one(WheelZoomTool)
# MeasureTools(speed_plot)
# -

# v_debugs=[v_gen.atT(ts_res[i]) for i in range(len(path_debugs))]
car_size_params = {'front_edge_to_rear_real': 4.015, 'length': 5.098, 'width': 2.115}
#更新滑动条指向的图片
scene_speed_margin_layer=SceneSpeedMarginLayer(scene_plot,[{'line_color': 'blue', 'legend_label':'obs_points'},
                                                            {'line_color': 'green', 'legend_label':'planner_path'}, 
                                                            {'line_color': 'pink', 'legend_label':'planner_box'}, 
                                                           {'line_color': 'black', 'legend_label':'margin_box', 'alpha':0.5}])
speed_curve_layer=MultiSpeedCurveLayer(speed_plot,[{'legend_label':'sv_margin_debug', 'line_width':2, 'line_color':'blue','alpha':0.5},
                                                {'legend_label':'adaptDynamicPlanning', 'line_width':2, 'line_color':'red','alpha':0.5},
                                                {'legend_label':'filterByRemainS', 'line_width':2, 'line_color':(200,150,100),'alpha':0.5},
                                                {'legend_label':'filterByRemainS_2', 'line_width':2, 'line_color':(200,70,100),'alpha':0.5},
                                                {'legend_label':'limitVByObstacle', 'line_width':2, 'line_color':(200,10,100),'alpha':0.5}],
                                    layer_num=5)

def frameCallback(frame_value):
    ind=frame_value.new
    print("frame====",ind)
    ks=path_gen.get_select_ks(ind)
    path_ind=min(ind,len(path_debugs)-1)
    path=path_debugs[path_ind]
    v_margin_debug=speed_margin_paras[ind]

    obs_pts = env_debugs[frame_value.new][0]
    obs_pts_bind = []
    for k  in range(len(obs_pts[0])):
        obs_pts_bind.append(apa_plan.Point2d(obs_pts[0][k], obs_pts[1][k], 0.0))
    debug_info = apa_plan.getSpeedMarginByParas(path[0],path[1],path[2],obs_pts_bind, ks, 0.1,v_margin_debug)
    
    all_sv=[]
    all_sv.append(debug_info.debug_vec_sv)
    res_vs=debug_info.vec_sv
    for i in range(len(res_vs)-1,-1,-1):
        all_sv.append(res_vs[i])
    speed_curve_layer.updateUtil(all_sv)
    scene_speed_margin_layer.updateUtil((path_debugs[frame_value.new], env_debugs[frame_value.new],car_size_params, debug_info.polygons))   
    push_notebook()

# +
slider.setCallBack(frameCallback)
if len(path_debugs)> 2:
    slider.slider.value = 1
slider.slider.value = 0

scene_plot.legend.click_policy="hide"
speed_plot.legend.click_policy="hide"


# +
from bokeh.layouts import gridplot
grid = gridplot([[scene_plot,speed_plot]])
show(grid,notebook_handle=True)

display(HBox((
    ipywidgets.Label(
        "FRAME",
        layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
    ),
    slider.slider
)))
print("up length:", len(path_debugs))

# -


