# -*- coding: utf-8 -*-
# +
import sys, os
sys.path.insert(0, os.path.join(os.getcwd(), "../../lib"))
sys.path.insert(0, os.path.join(os.getcwd(), "../"))

import py_parking_plotter as apa_plan

from bokeh.plotting import figure, curdoc
from bokeh.events import PointEvent, Tap
from bokeh.io import output_file, show
from bokeh.layouts import row, column
from bokeh.io import output_file, show, output_notebook, push_notebook
from bokeh.models import WheelZoomTool, Title
from plotter.load_bag import DataLoader
from plotter.utils.common import GEO_TYPE, getCurvature
import json
from easydict import EasyDict
from plotter.composite_layers import SpeedMarginLayer
from plotter.event_tools import MeasureTools


# +
APA_PARALLEL = "../../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../../resources/apa_vertical.yaml"                  # apa file for vertical scenario
CAR_TYPE = 'LCAR'
CAR_PARAMS = {
    'BCAR':"../../resources/vehicle_param_bcar.yaml",
    'LCAR':"../../resources/vehicle_param_l.yaml",
    'ETCAR':"../../resources/vehicle_param_etcar.yaml"
}
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/0309_2/1502.bag" 
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/sim/sim_lead.bag"
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/sim/130029.bag"
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/0320/130029_local1.bag"
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/ygg_0322_2/ygg_0322/2122.bag"
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/data0320_1/2023-03-20-11-38-17.bag"
# bag_file = "/home/yee/Downloads/apa_bag/speed_improve/ygg_0324/2023-03-24-10-50-16.bag"
bag_file = "/home/ros/Downloads/apa_bag/2023-04-09-16-08-14.bag"
SBP_REQUEST = "/msd/sbp_request"
PLANNING_PLAN = "/msd/planning/plan"
GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon

if not os.path.exists(bag_file):
    raise Exception('bag file does not exist:{}'.format(bag_file))
# -


car_param = CAR_PARAMS[CAR_TYPE]
print(car_param)
print(APA_VERTICAL)
apa_plan.initSingletonParams(car_param, APA_VERTICAL)


dataloader = DataLoader(bag_file)
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

#  margin velocity
odo_str = json.dumps(real_odo[0])
plan_res_str = apa_plan.planInterfaceSerialize(odo_str)
plan_res = EasyDict(json.loads(plan_res_str))
ks = getCurvature(plan_res.x, plan_res.y)
delta_len = len(plan_res.x) - len(ks)
head_ks = [0 for  i in range(delta_len)]
head_ks.extend(ks)
ks = head_ks
# print(len(ks), len(plan_res.x))
# print(ks)

def read_st_from_text(file_name):
    f = open(file_name, encoding='utf-8')
    st_data_list = []
    while True:
        line = f.readline()
        if line:
            data = line.split("ST: ")
            st_data = []
            if len(data) > 0:
                planning_data = data[1].split(" ")
                st_data.append(float(planning_data[0]))
                st_data.append(float(planning_data[1]))
                st_data.append(float(planning_data[2]))
                if len(st_data) > 0:
                    st_data_list.append(st_data)
#                 print(st_data)
                # print(planning_data[1])
            # path_x.append(data[1])
            # path_y.append(data[2])
        else:
            break
    f.close()
    return st_data_list


PLANNING_PLAN = "/msd/planning/plan"
dataloader = DataLoader(bag_file)
planning_data = dataloader.getTopic(PLANNING_PLAN)
param_strs = []
def get_planning_data(data):
    if data is None:
        return [[], []]
    ts = []
    xys = []

    for d in planning_data:
        ts.append(d[1])
        t_s = []
        s_s = []
        v_s = []
        for p in d[0].trajectory.velocity.vel_points:
            t_s.append(p.relative_time)
            s_s.append(p.distance)
            v_s.append(p.target_velocity)
        xys.append((t_s, s_s, v_s))
    return (xys, ts)
planning_result = get_planning_data(planning_data)

# +
# st_res = apa_plan.getSVByT(odo_str, ks, 0.1)
# st_res_optimized = apa_plan.solveSpeed(st_res, 0.1, 0.0)
st_data_list = read_st_from_text("/home/ros/Downloads/rosbag/osqp/0421/st_file.txt")
st_res = st_data_list
config = []
st_res_optimized = apa_plan.solveSpeedByConfig(st_data_list, 0.1, 0.0, config)

# print(st_data_list)
# st_res_optimized = apa_plan.solveSpeed(st_data_list, 0.1, 0.0)

# print(st_res)
# print(st_res_optimized)

use_file_st_for_debug = True


# +
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout
from bokeh.plotting import ColumnDataSource
import sys, ipywidgets

from IPython.core.display import display, HTML
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


j_spline_fig = bkp.figure(title='jerk ', x_axis_label='t', y_axis_label='j', match_aspect=True,
                       width=600, height=400, y_range=(-20.0, 20.0))

a_spline_fig = bkp.figure(title='acceleration ', x_axis_label='t', y_axis_label='a', match_aspect=True,
                       width=400, height=400, y_range=(-3.0, 5.0))

v_spline_fig = bkp.figure(title='velocity ', x_axis_label='t', y_axis_label='v', match_aspect=True,
                       width=400, height=400, y_range=(0.0, 1.2))

s_spline_fig = bkp.figure(title='distance ', x_axis_label='t', y_axis_label='s', match_aspect=True,
                       width=400, height=400, y_range=(0.0, 16.0))

s_v_fig = bkp.figure(title='distance-velocity ', x_axis_label='s', y_axis_label='v', match_aspect=True,
                       width=700, height=450, y_range=(0.0, 3.0))

all_data_fig = bkp.figure(title='all-data ', x_axis_label='t', y_axis_label='all', match_aspect=True,
                       width=700, height=450, y_range=(0.0, 10.0))

j_spline_fig.toolbar.active_scroll = j_spline_fig.select_one(WheelZoomTool)
v_spline_fig.toolbar.active_scroll = v_spline_fig.select_one(WheelZoomTool)
a_spline_fig.toolbar.active_scroll = a_spline_fig.select_one(WheelZoomTool)
s_spline_fig.toolbar.active_scroll = s_spline_fig.select_one(WheelZoomTool)
s_v_fig.toolbar.active_scroll = s_spline_fig.select_one(WheelZoomTool)
all_data_fig.toolbar.active_scroll = s_spline_fig.select_one(WheelZoomTool)

jerk_data = ColumnDataSource(data={
    't':[],
    'j':[],
    'j_max':[],
    'j_min':[],
})

acc_data = ColumnDataSource(data={
    't':[],
    'a':[],
})

vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

qp_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

dis_data = ColumnDataSource(data={
    't':[],
    's':[],
})

qp_dis_data = ColumnDataSource(data={
    't':[],
    's':[],
})

dis_vel_data = ColumnDataSource(data={
    's':[],
    'v':[],
})

dis_vel_limit_data = ColumnDataSource(data={
    's_limit':[],
    'v_limit':[],
})

all_data = ColumnDataSource(data={
    't':[],
    'j':[],
    'a':[],
    'v':[],
    's':[],
    
})

j_spline_fig.line('t', 'j', color='blue', source=jerk_data, line_width=2,
                legend_label='jerk')
j_spline_fig.line('t', 'j_max', color='red', source=jerk_data, line_width=2,
                legend_label='jerk')
j_spline_fig.line('t', 'j_min', color='red', source=jerk_data, line_width=2,
                legend_label='jerk')
a_spline_fig.line('t', 'a', color='blue', source=acc_data, line_width=2,
                legend_label='a')
# a_spline_fig.line('t', 'a_max', color='red', source=acc_data, line_width=2,
#                 legend_label='a_max')
# a_spline_fig.line('t', 'a_min', color='red', source=acc_data, line_width=2,
#                 legend_label='a_min')
v_spline_fig.line('t', 'v', color='cyan', source=vel_data, line_width=2,
                legend_label='st_v')
v_spline_fig.line('t', 'v', color='red', source=qp_vel_data, line_width=2,
                legend_label='qp_v')
s_spline_fig.line('t', 's', color='cyan', source=dis_data, line_width=2,
                legend_label='s')
s_spline_fig.line('t', 's', color='red', source=qp_dis_data, line_width=2,
                legend_label='qp_s')
s_v_fig.line('s', 'v', color='red', source=dis_vel_data, line_width=2,
                legend_label='s')
s_v_fig.line('s_limit', 'v_limit', color='green', source=dis_vel_limit_data, line_width=1,
                legend_label='s')

all_data_fig.line('t', 'j', color='blue', source=all_data, line_width=2,
                legend_label='j')
all_data_fig.line('t', 'a', color='red', source=all_data, line_width=2,
                legend_label='a')
all_data_fig.line('t', 'v', color='purple', source=all_data, line_width=2,
                legend_label='v')
all_data_fig.line('t', 's', color='cyan', source=all_data, line_width=2,
                legend_label='s')


a_spline_fig.legend.click_policy = 'hide'
v_spline_fig.legend.click_policy = 'hide'
s_spline_fig.legend.click_policy = 'hide'
s_v_fig.legend.click_policy = 'hide'
all_data_fig.legend.click_policy = 'hide'


# from get_result_for_constant_jerk import constant_jerk, draw_constant_jerk_pic

# from jerk_constant import calculate_jerk, plot_jerk
# from constant_jerk_algorithm import test_three, algorithm_run
# sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
# import py_parking_plotter as pp

import time

use_lib_flag = True

debug_output = ipywidgets.Output(layout={'border': '1px solid black'})
@debug_output.capture(clear_output=True)
def slider_callback(s_w,v_w,a_w,j_w,s_ref_w,v_ref_w,a_lo,a_up,j_lo,j_up, init_v):

#       double s_w;   // weight for s
#   double v_w;   // weight for v
#   double a_w;   // weight for a
#   double j_w;   // weight for jerk
#   double s_ref_w;   // weight for s_ref
#   double v_ref_w;   // weight for v_ref
#   double dt;    // delta t
#   double a_lo;  // for acc
#   double a_up;
#   double v_lo;  // for velocity
#   double v_up;
#   double j_lo;  // for jerk
#   double j_up;
    
    
#     a_e = 0.0
#     s_list = [s0,s1]
#     v_list = [v0,v1]
    if use_file_st_for_debug:
        st_data_list = read_st_from_text("/home/ros/Downloads/rosbag/osqp/0421/st_file.txt")
    st_res = st_data_list
    v_lo = 0.0
    v_up = 0.9
    dt = 0.1
#     print(st_data_list)
    config = [s_w,v_w,a_w,j_w, s_ref_w,v_ref_w,dt,a_lo,a_up,v_lo,v_up,j_lo,j_up]
#     st_res_optimized = apa_plan.solveSpeed(st_res, 0.1, 0.0)

    st_res_optimized = apa_plan.solveSpeedByConfig(st_data_list, 0.1, init_v, config)
#     print(st_res_optimized)

    st = [t[0] for t in st_res]
    ss = [t[1] for t in st_res]
    sv = [t[2] for t in st_res]
#     sa = []
    # sa = [t[3] for t in st_res]
#     sv_seg_list = (ss, sv, sa)
#     print("&&&&&&&&", ss)


    ss2 = [t[0] for t in st_res_optimized]
    sv2 = [t[1] for t in st_res_optimized]
    sa2 = [t[2] for t in st_res_optimized]
#     print("*********", ss2)

    vel_data.data.update({
        't' : st,
        'v' : sv,
    })
    dis_data.data.update({
        't' : st,
        's' : ss,
    })
    acc_data.data.update({
        't' : st,
        'a' : sa2,
    })
    
    qp_vel_data.data.update({
        't' : st,
        'v' : sv2,
    })
    qp_dis_data.data.update({
        't' : st,
        's' : ss2,
    })
    
    push_notebook()

# display plots
bkp.show(layout([[v_spline_fig, s_spline_fig, a_spline_fig]]), notebook_handle=True)
# setup sliders
s_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=0.0)
v_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=0.0)
a_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=5.0)
j_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=5.0)
s_ref_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=1.0)
v_ref_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=1.0)

# dt_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.1, max=0.1, value=0.1)
a_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=-2.0, max=-0.5, value=-0.5)
a_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=1.5, max=5.0, value=1.5)
# v_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.9)
# v_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.0)
j_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=-5.0, max=-1.0, value=-1.0)
j_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=2.0, max=5.0, value=2.0)
init_speed_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=1.0, value=0.0)

ipywidgets.interact(slider_callback, s_w=s_w_slider, v_w=v_w_slider, a_w=a_w_slider, j_w=j_w_slider, s_ref_w=s_ref_w_slider,
                    v_ref_w=v_ref_w_slider,a_lo=a_lo_slider, a_up=a_up_slider, j_lo=j_lo_slider, j_up=j_up_slider, 
                    init_v=init_speed_slider)
display(debug_output)
