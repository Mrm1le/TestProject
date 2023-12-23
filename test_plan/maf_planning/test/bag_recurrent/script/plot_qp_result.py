# +
import sys
import os
import numpy as np
import yaml, json
try:
    import rosbag
except:
    print("There is error when importing rosbag, while it does not make a difference.")
from easydict import EasyDict
import sys, ipywidgets
import numpy as np
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout
from bokeh.plotting import ColumnDataSource
from scipy.interpolate import CubicSpline
from plotter.load_bag import DataLoader

# Wide cells
from IPython.core.display import display, HTML
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()
# -

bag_file = "/home/ros/Downloads/rosbag/osqp/0422/9bb640b81bf43a31726119758016e72f.bag"
if not os.path.exists(bag_file):
    raise Exception('bag file does not exist:{}'.format(bag_file))

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
print(len(planning_result[1]))
# print(planning_result[0])

# +
# # +
# Scipy cubic spline ref:
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html

j_spline_fig = bkp.figure(title='jerk ', x_axis_label='t', y_axis_label='j', match_aspect=True,
                       width=600, height=400, y_range=(-20.0, 20.0))

a_spline_fig = bkp.figure(title='acceleration ', x_axis_label='t', y_axis_label='a', match_aspect=True,
                       width=600, height=400, y_range=(-6.0, 6.0))

v_spline_fig = bkp.figure(title='velocity ', x_axis_label='t', y_axis_label='v', match_aspect=True,
                       width=600, height=400, y_range=(0.0, 1.2))

s_spline_fig = bkp.figure(title='distance ', x_axis_label='t', y_axis_label='s', match_aspect=True,
                       width=600, height=400, y_range=(0.0, 16.0))

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
    'a_max':[],
    'a_min':[],
})

vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

pre_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

dis_data = ColumnDataSource(data={
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
a_spline_fig.line('t', 'a_max', color='red', source=acc_data, line_width=2,
                legend_label='a_max')
a_spline_fig.line('t', 'a_min', color='red', source=acc_data, line_width=2,
                legend_label='a_min')
v_spline_fig.line('t', 'v', color='cyan', source=vel_data, line_width=2,
                legend_label='v')
v_spline_fig.line('t', 'v', color='blue', source=pre_vel_data, line_width=2,
                legend_label='pre_v')
s_spline_fig.line('t', 's', color='purple', source=dis_data, line_width=2,
                legend_label='s')
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
def slider_callback(t,s1,s2,s3,s4,s5,v0,v1,v2,v3,v4,v5):

    a_e = 0.0
    qp_result = planning_result[0]
#     len(qp_result)
    
#     s_list = [s0,s1]
    t_index = int(t)
    t_result = qp_result[t_index][0]
    v_result = qp_result[t_index][2]
    t_result = [data  + 0.1 * t for data in t_result]

    if t > 0:
        pre_t_result = qp_result[t_index - 1][0]
        pre_v_result = qp_result[t_index - 1][2]
        pre_t_result = [data  + (t - 1) * 0.1 for data in pre_t_result]           

    vel_data.data.update({
        't' : t_result,
        'v' : v_result,
#         'v_limit':v_limit
    })
    
    if t > 0:
        pre_vel_data.data.update({
            't' : pre_t_result,
            'v' : pre_v_result,
    #         'v_limit':v_limit
        })
    
#     all_data.data.update({
#         't' : t_result,
#         'j' : j_result,
#         'a' : a_result,
#         'v' : v_result,
#         's' : s_result,
# #         'v_limit':v_limit
#     })
#     dis_vel_limit_data.data.update({
#         's_limit' : s_limit,
#         'v_limit' : v_limit,
# #         'v_limit':v_limit
#     })
    
    push_notebook()

# display plots
bkp.show(layout([[v_spline_fig]]), notebook_handle=True)
# setup sliders
t_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0, max=len(planning_result[1]), value=0)
s1_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=3.0)
s2_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=6.0)
s3_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=9.0)
s4_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=12.0)
s5_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=14.0)

v0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.0)
v1_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.6)
v2_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.5)
v3_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.0)
v4_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.2)
v5_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.0)

ipywidgets.interact(slider_callback, t=t_slider, s1=s1_slider, s2=s2_slider, s3=s3_slider, s4=s4_slider,
                    s5=s5_slider, v0=v0_slider, v1=v1_slider, v2=v2_slider, v3=v3_slider, 
                    v4=v4_slider, v5=v5_slider)
display(debug_output)
