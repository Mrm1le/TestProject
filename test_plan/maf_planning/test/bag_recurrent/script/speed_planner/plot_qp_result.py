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


# +

request_topic = "/msd/sbp_request"
dataloader = DataLoader(bag_file)
request_msg = dataloader.getTopic(SBP_REQUEST)
param_strs = []
for msg in request_msg:
    if msg[0].task_info.goal_id.id != "search_problem":
        continue
    param_strs.append(msg[0].task_config.problem_config.params_string)
print("read request size:", len(param_strs))

# +
# # +
# Scipy cubic spline ref:
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html

j_spline_fig = bkp.figure(title='jerk ', x_axis_label='t', y_axis_label='j', match_aspect=True,
                       width=600, height=400, y_range=(-20.0, 20.0))

a_spline_fig = bkp.figure(title='acceleration ', x_axis_label='t', y_axis_label='a', match_aspect=True,
                       width=600, height=400, y_range=(-6.0, 6.0))

v_spline_fig = bkp.figure(title='velocity ', x_axis_label='t', y_axis_label='v', match_aspect=True,
                       width=600, height=400, y_range=(0.0, 6.0))

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
def slider_callback(s0,s1,s2,s3,s4,s5,v0,v1,v2,v3,v4,v5):

    a_e = 0.0
    s_list = [s0,s1]
    v_list = [v0,v1]
#     s_list = [s0,s1,s2, s3, s4, s5]
#     v_list = [v0,v1,v2, v3, v4, v5]
#     s_list = [s0,s1]
#     v_list = [v0,v1]
# #     s_list = [0, 1.5, 3.488, 8.89255]
# #     v_list = [0.101, 0.8, 0.5, 0.2]
# #     s_list = [0, 1.5, 8.89255]
# #     v_list = [0.101, 0.8, 0.2]
# #     s_list = [0, 2.5, 10.89255]
# #     v_list = [0.8, 0.2, 1.0]
# #     s_list = [0, 5.6, 7, 10.89255]
# #     v_list = [0.101, 0.75, 0.2, 0.01]

# #     s_list = [0, 1.19928, 3.48775, 4.9166, 8.89282]
# #     v_list = [0.0, 0.5, 0.75, 0.2, 0.01]
# #     s_list = [0, 0.399762, 0.799523, 1.19928, 2.68822, 4.28727, 8.89282]
# #     v_list = [0.324064, 0.5, 0.75, 0.5, 0.75, 0.2, 0.01]
#     s_list = [0, 2.77666, 4.27666, 5.4023]
#     v_list = [0.766174, 0.75, 0.2, 0.01]
#     s_list = [0, 0.436687, 1.88585, 2.20477, 3.60477, 5.36487]
#     v_list = [0.00859379, 0.75, 0.5, 0.75, 0.2, 0.11]
#     s_list = [0, 9.23321, 9.63297, 16.3857]
#     v_list = [0.00232836, 0.75, 0.11, 0.0001]

#     s_list = [0, 0.5917, 3.8917, 5.0343]
#     v_list = [0.75095, 0.65,0.2, 0.11]
    
#     if use_lib_flag:
#         json_result = pp.constantJerkVelPlanner(s_list, v_list)
#         result = EasyDict(json.loads(json_result))
#         t_result = result['t_list']
#         j_result = result['j_list']
#         a_result = result['a_list']
#         v_result = result['v_list']
#         s_result = result['s_list']
#     else:
#         s_result, v_result = algorithm_run(s0,s1,s2,s3,s4,s5,v0,v1,v2,v3,v4,v5)
        
#     s_limit = [0,s1,s1,s2,s2,s3,s3, s4, s4,s5, s5]
#     v_limit = [v1,v1,v2, v2,v3, v3,v4, v4,v5, v5]
    dis_vel_data.data.update({
        's' : s_result,
        'v' : v_result,
#         'v_limit':v_limit
    })
    
    all_data.data.update({
        't' : t_result,
        'j' : j_result,
        'a' : a_result,
        'v' : v_result,
        's' : s_result,
#         'v_limit':v_limit
    })
#     dis_vel_limit_data.data.update({
#         's_limit' : s_limit,
#         'v_limit' : v_limit,
# #         'v_limit':v_limit
#     })
    
    push_notebook()

# display plots
bkp.show(layout([[s_v_fig, all_data_fig]]), notebook_handle=True)
# setup sliders
s0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=0.0)
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

ipywidgets.interact(slider_callback, s0=s0_slider, s1=s1_slider, s2=s2_slider, s3=s3_slider, s4=s4_slider,
                    s5=s5_slider, v0=v0_slider, v1=v1_slider, v2=v2_slider, v3=v3_slider, 
                    v4=v4_slider, v5=v5_slider)
display(debug_output)
