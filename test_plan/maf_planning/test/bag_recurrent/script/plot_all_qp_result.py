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

bag_file = "/home/ros/Downloads/rosbag/osqp/0424/9bb640b81bf43a31726119758016e72f.bag"
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



# +
v_spline_fig = bkp.figure(title='velocity ', x_axis_label='t', y_axis_label='v', match_aspect=True,
                       width=600, height=400, y_range=(0.0, 1.2))
v_spline_fig.toolbar.active_scroll = v_spline_fig.select_one(WheelZoomTool)

vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

st_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

pre_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

all_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

v_spline_fig.line('t', 'v', color='cyan', source=vel_data, line_width=2,
                legend_label='v')
v_spline_fig.line('t', 'v', color='green', source=st_vel_data, line_width=2,
                legend_label='st_v')
v_spline_fig.line('t', 'v', color='blue', source=pre_vel_data, line_width=2,
                legend_label='pre_v')
v_spline_fig.line('t', 'v', color='red', source=all_vel_data, line_width=0.5,
                legend_label='all_v')

v_spline_fig.legend.click_policy = 'hide'
# bkp.show(layout([[v_spline_fig]]), notebook_handle=True)


# +
import time

use_lib_flag = True

debug_output = ipywidgets.Output(layout={'border': '1px solid black'})
@debug_output.capture(clear_output=True)
def slider_callback(t):

    a_e = 0.0
    qp_result = planning_result[0]
#     len(qp_result)
    
#     s_list = [s0,s1]
    t_index = int(t)
    t_result = qp_result[t_index][0]
    v_result = qp_result[t_index][2]
    print("---------------->", t_result)
    qp_t_result = []
    qp_v_result = []
    st_t_result = []
    st_v_result = []
    index = 0
    for i in range(len(t_result)):
        if t_result[i] < 0:
            index = i
            break;
        qp_t_result.append(t_result[i])
        qp_v_result.append(v_result[i])
    print("-----------", qp_t_result)
    print("-----------", qp_v_result)
        
    for i in range(index + 1, len(t_result)):
        st_t_result.append(t_result[i])
        st_v_result.append(v_result[i])
    print("-----------****", st_t_result)
    print("-----------*****", st_v_result)
            
    qp_t_result = [data  + 0.1 * t for data in qp_t_result]
    st_t_result = [data  + 0.1 * t for data in st_t_result]
    
    all_t = []
    all_v = []
    for i in range(len(qp_result)):
        data = qp_result[i]
        if len(data[0]) > 0 and len(data[2]) > 0:
            for j in range(len(data[0])):
                if data[0][j] < 0:
                    break;
                all_t.append(data[0][j] + 0.1 * i)
                all_v.append(data[2][j])
    if t > 0:
        pre_t_result = qp_result[t_index - 1][0]
        pre_v_result = qp_result[t_index - 1][2]
        pre_t_result = [data  + (t - 1) * 0.1 for data in pre_t_result]           

    vel_data.data.update({
        't' : qp_t_result,
        'v' : qp_v_result,
    })
    
    st_vel_data.data.update({
        't' : st_t_result,
        'v' : st_v_result,
    })
    
#     if t > 0:
#         pre_vel_data.data.update({
#             't' : pre_t_result,
#             'v' : pre_v_result,
#         })
        
    all_vel_data.data.update({
            't' : all_t,
            'v' : all_v,
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
# s1_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=3.0)
# s2_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=6.0)
# s3_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=9.0)
# s4_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=12.0)
# s5_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=14.0)

# v0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.0)
# v1_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.6)
# v2_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.5)
# v3_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.0)
# v4_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.2)
# v5_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=1.0)

ipywidgets.interact(slider_callback, t=t_slider)
display(debug_output)
