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
                       plot_width=600, plot_height=400, y_range=(-20.0, 20.0))

a_spline_fig = bkp.figure(title='acceleration ', x_axis_label='t', y_axis_label='a', match_aspect=True,
                       plot_width=200, plot_height=400, y_range=(-3.0, 5.0))

v_spline_fig = bkp.figure(title='velocity ', x_axis_label='t', y_axis_label='v', match_aspect=True,
                       plot_width=1600, plot_height=400, y_range=(0.0, 1.2))

s_spline_fig = bkp.figure(title='distance ', x_axis_label='t', y_axis_label='s', match_aspect=True,
                       plot_width=200, plot_height=400, y_range=(0.0, 16.0))

s_v_fig = bkp.figure(title='distance-velocity ', x_axis_label='s', y_axis_label='v', match_aspect=True,
                       plot_width=700, plot_height=450, y_range=(0.0, 3.0))

all_data_fig = bkp.figure(title='all-data ', x_axis_label='t', y_axis_label='all', match_aspect=True,
                       plot_width=700, plot_height=450, y_range=(0.0, 10.0))

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

ref_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

qp_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

replay_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

all_init_vel_data = ColumnDataSource(data={
    't':[],
    'v':[],
})

all_init_acc_data = ColumnDataSource(data={
    't':[],
    'a':[],
})

all_vel_data = ColumnDataSource(data={
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

replay_dis_data = ColumnDataSource(data={
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
v_spline_fig.line('t', 'v', color='green', source=ref_vel_data, line_width=2,
                legend_label='ref_v')
v_spline_fig.line('t', 'v', color='red', source=qp_vel_data, line_width=2,
                legend_label='qp_v')
v_spline_fig.line('t', 'v', color='blue', source=replay_vel_data, line_width=2,
                legend_label='bag_qp_v')
v_spline_fig.inverted_triangle('t', 'v', color='red', source=all_init_vel_data, line_width=2,
                legend_label='init_v')
v_spline_fig.inverted_triangle('t', 'a', color='green', source=all_init_acc_data, line_width=0.5,
                legend_label='init_a')
v_spline_fig.multi_line('t', 'v', color='green', source=all_vel_data, line_width=0.2,
                legend_label='all_v')
s_spline_fig.line('t', 's', color='cyan', source=dis_data, line_width=2,
                legend_label='s')
s_spline_fig.line('t', 's', color='red', source=qp_dis_data, line_width=2,
                legend_label='qp_s')
s_spline_fig.line('t', 's', color='blue', source=replay_dis_data, line_width=2,
                legend_label='replay_s')
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


# -

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


# +
PLANNING_PLAN = "/msd/planning/plan"
# bag_file = "/home/ros/Downloads/rosbag/osqp/0_duan_0510_291/2023-05-10-19-46-48.bag"
bag_file = "/home/ros/Downloads/rosbag/osqp/0705/2023-07-05-16-20-17.bag"
# bag_file = "/home/ros/Downloads/rosbag/osqp/rosbag0608/2023-06-08-15-53-06.bag"

st_file = "/home/ros/Downloads/rosbag/osqp/st_file.txt"

if not os.path.exists(bag_file):
    raise Exception('bag file does not exist:{}'.format(bag_file))
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
        a_s = []
        vel_points = d[0].trajectory.velocity.vel_points
        acc_points = d[0].trajectory.acceleration.acc_points
        for i in range(len(vel_points)):
            p = vel_points[i]
            t_s.append(p.relative_time)
            s_s.append(p.distance)
            v_s.append(p.target_velocity)
            a_s.append(acc_points[i].acc)
        xys.append((t_s, s_s, v_s, a_s))
#         print(s_s)
    return (xys, ts)
planning_result = get_planning_data(planning_data)

# +
st_data_list = read_st_from_text(st_file)
st_res = st_data_list
config = []
# st_res_optimized = apa_plan.solveSpeedByConfig(st_data_list, 0.1, 0.0, 0.0, config)

# st_res_optimized = apa_plan.solveSpeed(st_data_list, 0.1, 0.0)

use_file_st_for_debug = False

# +


import time

use_lib_flag = True

debug_output = ipywidgets.Output(layout={'border': '1px solid black'})
@debug_output.capture(clear_output=True)
def slider_callback(t,s_w,v_w,a_w,j_w,s_ref_w,v_ref_w,a_lo,a_up,j_lo,j_up, init_v):
    
    
    
    qp_result = planning_result[0]

    t_index = int(t)
    print(t_index)
    t_result = qp_result[t_index][0]
    s_result = qp_result[t_index][1]
    v_result = qp_result[t_index][2]
    a_result = qp_result[t_index][3]
#     print("---------------->", t_result)
#     print("---------------->", v_result)
    st_data_list = []
    st_res_optimized = []
    
    st_data_list_txt = read_st_from_text(st_file)
#     st_res = st_data_list
    v_lo = 0.0
    v_up = 0.9
    dt = 0.1
    

    qp_t_result = []
    qp_s_result = []
    qp_v_result = []
    st_t_result = []
    st_s_result = []
    st_v_result = []
    replay_qp_result = []
    index = 0
    first_obs_index = 0
    second_obs_index = 0
    print("##########", t_index)
    for i in range(len(t_result)):
        if t_result[i] < 0:
            index = i
            break;
        qp_data_list = []
        qp_t_result.append(t_result[i] + 0.1 * t_index)
        qp_s_result.append(s_result[i])
        qp_v_result.append(v_result[i])
        qp_data_list.append(t_result[i])
        qp_data_list.append(s_result[i])
        qp_data_list.append(v_result[i])
        replay_qp_result.append(qp_data_list)
    init_a = 0.0
    print("&&&&&&&&&&", replay_qp_result)

    
    for i in range(len(t_result)):
        if t_result[i] == -9 :
            first_obs_index = i
            break;
    for i in range(len(t_result)):
        if t_result[i] == -7 :
            second_obs_index = i
            break;
    print("------------", index)
    print("------------", first_obs_index)
    print("------------", second_obs_index)
    print("------------", len(t_result))
    
    first_obs_st = []
    second_obs_st = []
        
    if index < len(v_result):
        print("----------> init_v:", v_result[index])
        print("----------> init_a:", a_result[index])
        print("----------> t:", t_result[index])
        init_v = v_result[index]
        init_a = a_result[index]
    st_data_list_bag = []
    index_max_scope = len(t_result)
    if index < first_obs_index:
        index_max_scope = first_obs_index
    for i in range(index + 1, index_max_scope):
        st_data = []
        st_t_result.append(t_result[i] + 0.1 * t_index)
        st_s_result.append(s_result[i])
        st_v_result.append(v_result[i])
        st_data.append(t_result[i])
        st_data.append(s_result[i])
        st_data.append(v_result[i])
        st_data_list_bag.append(st_data)
#         print("=========", s_result[i])
#     print("-----------****", st_t_result)
#     print("-----------*****", st_v_result)
    if index < first_obs_index and first_obs_index <= second_obs_index:
        for i in range(first_obs_index + 1, second_obs_index):
            st_data = []
#             st_t_result.append(t_result[i])
#             st_s_result.append(s_result[i])
#             st_v_result.append(v_result[i])
            st_data.append(t_result[i])
            st_data.append(s_result[i])
            st_data.append(v_result[i])
            first_obs_st.append(st_data)
            print("first obs t : " , t_result[i], v_result[i])
    if index < first_obs_index and first_obs_index <= second_obs_index and second_obs_index < len(t_result):
        for i in range(second_obs_index + 1, len(t_result)):
            st_data = []
#             st_t_result.append(t_result[i])
#             st_s_result.append(s_result[i])
#             st_v_result.append(v_result[i])
            st_data.append(t_result[i])
            st_data.append(s_result[i])
            st_data.append(v_result[i])
            second_obs_st.append(st_data)
    all_t = []
    all_v = []
    all_init_t = []
    all_init_v = []
    all_init_a = []
    for i in range(len(qp_result)):
        data = qp_result[i]
        cur_t = []
        cur_v = []
        if len(data[0]) > 0 and len(data[2]) > 0:
            for j in range(len(data[0])):
                if data[0][j] < 0:
                    break;
                if j == 0:
                    all_init_v.append(data[2][j])
                    all_init_a.append(data[3][j])
                    all_init_t.append(data[0][j] + 0.1 * i)
                cur_t.append(data[0][j] + 0.1 * i)
                cur_v.append(data[2][j])
        all_t.append(cur_t)
        all_v.append(cur_v)
                
                
    if use_file_st_for_debug:
        st_data_list = st_data_list_txt
    else:
        st_data_list = st_data_list_bag
    
    config = [s_w,v_w,a_w,j_w, s_ref_w,v_ref_w,dt,a_lo,a_up,v_lo,v_up,j_lo,j_up]
#     st_res_optimized = apa_plan.solveSpeed(st_res, 0.1, 0.0)
#     print("****************", st_data_list)
    print("==================curr a is:", init_a)
#     first_obs_st = []
#     second_obs_st =[]

    st_res_optimized = apa_plan.solveSpeedByConfig(st_data_list, first_obs_st, second_obs_st, 0.1, init_v, init_a, config)
#     st_res_optimized = replay_qp_result
    print("result is:", st_res_optimized)
        

    st = [t[0]+ 0.1 * t_index for t in st_data_list]
    ss = [t[1] for t in st_data_list]
    sv = [t[2] for t in st_data_list]


    ref_sv = []
    for i in range(len(sv)):
        ref_v = sv[i]
        if len(first_obs_st) - 1 < i:
            ref_sv.append(ref_v)
            continue
        if sv[i] > first_obs_st[i][2]:
            ref_v = (0.5 * sv[i] + 0.5 * first_obs_st[i][2])
            ref_sv.append(ref_v)
        else :
            ref_sv.append(ref_v)

    ss2 = [t[0] for t in st_res_optimized]
    sv2 = [t[1] for t in st_res_optimized]
    sa2 = [t[2] for t in st_res_optimized]
    
    print("st:", st)
    print("sv2:", sv2)
#     print("*********", sv2)

    vel_data.data.update({
        't' : st,
        'v' : sv,
    })
    ref_vel_data.data.update({
        't' : st,
        'v' : ref_sv,
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
    replay_vel_data.data.update({
        't' : qp_t_result,
        'v' : qp_v_result,
    })
    all_vel_data.data.update({
        't' : all_t,
        'v' : all_v,
    })
    all_init_vel_data.data.update({
        't' : all_init_t,
        'v' : all_init_v,     
    }
    )
    all_init_acc_data.data.update({
        't' : all_init_t,
        'a' : all_init_a,     
    }
    )
    qp_dis_data.data.update({
        't' : st,
        's' : ss2,
    })
    replay_dis_data.data.update({
        't' : st,
        's' : qp_s_result,
    })
    
    push_notebook()

# display plots
bkp.show(layout([[v_spline_fig, s_spline_fig, a_spline_fig]]), notebook_handle=True)
# setup sliders
t_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=len(planning_result[1]), value=290.0)
s_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=0.0)
v_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=0.0)
a_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=5.0)
j_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=15.0, value=5.0)
s_ref_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=25.0, value=15.0)
v_ref_w_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=15.0, max=150.0, value=15.0)

# dt_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.1, max=0.1, value=0.1)
a_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=-2.0, max=-0.5, value=-0.5)
a_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=1.5, max=5.0, value=1.5)
# v_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.9)
# v_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=2.5, value=0.0)
j_lo_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=-5.0, max=-1.0, value=-1.0)
j_up_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=2.0, max=5.0, value=2.0)
init_speed_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), min=0.0, max=1.0, value=0.0)

ipywidgets.interact(slider_callback, t=t_slider, s_w=s_w_slider, v_w=v_w_slider, a_w=a_w_slider, j_w=j_w_slider, s_ref_w=s_ref_w_slider,
                    v_ref_w=v_ref_w_slider,a_lo=a_lo_slider, a_up=a_up_slider, j_lo=j_lo_slider, j_up=j_up_slider, 
                    init_v=init_speed_slider)
display(debug_output)
# -


