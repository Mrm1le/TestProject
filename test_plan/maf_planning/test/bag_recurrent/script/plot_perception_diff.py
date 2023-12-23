import sys
import os
import json
from easydict import EasyDict
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool
from bokeh.layouts import layout
from bokeh.io import output_notebook, push_notebook
from IPython.core.display import display, HTML
display(HTML("<style>.container { width:95% !important;  }</style>"))
display(
  HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
output_notebook()

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as bag_reader
bag_analyzer = bag_reader.BagAnalysis()

# +
bag_file_0 = "/home/ros/Downloads/apa_bag/1230/PLSNV105_recording_APA_PARKIN_RECORD_20211223-150611_20211223-150700_0.bag"
bag_file_1 = "/home/ros/Downloads/apa_bag/1228/PLSVV120_recording_APA_PARKIN_RECORD_no_cam_20211227-192557_20211227-192727_0.bag"
bag_file_0 = "/home/ros/Downloads/apa_bag/1230/52/PLAGD8050_recording_APA_PARKIN_RECORD_no_cam_20211230-165915_20211230-170038_0.bag"
bag_file_1 = "/home/ros/Downloads/apa_bag/1230/52/PLSVV120_recording_APA_PARKIN_RECORD_no_cam_20211230-160744_20211230-160826_0.bag"
bag_file_0 = "/home/ros/Downloads/apa_bag/1230/52_2/15119.bag"  # 52-2 dev
bag_file_1 = "/home/ros/Downloads/apa_bag/1230/52_2/15086.bag"  # 52-2 ep
# bag_file_1 = "/home/ros/Downloads/apa_bag/1230/52_2/15087.bag"  # 52-2 ep

if not os.path.exists(bag_file_0):
    raise Exception('bag file does not exist:{}'.format(bag_file_0))
if not os.path.exists(bag_file_1):
    raise Exception('bag file does not exist:{}'.format(bag_file_1))
# -

odo_str_array = bag_analyzer.comparePerceptions(bag_file_0, bag_file_1)
odo_list = [EasyDict(json.loads(x)) for x in odo_str_array ]
assert len(odo_list) == 4  
# print([o['T_lines'] for o in odo_list[:2]])

odo0 = odo_list[0]
odo1 = odo_list[1]
car_param0 = odo_list[2]
car_param1 = odo_list[3]

fig = bkp.figure(title='lane plot',
                 x_axis_label='x',
                 y_axis_label='y',
                 match_aspect=True,
                 width=1000,
                 height=600)
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer
planner_input0 = PlannerInputLayer(fig, [
    {'legend_label': 'obs_points0'},{'legend_label': 'obs_lines0'},
    {'legend_label': 'map_lines0'},{'legend_label': 'obs_boxs0'},
    {'legend_label': 'planner_boundary0'},{'legend_label': 'tlines0'},
    {'legend_label': 'target_car0'}, {'legend_label': 'init_car0'}
])
planner_input1 = PlannerInputLayer(fig, [
    {'legend_label': 'obs_points1', 'color':'#084594'},{'legend_label': 'obs_lines1', 'line_color':'#084594'},
    {'legend_label': 'map_lines1', 'line_color':'#084594'},{'legend_label': 'obs_boxs1','line_color':'#2171b5', 'fill_color':'#2171b5'},
    {'legend_label': 'planner_boundary1'},{'legend_label': 'tlines1'},
    {'legend_label': 'target_car1', 'line_color':'yellow', 'fill_color':'yellow'}, {'legend_label': 'init_car1'}
])
planner_input0.updateUtil((odo0, car_param0))
planner_input1.updateUtil((odo1, car_param1))
planner_input0.entity_layers[4].geo.plot.visible=False
planner_input0.entity_layers[5].geo.plot.visible=False
planner_input1.entity_layers[4].geo.plot.visible=False
planner_input1.entity_layers[5].geo.plot.visible=False

fig.legend.click_policy = 'hide'
fig_handle = bkp.show(layout([fig]), notebook_handle=True)





