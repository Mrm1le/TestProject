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
from plotter.composite_layers import SpeedMarginLayer, SpeedCurveLayer
from plotter.event_tools import MeasureTools
import ipywidgets
from ipywidgets.widgets.widget_box import HBox, VBox
from IPython.core.display import display, HTML



# +
APA_PARALLEL = "../../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../../resources/apa_vertical.yaml"                  # apa file for vertical scenario
CAR_TYPE = 'LCAR'
CAR_PARAMS = {
    'BCAR':"../../resources/vehicle_param_bcar.yaml",
    'LCAR':"../../resources/vehicle_param_l.yaml",
    'ETCAR':"../../resources/vehicle_param_etcar.yaml"
}
bag_file = "/home/yee/Downloads/apa_bag/speed_improve/0512_parallel/2023-05-12-20-42-47.bag" 
bag_file = "/home/yee/Downloads/apa_bag/0518/2023-05-18-19-32-48.bag"

SBP_REQUEST = "/msd/sbp_request"
PLANNING_PLAN = "/msd/planning/plan"
GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon

if not os.path.exists(bag_file):
    raise Exception('bag file does not exist:{}'.format(bag_file))
# -


car_param = CAR_PARAMS[CAR_TYPE]
apa_plan.initSingletonParams(car_param, APA_VERTICAL)


# +
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


planning_plan_msg = dataloader.getTopic(PLANNING_PLAN)
speed_margin_debugs = []
for msg in planning_plan_msg:
    json_str = msg[0].extra.json if hasattr(msg[0].extra, 'json') else ''
    extra =  json.loads(json_str) if json_str else {}
    if type(extra) != dict:
        continue
    if "speed_margin_debug" not in extra:
        continue
    speed_margin_debug = extra['speed_margin_debug']
    if not speed_margin_debug:
        continue
    
    speed_margin_debugs.append(EasyDict(json.loads(speed_margin_debug)))

print("read speed_margin_debugs size:{}".format(len(speed_margin_debugs)))
    
# -

#  margin velocity
odo_str = json.dumps(real_odo[1])
plan_res_str = apa_plan.planInterfaceSerialize(odo_str)
plan_res = EasyDict(json.loads(plan_res_str))
ks = getCurvature(plan_res.x, plan_res.y)
delta_len = len(plan_res.x) - len(ks)
head_ks = [0 for  i in range(delta_len)]
head_ks.extend(ks)
ks = head_ks
print(len(ks), len(plan_res.x))
print(ks)


sv_res = apa_plan.marginVelocityLimitDynamic(odo_str, ks, [0.8, 1, 0.8])
print("segments:", len(sv_res))
sv_seg_list = [
    ([sv[0] for sv in sv_list], [sv[1] for sv in sv_list]) for sv_list in sv_res
]

# +

p = figure(title="SpeedMarginLimiter",   # Step 2 
            #    match_aspect=True,
            x_axis_label='x',
            y_axis_label='y',
            width=1200,
            height=600)
colors = ['red', 'blue', 'green','red', 'blue', 'green']
legends = ['limitVByObstacle','filterByRemainS', 'limitVAtStart', 'adaptDynamicPlanning', 'inter res',]
lines = []
for index, sv_list in enumerate(sv_seg_list):
    l = p.line(sv_list[0], sv_list[1], legend_label=legends[index], line_width=2, color=colors[index], alpha = 0.5)
    l.visible = False
    lines.append(l)
lines[-3].visible = True
lines[-2].visible = True


p.legend.click_policy="hide"
output_notebook()
show(p)
pass


# +
# sv_res[0]

# +
# plot speed_margin_debugs
frame = 5
from plotter.planner_params import SliderParams
slider = SliderParams('frame', 0, 0, len(speed_margin_debugs)-1, 1)

scene_plot = figure(title="SpeedMarginLimiter",   # Step 2 
               match_aspect=True,
            x_axis_label='x',
            y_axis_label='y',
            width=1200,
            height=400)
scene_plot.legend.click_policy="hide"
scene_plot.toolbar.active_scroll = scene_plot.select_one(WheelZoomTool)
MeasureTools(scene_plot)

speed_plot = figure(title="vs",   # Step 2 
#                match_aspect=True,
            x_axis_label='x',
            y_axis_label='y',
            width=1200,
            height=300)
speed_plot.legend.click_policy="hide"
speed_plot.toolbar.active_scroll = speed_plot.select_one(WheelZoomTool)
sss = [ k[0] for k in speed_margin_debugs[frame].vec_sv]
ssv = [ k[1] for k in speed_margin_debugs[frame].vec_sv]
# speed_plot.line(sss, ssv, legend_label='sv', line_width=2, color='red', alpha = 0.5)


car_size_params = {'front_edge_to_rear_real': 4.015, 'length': 5.098, 'width': 2.115}


speed_margin_layer = SpeedMarginLayer(scene_plot,[{},{}, {}, {'line_color': 'green', 'legend_label':'planner_path'}, {'line_color': 'pink', 'legend_label':'planner_box'}])
spped_curve_layer = SpeedCurveLayer(speed_plot, [{'legend_label':'sv', 'line_width':2, 'color':'red', 'alpha':0.5}])
def frameCallback(frame_value):
    speed_margin_layer.updateUtil((speed_margin_debugs[frame_value.new], car_size_params))
    spped_curve_layer.updateUtil(speed_margin_debugs[frame_value.new].vec_sv)
    push_notebook()


slider.setCallBack(frameCallback)
slider.slider.value = 0
# output_notebook()
scene_plot.legend.click_policy="hide"
speed_plot.legend.click_policy="hide"
show(row(scene_plot, speed_plot),  notebook_handle=True)
display(HBox((
    ipywidgets.Label(
        "frame",
        layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
    ),
    slider.slider
)))


# +

print("up length:", len(speed_margin_debugs))
print("length:", len(speed_margin_debugs[0].vec_sv))

# -


