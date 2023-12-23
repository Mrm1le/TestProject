# -*- coding: utf-8 -*-
import os
import sys
import json
from easydict import EasyDict
import logging
import math
logging.basicConfig(level=logging.INFO)

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as pp
from plotter.utils.common import GEO_TYPE
from plotter.utils.common import getCurvature

# +
CAR_PARAM_RX5 = "../resources/vehicle_param_rx5.yaml"
CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
CAR_PARAM_WLS_C03 = "../resources/vehicle_param_wls_c03.yaml"
CAR_PARAM_LHS_SG = "../resources/vehicle_param_sg.yaml"
RPA_STRAIGHT = "../resources/rpa_straight.yaml"                  # apa file for parallel scenario
CAR_PARAM_PTCAR = "../resources/vehicle_param_ptcar.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
APA_VERTICAL_2ND = "../resources/apa_vertical_2nd.yaml"                  # apa file for vertical scenario 2
APA_OBLIQUE = "../resources/apa_oblique.yaml"
APOA_VERTICAL = "../resources/apoa_vertical.yaml"
APOA_OBLIQUE = "../resources/apoa_oblique.yaml"

SCENARIO = {
    pp.ScenarioType.PARALLEL: "parallel",
    pp.ScenarioType.VERTICAL_BASE: "vertical_base",
    pp.ScenarioType.VERTICAL_SINGLE_CAR: "vertical_single_car",
    pp.ScenarioType.OBLIQUE_SLOT: "oblique_slot",
    pp.ScenarioType.PARALLEL_OUT: "parallel slot park out",
    pp.ScenarioType.VERTICAL_BASE_OUT: "vertical base slot park out",
    pp.ScenarioType.VERTICAL_SINGLE_CAR_OUT: "vertical single car slot park out",
    pp.ScenarioType.OBLIQUE_OUT: "oblique slot park out",
}
WIDTH = 1200
HEIGHT = 600
GEO_TYPE[0] = "8"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon
# -

plan_param = pp.PlanParams()
plan_param.scenario_type = pp.ScenarioType.PARALLEL
plan_param.channel_width = 4.0
plan_param.slot_margin = 1.0
plan_param.apa_file = APA_PARALLEL
plan_param.vehicle_param_file = CAR_PARAM_EP

# +
pose_adjuster = pp.PoseAdjuster(plan_param)
odo_str = pose_adjuster.getOdo()

def planCallback(pose):
    search_process_debug=pp.SearchProcessDebug()
    res =  pose_adjuster.planOnce(pose, search_process_debug)
    return (res, search_process_debug.added_edges)
# -




# +
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool
from bokeh.io import output_notebook, push_notebook,output_file,export_png
from bokeh.layouts import layout, column
from bokeh.plotting import figure, output_file, show
from plotter.composite_layers import PerfPlotLayer
from plotter.event_tools import MeasureTools
fig = bkp.figure(title="pose adjuster",
                 x_axis_label='x',
                 y_axis_label='y',
                 match_aspect=True,
                 width=WIDTH,
                 height=HEIGHT)
MeasureTools(fig)
fig.title.align = 'center'
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
output_notebook()

from plotter.layer_pose_adjuster import PoseAdjusterLayer

p1 = pp.Point2d(8.5, 8.30, 0.0)   # default pose
pose_adjuster_plot = PoseAdjusterLayer(fig, odo_str, p1)
pose_adjuster_plot.setSlider(pose_adjuster.getMinX(), pose_adjuster.getMinY(), pose_adjuster.getMaxX(), pose_adjuster.getMaxY())
pose_adjuster_plot.setPlanCallback(planCallback)
pose_adjuster_plot.trigger()

fig.legend.click_policy="hide"

fig_handle = bkp.show(layout(fig), notebook_handle=True)
pose_adjuster_plot.showWidgets()

push_notebook()
# +
xs = pose_adjuster_plot.sbp_res.x[::-1]
ys = pose_adjuster_plot.sbp_res.y[::-1]

ks = getCurvature(xs, ys)
fig2 = bkp.figure(title="curvature with spiral planner",
                 x_axis_label='path sample pose',
                 y_axis_label='kappa',
#                  match_aspect=True,
                 width=WIDTH,
                 height=HEIGHT)
# MeasureTools(fig)
fig2.title.align = 'center'
fig2.toolbar.active_scroll = fig.select_one(WheelZoomTool)
output_notebook()
fig2.line(range(len(ks)),ks,line_width= 3, line_color = 'blue', alpha = 0.5)


# bkp.show(layout(fig2), notebook_handle=True)
# -


