# +
from bokeh.plotting import figure, curdoc
from bokeh.events import PointEvent, Tap
from bokeh.io import output_file, show
from bokeh.layouts import row
from bokeh.io import output_file, show, output_notebook, push_notebook
from bokeh.models import WheelZoomTool, Title

import sys, os
sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan

ego_pose = apa_plan.Point2d(0,0,0)
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
CAR_TYPE = 'LCAR'
CAR_PARAMS = {
    'BCAR':"../resources/vehicle_param_bcar.yaml",
    'LCAR':"../resources/vehicle_param_l.yaml",
    'ETCAR':"../resources/vehicle_param_etcar.yaml"
}
CALIB_PATHS = {
    'BCAR':"/home/ros/Downloads/apa_bag/car_calib/bcar.xlsx",
    'LCAR':"/home/ros/Downloads/apa_bag/car_calib/lacar.xlsx",
    'ETCAR':"/home/ros/Downloads/apa_bag/car_calib/ETcar_9871.xlsx"
}
SHEET_NAME = {
    'BCAR':"20220918",
    'LCAR':"Sheet1",
    'ETCAR':"ET-9871"
}

CAR_PARAM = CAR_PARAMS[CAR_TYPE]
calib_path = CALIB_PATHS[CAR_TYPE]
sheet_name = SHEET_NAME[CAR_TYPE]
print("sheet name:{}".format(sheet_name))
apa_plan.initSingletonParams(CAR_PARAM, APA_PARALLEL)

params_list=[
#     ('raw_shape', 0.0, 0.0, 'red'),
#     ('rect', 0.0, 0.0, 'red'),
# #     ('rect', 0.12, 0.25, 'red'),
#     ('raw inflation', 0.1, 0.25, 'pink'),
#     ('wheel', 0.0, 0.0, 'green'),
    ('wheel', 0.12, 0.25, 'green'),
    ('rotate', 0.17, 0.30, 'blue'),
    ('raw_shape', 0.0, 0.0, 'green'),
    ('rect', 0.0, 0.0, 'pink'),
# #     ('rect', 0.12, 0.25, 'red'),
#     ('raw inflation', 0.1, 0.25, 'pink'),
#     ('wheel', 0.0, 0.0, 'green'),
#     ('wheel', 0.12, 0.25, 'green'),
#     ('rotate', 0.17, 0.30, 'blue'),
#     ('octagon', 0.17, 0.30, 'blue'),
    ('octagon', 0.0, 0.0, 'red')
]
fig = figure(title="simple line example",   # Step 2 
           match_aspect=True,
           x_axis_label='x',
           y_axis_label='y',
           width=1200,
           height=800)
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)


ego_pose = apa_plan.Point2d(0,0,0)

csg = apa_plan.CollisionShapeGenerator()

for params in params_list:
    if params[0] == 'raw_shape' or params[0] == 'raw inflation':
        shape_points = csg.getRawShape(ego_pose, params[1], params[2])
    elif params[0] == 'wheel':
        shape_points = csg.getWheelBaseShape(ego_pose, params[1], params[2])
        raw_xs = [p.x for p in shape_points]
        raw_ys = [p.y for p in shape_points]
        print(raw_xs, raw_ys)
    elif params[0] == 'rotate':
        shape_points = csg.getRotateShape(ego_pose, params[1], params[2])
        
    elif params[0] == 'rect':
        shape_points = csg.getRectShape(ego_pose, params[1], params[2])
    elif params[0] == "octagon":
        shape_points = csg.getOctagonShape(ego_pose, params[1], params[2])
        for sp in shape_points:
            print("[{},{}]".format(sp.x, sp.y))
    else:
        continue
    raw_xs = [sp.x for sp in shape_points]
    raw_ys = [sp.y for sp in shape_points]
    legend = "{}:lat={} lon={}".format(params[0], params[1], params[2])
    poly = fig.multi_polygons([[[raw_xs]]], [[[raw_ys]]], legend_label=legend, fill_color=None, color = params[3], line_width=4, alpha = 0.6)
fig.legend.click_policy="hide"

output_notebook()          
show(fig)    
# -


