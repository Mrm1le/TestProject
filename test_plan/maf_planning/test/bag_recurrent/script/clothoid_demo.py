import os
import sys
import json
import numpy as np
import scipy
import math
from easydict import EasyDict
sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as pp

from bokeh.plotting import figure, show
from bokeh.io import output_notebook, push_notebook,output_file
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, Circle

fig = figure(title="",
                x_axis_label='x',
                y_axis_label='y',
                match_aspect=True,
                width=800,
                height=600)
fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)


def circleXY(xo,yo,r,theta_start, theta_end, num):
    theta_r = np.linspace(theta_start, theta_end, num)
    xr = [xo + r * math.sin(t) for t in theta_r]
    yr = [yo - r * math.cos(t) for t in theta_r]

    return xr, yr


# +
cr = 5.015
alpha = 0.2 #0.4913
alpha = 0.3684
alpha = 0.7
alpha = 0.29476
alpha = 0.42  # 0.35
clo_solver = pp.ClothoidSolver(alpha)
clo_p = pp.ClothoidPoint()
clo_p = pp.getPointByRadius(clo_solver, cr)

# fig.line(x,y, line_width = 2, line_color = 'blue')
clo_curve = pp.getCurveByS(clo_solver, clo_p.s, 0.01)


# +
# clo_curve2 = pp.getCurveByS(clo_solver, 50, 0.02)
# xs = [n.x for n in clo_curve2 ]
# ys = [n.y for n in clo_curve2 ]
# axs = [-n.x for n in clo_curve2 ]
# ays = [-n.y for n in clo_curve2 ]
# axs = axs[::-1]
# ays = ays[::-1]
# axs.extend(xs)
# ays.extend(ys)

clo_curve2 = pp.getCurveByS(clo_solver, clo_p.s, 0.01)
xs = [n.x for n in clo_curve2 ]
ys = [n.y for n in clo_curve2 ]
axs = [-n.x for n in clo_curve2 ]
ays = [-n.y for n in clo_curve2 ]
axs = axs[::-1]
ays = ays[::-1]
axs.extend(xs)
ays.extend(ys)

cx, cy, mu = pp.getMuOffset(clo_solver, cr)
xr, yr = circleXY(cx, cy, cr, -4.0*clo_p.theta, 6.0*clo_p.theta, 100)

fig.line(axs, ays, line_width = 2, line_color = 'green', legend = 'Clothoid')
fig.line(xr, yr, line_width = 2, line_color = 'green', legend = 'circle')
fig.line([0, cx, clo_p.x], [0, cy, clo_p.y], line_width = 2, line_color = 'red', alpha = 0.5)
output_notebook()
show(fig)
# -

print(clo_p.theta * 6.0)
print(5 * (1-math.cos(0.25)))
print(0.4 * math.sin(0.25))

# +
# xs = [n.x for n in clo_curve ]
# ys = [n.y for n in clo_curve ]
# fig.line(xs, ys, line_width = 2, line_color = 'blue', legend_label = 's1')
# clo_s2 = pp.transformS2Py(clo_curve)
# xs = [n.x for n in clo_s2 ]
# ys = [n.y for n in clo_s2 ]
# fig.line(xs, ys, line_width = 2, line_color = 'green', legend_label = 's2')
# clo_s3 = pp.transformS3Py(clo_curve)
# xs = [n.x for n in clo_s3 ]
# ys = [n.y for n in clo_s3 ]
# fig.line(xs, ys, line_width = 2, line_color = 'red', legend_label = 's3', alpha = 0.5)
# clo_s4 = pp.transformS4Py(clo_curve)
# xs = [n.x for n in clo_s4 ]
# ys = [n.y for n in clo_s4 ]
# fig.line(xs, ys, line_width = 2, line_color = 'orange', legend_label = 's4', alpha = 0.5)


# clo_s2 = pp.transform2DPy(clo_s2, clo_curve[-1].x, clo_curve[-1].y, 2 * clo_curve[-1].theta)
# xs = [n.x for n in clo_s2 ]
# ys = [n.y for n in clo_s2 ]
# fig.line(xs, ys, line_width = 2, line_color = 'green', legend_label = 's2')
# clo_s3 = pp.transform2DPy(clo_s3, clo_s2[-1].x, clo_s2[-1].y, 2 * clo_curve[-1].theta)
# xs = [n.x for n in clo_s3 ]
# ys = [n.y for n in clo_s3 ]
# fig.line(xs, ys, line_width = 2, line_color = 'red', legend_label = 's3', alpha = 0.5)
# clo_s4 = pp.transform2DPy(clo_s4,clo_s3[-1].x, clo_s3[-1].y, 0)
# xs = [n.x for n in clo_s4 ]
# ys = [n.y for n in clo_s4 ]
# fig.line(xs, ys, line_width = 2, line_color = 'orange', legend_label = 's4', alpha = 0.5)

# output_notebook()
# show(fig)

# -





[n.s for n in clo_curve ]
print(cx, cy, mu)

math.atan2(cx, cy)

# +

cx, cy, mu1 = pp.getMuOffset(clo_solver, 5.015)
cr1 = math.hypot(cx, cy)
v1 = cr1*math.cos(mu1)
p1 = cr1*math.sin(mu1)
cx, cy, mu2 = pp.getMuOffset(clo_solver, 10.0)
cr2 = math.hypot(cx, cy)
v2 = cr2*math.cos(mu2)
p2 = cr2*math.sin(mu2)
print(cr1, cr2)
print(v1, p1, v2, p2)
# -

print(math.sqrt((cr1+cr2)*(cr1+cr2) - (v1+v2)*(v1+v2)))
print(p1+p2, mu1, mu2)

1/5.015 / (460.0/340 * 0.3)

clo_p.theta 

lo_s2 = pp.transformS2Py(clo_curve)
xs = [n.x for n in clo_s2 ]
ys = [n.y for n in clo_s2 ]
thes = [n.theta for n in clo_s2 ]
print(xs)
print(ys)
print(thes)

print(clo_p.x, clo_p.y, clo_p.theta)


