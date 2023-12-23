import math
import numpy.linalg as LA
import numpy as np
from plotter.utils.common_data import RECTANGLE_POINTS, HEXAGON_POINTS, TETRADECAGON_POINTS, OCTAGON_POINTS

GEO_TYPE = ["4"]

corner_points_dict = {
    "4": RECTANGLE_POINTS,
    "6": HEXAGON_POINTS,
    "8": OCTAGON_POINTS,
    "14": TETRADECAGON_POINTS
}


def rotate(x, y, theta):
  x_rotated = x * math.cos(theta) - y * math.sin(theta)
  y_rotated = x * math.sin(theta) + y * math.cos(theta)
  return x_rotated, y_rotated

def local2global(x, y, ox, oy, otheta):
    tx, ty = rotate(x, y, otheta)
    return (tx+ox, ty+oy)

def global2local(x, y, ox, oy, otheta):
    x1 = x - ox
    y1 = y - oy
    tx, ty = rotate(x1, y1, -otheta)
    return (tx, ty)

def calcDisPointToLine(point, line_point1, line_point2):
    #对于两点坐标为同一点时,返回点与点的距离
    if line_point1 == line_point2:
        point_array = np.array(point )
        point1_array = np.array(line_point1)
        return np.linalg.norm(point_array -point1_array )
    #计算直线的三个参数
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    #根据点到直线的距离公式计算距离
    distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A**2 + B**2))
    return distance

def getHexagonCorners(car_x, car_y, car_theta, car_param):
    points_dict = corner_points_dict[GEO_TYPE[0]]
    #  TODO: access car type
    car_type = "ep" if car_param['length'] > 5.0 else "rx5"
    local_points =  points_dict[car_type]
    global_points = [local2global( *p, car_x, car_y, car_theta) for p in local_points]
    
    gxx = [p[0] for p in global_points]
    gyy = [p[1] for p in global_points]    
    
    return (gxx, gyy)

def getObstacleCorner(obs_x, obs_y, obs_theta, obs_shape):
    obs_front_length = obs_shape.length/2
    obs_back_length = obs_front_length
    obs_width = obs_shape.width

    obs_corners_data = [[obs_front_length, obs_width / 2],
                        [-obs_back_length, obs_width / 2],
                        [-obs_back_length, -obs_width / 2],
                        [obs_front_length, -obs_width / 2]]

    for corner in obs_corners_data:
        corner[0], corner[1] = rotate(corner[0], corner[1], obs_theta)
        corner[0] += obs_x
        corner[1] += obs_y

    obs_xx = [item[0] for item in obs_corners_data]
    obs_yy = [item[1] for item in obs_corners_data]
    obs_xx.append(obs_corners_data[0][0])
    obs_yy.append(obs_corners_data[0][1])

    return obs_xx, obs_yy


def getCarCorner(car_x, car_y, car_theta, car_param):
    if GEO_TYPE[0] != "4":
        return getHexagonCorners(car_x, car_y, car_theta, car_param)
    
    car_front_length = car_param['front_edge_to_rear_real']
    car_back_length = car_param['length'] - car_front_length
    car_width = car_param['width']

    car_corners_data = [[car_front_length, car_width / 2],
                        [-car_back_length, car_width / 2],
                        [-car_back_length, -car_width / 2],
                        [car_front_length, -car_width / 2]]

    for corner in car_corners_data:
        corner[0], corner[1] = rotate(corner[0], corner[1], car_theta)
        corner[0] += car_x
        corner[1] += car_y

    car_xx = [item[0] for item in car_corners_data]
    car_yy = [item[1] for item in car_corners_data]
    car_xx.append(car_corners_data[0][0])
    car_yy.append(car_corners_data[0][1])

    return car_xx, car_yy

def convertBox(x, y, theta, length, width):
    corners = [
        [length/2.0, width/2.0],
        [- length/2.0, width/2.0],
        [- length/2.0, - width/2.0],
        [length/2.0, - width/2.0]
    ]

    xs = []
    ys = []
    for local_p in corners:
        global_p = rotate(local_p[0], local_p[1], theta)
        xs.append(global_p[0] + x)
        ys.append(global_p[1] + y)
    
    return (xs, ys)


def combineDict(dict1, dict2):
    if dict2 is None:
        return dict1
    
    if dict1 is None:
        return dict2
    
    new_dict = dict(dict2.items()-dict1.items())
    for k,v in dict1.items():
        new_dict[k] = dict2[k] if k in dict2 else v
    
    return new_dict


import asyncio

class Timer:
    def __init__(self, timeout, callback):
        self._timeout = timeout
        self._callback = callback

    async def _job(self):
        await asyncio.sleep(self._timeout)
        self._callback()

    def start(self):
        self._task = asyncio.ensure_future(self._job())

    def cancel(self):
        self._task.cancel()

def debounce2(wait):
    """ Decorator that will postpone a function's
        execution until after `wait` seconds
        have elapsed since the last time it was invoked. """
    def decorator(fn):
        timer = None
        def debounced(*args, **kwargs):
            nonlocal timer
            def call_it():
                fn(*args, **kwargs)
            if timer is not None:
                timer.cancel()
            timer = Timer(wait, call_it)
            timer.start()
        return debounced
    return decorator

from threading import Timer


def debounce(wait):
    """ Decorator that will postpone a functions
        execution until after wait seconds
        have elapsed since the last time it was invoked. """
    def decorator(fn):
        def debounced(*args, **kwargs):
            def call_it():
                fn(*args, **kwargs)
            try:
                debounced.t.cancel()
            except(AttributeError):
                pass
            debounced.t = Timer(wait, call_it)
            debounced.t.start()
        return debounced
    return decorator

def getReverseTime(xs, ys):
    reverse_num = 0
    if len(xs) != len(ys):
        return reverse_num

    if len(xs) < 3:
        return reverse_num

    dx = [xs[i+1] -xs[i] for i in range(len(xs)-1)]
    dy = [ys[i+1] -ys[i] for i in range(len(ys)-1)]

    dx = []
    dy = []
    for i in range(len(xs)-1):
        # avoid two same points in planning path
        if math.hypot(xs[i+1] -xs[i], ys[i+1] -ys[i]) < 1e-6:
            continue
        
        dx.append(xs[i+1] -xs[i])
        dy.append(ys[i+1] -ys[i])


    for i in range(len(dx)-1):
        if dx[i] * dx[i+1] + dy[i] * dy[i+1] < 0.0:
            reverse_num += 1
    
    return reverse_num

def getReversePointIndexs(xs, ys):
    idx_list = list()

    if len(xs) != len(ys):
        return idx_list

    if len(xs) < 3:
        return idx_list

    dx = [xs[i+1] -xs[i] for i in range(len(xs)-1)]
    dy = [ys[i+1] -ys[i] for i in range(len(ys)-1)]

    dx = []
    dy = []
    for i in range(len(xs)-1):
        # avoid two same points in planning path
        if math.hypot(xs[i+1] -xs[i], ys[i+1] -ys[i]) < 1e-6:
            continue
        
        dx.append(xs[i+1] -xs[i])
        dy.append(ys[i+1] -ys[i])

    for i in range(len(dx)-1):
        if dx[i] * dx[i+1] + dy[i] * dy[i+1] < 0.0:
            idx_list.append(i)
    
    return idx_list

def filterNone(sbp_res):
    rx = []
    ry = []
    rphi = []
    if sbp_res is None:
        return (rx, ry, rphi)

    
    for i in range(len(sbp_res.x)):
        if sbp_res.x[i] is None:
            continue
        
        if sbp_res.y[i] is None:
            continue
        
        if sbp_res.phi[i] is None:
            continue
        
        rx.append(sbp_res.x[i])
        ry.append(sbp_res.y[i])
        rphi.append(sbp_res.phi[i])
    
    return (rx, ry, rphi)

def PJcurvature(x,y):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    t_a = LA.norm([x[1]-x[0],y[1]-y[0]])
    t_b = LA.norm([x[2]-x[1],y[2]-y[1]])
    
    M = np.array([
        [1, -t_a, t_a**2],
        [1, 0,    0     ],
        [1,  t_b, t_b**2]
    ])

    a = np.matmul(LA.inv(M),x)
    b = np.matmul(LA.inv(M),y)

    kappa = 2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)
    return kappa, [b[1],-a[1]]/np.sqrt(a[1]**2.+b[1]**2.)


def getCurvature(xs, ys):
    pxs = [xs[0]]
    pys = [ys[0]]

    for i in range(1,len(xs)):
        if  math.hypot(xs[i] -pxs[-1], ys[i] -pys[-1]) < 1e-6:
            continue
        pxs.append(xs[i])
        pys.append(ys[i])
        

    ks = []
    for i in range(1,len(pxs)-1):
        try:
            k, n = PJcurvature([pxs[i-1],pxs[i], pxs[i+1]], [pys[i-1],pys[i], pys[i+1]])
        except Exception as exp:
            continue

        if abs(k) < 1.0:
            ks.append(k)
    
    return ks