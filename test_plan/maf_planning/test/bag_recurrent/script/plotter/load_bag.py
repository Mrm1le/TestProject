from abc import ABC, abstractmethod
from json.tool import main
from unicodedata import name
import rosbag
import math
import numpy as np
import json
from easydict import EasyDict

import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool
from bokeh.io import output_notebook, push_notebook,output_file,export_png
from bokeh.layouts import layout, column
from bokeh.plotting import figure, output_file, show, ColumnDataSource

from plotter.utils.common import local2global, getCarCorner


GROUND_LINE = "/perception/fusion/ground_line"
FUSION_OBJECT = "/perception/fusion/object_parking_environment"
SLOT = "/worldmodel/parking_slot_info"
SBP_REQUEST = "/msd/sbp_request"
EGO_POSE_TPC = "/mla/egopose"

USS = "/uss_distance_marker"
PERCEPTION_OBJECT = "/perception/vision/object"

    
def loadCarParams(data_loader, sbp_topic):
    car_params = {
        'length':4.0,
        'width':2.0,
        'to_center': 2.0
    }
    sbp_datas = data_loader.getTopic(sbp_topic)
    if len(sbp_datas) == 0:
        return car_params
    
    for sbp_data in sbp_datas:
        if sbp_data[0].task_info.goal_id.id != 'search_problem':
            continue
        
        param_string = sbp_data[0].task_config.problem_config.params_string
        param_json = json.loads(param_string)

        if 'VehicleParam' not in param_json:
            return car_params
        
        car_params['length'] = param_json['VehicleParam']['length']
        car_params['width'] = param_json['VehicleParam']['width']
        car_params['to_center'] = param_json['VehicleParam']['front_edge_to_center']
        break

    return car_params


def getBoxCorners(cx, cy, theta, length, width):
    half_length = length / 2.0
    half_width = width / 2.0
    cos_heading = math.cos(theta)
    sin_heading = math.sin(theta)

    dx1 = cos_heading * half_length
    dy1 = sin_heading * half_length
    dx2 = sin_heading * half_width
    dy2 = -cos_heading * half_width

    xs = [
        cx + dx1 + dx2,
        cx + dx1 - dx2,
        cx - dx1 - dx2,
        cx - dx1 + dx2
    ]

    ys = [
        cy + dy1 + dy2,
        cy + dy1 - dy2,
        cy - dy1 - dy2,
        cy - dy1 + dy2
    ]

    return (xs, ys)



class DataGeneratorBase(ABC):
    def __init__(self, xys, ts, accu = False):
        self.xys = xys 
        self.ts = np.array(ts)
        
        self.accu = accu
    
    def getMinT(self):
        if len(self.ts)>0:
            return self.ts[0]
        else:
            return float('inf')
    
    def getMaxT(self):
        if len(self.ts)>0:
            return self.ts[-1]
        else:
            return -1.0
    
    def atTa(self, T):
        if len(self.xys[0]) == 0:
            return [[],[]]
        if self.getMinT() > T:
            return (self.xys[0][0:1],self.xys[1][0:1])
            
        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return (self.xys[0],self.xys[1])
        
        return (self.xys[0][0: first_larger_index[0]],self.xys[1][0:first_larger_index[0]])
    
    def atT(self, T):
        if self.accu:
            return self.atTa(T)
        if len(self.xys) == 0:
            return [[],[]]
        if self.getMinT() > T:
            return self.xys[0]
            
        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return self.xys[-1]
        return self.xys[first_larger_index[0] - 1]

    
        

class UssGenerator(DataGeneratorBase):
    def __init__(self, uss_data, ego_data, uss_config_folder, max_display_dis = 5.0):
        self.max_display_dis = max_display_dis
        self.arc_xys = []
        self.label_xys = []
        xys, ts = self._convert(uss_data, ego_data, uss_config_folder)
        super().__init__(xys, ts)
        
    
    def _convert(self, uss_data, ego_data, uss_config_folder):
        uss_config = UssConfig(uss_config_folder)
        files = uss_config.getFileList(uss_config_folder)
        uss_datas = uss_config.getUssDatas(files)
        
        
        ts = []
        xys = []

        ego_l = len(ego_data)
        uss_l = len(uss_data)
        ego_s = 0
        uss_s = 0
        
        if ego_l <1:
            return (xys, ts)
        
        next_t = ego_data[1][1] if ego_l >1 else ego_data[0][1]
        while ego_s < ego_l and uss_s < uss_l:
            while uss_data[uss_s][1] > next_t and ego_s < ego_l-1 :
                ego_s += 1
                next_t = ego_data[ego_s][1]
            
            ts.append(uss_data[uss_s][1])
            cx = ego_data[ego_s][0].position.position_local.x
            cy = ego_data[ego_s][0].position.position_local.y
            ctheta = ego_data[ego_s][0].orientation.euler_local.yaw
            
            diss = [sonar.upadistance for sonar in uss_data[uss_s][0].uss_upa_distance.sonar]
            
            xs = []
            ys = []
            
            arc_xs = []
            arc_ys = []
            arc_min_rs = []
            arc_max_rs = []
            arc_rs = []
            
            label_xs = []
            label_ys = []
            label_texts = []
            
            
            for i in range(len(diss)):
                
                ox, oy= local2global(uss_datas[i].x, uss_datas[i].y, cx, cy,ctheta)
                # arc_xs.extend([ox, ox])
                # arc_ys.extend([oy, oy])
                # arc_max_rs.extend([uss_datas[i].max_angle + ctheta, uss_datas[i].max_angle + ctheta])
                # arc_min_rs.extend([uss_datas[i].min_angle + ctheta, uss_datas[i].min_angle + ctheta])
                # arc_rs.extend([uss_datas[i].min, uss_datas[i].max])
                
                
                if diss[i] > self.max_display_dis  or  diss[i] < 0.01:
                    continue
                mid_ps = uss_datas[i].interpolateDis(diss[i])
                x0, y0 = local2global(mid_ps[0].x, mid_ps[0].y, cx, cy,ctheta)
                x1, y1 = local2global(mid_ps[1].x, mid_ps[1].y, cx, cy,ctheta)
                
                
                arc_xs.append(ox)
                arc_ys.append(oy)
                arc_max_rs.append(uss_datas[i].max_angle + ctheta)
                arc_min_rs.append(uss_datas[i].min_angle + ctheta)
                arc_rs.append(diss[i])
                
                label_xs.append((x0+x1)/2.0)
                label_ys.append((y0+y1)/2.0)
                label_texts.append(("%.3f"% diss[i]))

                
                # xs.append([x0, x1])
                # ys.append([y0, y1])
            
            xys.append((xs, ys))
            self.arc_xys.append((arc_xs, arc_ys, arc_min_rs, arc_max_rs, arc_rs))
            self.label_xys.append((label_xs, label_ys, label_texts))
            uss_s += 1
        
        if len(xys) ==0:
            xys = [([], [])]
            self.arc_xys = [([],[],[],[],[])]
            self.label_xys = [([],[],[])]
        return (xys, ts)


class PathStopLineGeneratore(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        ts = []
        xys = []
        
        for d in data:
            # if len(d[0].trajectory.path) == 0:
            #     continue
            
            ts.append(d[1])
            xs = []
            ys = []
            for p in d[0].trajectory.path:
                xs.append(p.position_enu.x)
                ys.append(p.position_enu.y)
            
            if len(xs) < 12:
                xys.append(([], []))
                continue
            
            path_theta = math.atan2(ys[-12]-ys[-11], xs[-12] - xs[-11])
            xys.append((
                [math.cos(path_theta+math.pi/2.0) + xs[-11], math.cos(path_theta - math.pi/2.0) + xs[-11]],
                [math.sin(path_theta+math.pi/2.0)  + ys[-11], math.sin(path_theta - math.pi/2.0)  + ys[-11]]
            ))
        
        if len(xys) == 0:
            xys = [([],[])]
                
        return (xys,  ts)
class PlanningPathGeneratore(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        ts = []
        xys = []
        
        for d in data:
            # if len(d[0].trajectory.path) == 0:
            #     continue
            
            ts.append(d[1])
            xs = []
            ys = []
            for p in d[0].trajectory.path:
                xs.append(p.position_enu.x)
                ys.append(p.position_enu.y)
            
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]
                
        return (xys,  ts)

class UssControlGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts, accu = True)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        ts = []
        xys = []

        for d in data:
            ts.append(d[1])
            try:
                param_string = d[0].extra.json
                param_json = json.loads(param_string)
                if param_json['replanning_flag']:
                    car_color = ['black']
                elif param_json['is_limited_']:
                    car_color = ['red'] 
                else:
                    car_color = ['green']
                if(abs(param_json['x_main_ego_used_for_debug']) < 0.01 and abs(param_json['x_main_obs_used_for_debug']) < 0.01 and abs(param_json['y_main_ego_used_for_debug'])< 0.01 and abs(param_json['y_main_obs_used_for_debug'])< 0.01):
                    x_main_obstacle= []
                    y_main_obstacle = []
                else:
                    x_main_obstacle = [param_json['x_main_ego_used_for_debug'], param_json['x_main_obs_used_for_debug']]
                    y_main_obstacle = [param_json['y_main_ego_used_for_debug'], param_json['y_main_obs_used_for_debug']]

                debug_string = ' s_uss: ' + str(param_json['remain_s_uss']) + ' s_plan: ' + str(param_json['remain_s_plan']) +' vel: ' + str(param_json['velocity_mps'])

                debug_list = [debug_string]
            except:
                car_color = ['green']
                x_main_obstacle = []
                y_main_obstacle = []
                debug_list = ['null']

            xys.append((debug_list, x_main_obstacle, y_main_obstacle, car_color))

        if len(xys) == 0:
            xys = [(['null'],[],[],['green'])]

        return (xys,  ts)

class EgoTargetGenerator(DataGeneratorBase):
    def __init__(self, data, car_params):
        xys, ts = self._convert(data,car_params)
        super().__init__(xys, ts)
    
    def _convert(self, sbp_datas, car_params):
        if sbp_datas is None or len(sbp_datas) == 0:
            return [[], []]
        
        ts = []
        xys = []
        for d in sbp_datas:
            if d[0].task_info.goal_id.id != 'search_problem':
                continue
            
            param_string = d[0].task_config.problem_config.params_string
            param_json = json.loads(param_string)

            if 'OpenspaceDeciderOutput' not in param_json:
                continue
            
            odo = EasyDict(param_json['OpenspaceDeciderOutput'])
            ts.append(d[1])
            
            cx = odo.init_state.path_point.x
            cy = odo.init_state.path_point.y
            ctheta = odo.init_state.path_point.theta
            
            car_params_raw = {
                'front_edge_to_rear_real': car_params['to_center'],
                'length': car_params['length'],
                'width' : car_params['width']
            }

            xs, ys = getCarCorner(cx, cy, ctheta, car_params_raw)
            xys.append(([[[xs]]], [[[ys]]]))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys,  ts)

class EgoGenerator(DataGeneratorBase):
    def __init__(self, data, car_params):
        xys, ts = self._convert(data,car_params)
        super().__init__(xys, ts)
    
    def _convert(self, data, car_params):
        if data is None:
            return [[], []]
        
        ts = []
        xys = []
        for d in data:
            ts.append(d[1])
            cx = d[0].position.position_local.x
            cy = d[0].position.position_local.y
            ctheta = d[0].orientation.euler_local.yaw
            
            car_params_raw = {
                'front_edge_to_rear_real': car_params['to_center'],
                'length': car_params['length'],
                'width' : car_params['width']
            }

            xs, ys = getCarCorner(cx, cy, ctheta, car_params_raw)
            xys.append(([[[xs]]], [[[ys]]]))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys,  ts)

class EgoPathGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts, accu = True)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        ts = []
        xs = []
        ys = []
        for d in data:
            ts.append(d[1])
            cx = d[0].position.position_local.x
            cy = d[0].position.position_local.y
            
            xs.append(cx)
            ys.append(cy)

        xys = (xs, ys)

        return (xys,  ts)

class MpcGenerator(DataGeneratorBase):
    def __init__(self, ego_data, car_params, data):
        self.label_xys = []
        xys, ts = self._convert(ego_data, car_params, data)
        super().__init__(xys, ts)
    
    def _convert(self, ego_data, car_params, data):
        if data is None:
            return [[], []]
        
        ts = []
        xys = []
        ego_index = 0
        self.label_xys = []

        for d in data:
            ts.append(d[1])
            xs = []
            ys = []
            label_xs = []
            label_ys = []
            label_texts = []
            collide_to_wheel_stop = False
            while ego_index + 1 < len(ego_data) and ego_data[ego_index + 1][1] < d[1]:
                ego_index += 1
            ego_x = ego_data[ego_index][0].position.position_local.x
            ego_y = ego_data[ego_index][0].position.position_local.y
            ego_theta = ego_data[ego_index][0].orientation.euler_local.yaw

            for (idx, p) in enumerate(d[0].mpc_trajectory.path_points):
                px, py= local2global(p.position_enu.x + car_params['to_center'], p.position_enu.y, ego_x, ego_y, ego_theta)
                if (idx == 0):
                    label_xs.append(px)
                    label_ys.append(py)

                if (idx > 25):
                    if p.position_enu.x > 0:
                        collide_to_wheel_stop = True
                    break
                xs.append(px)
                ys.append(py)
            
            xys.append((xs, ys))
            if collide_to_wheel_stop:
                label_texts.append(("collide_to_wheel_stop"))
            else:
                label_texts.append((""))
            self.label_xys.append((label_xs, label_ys, label_texts))

        return (xys,  ts)

class SlotGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)

    def _convert(self, data):
        if data is None:
            return ([],[])

        xys = []
        ts = []
        for frame in data:
            ts.append(frame[1])
            slot_data = frame[0].parking_slots

            xs = []
            ys = []
            for slot in slot_data:
                points = slot.parking_slot.local_points_fusion
                for i in range(len(points)-1):
                    xs.append([points[i].x, points[i+1].x])
                    ys.append([points[i].y, points[i+1].y])
            
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]
        
        return (xys,  ts)

class WheelStopGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)

    def _convert(self, data):
        if data is None:
            return ([],[])

        xys = []
        ts = []
        for frame in data:
            ts.append(frame[1])
            xs = []
            ys = []
            for wheel_stop_json in frame[0].reserved_info:
                wheel_stop_data = json.loads(wheel_stop_json)
                if (wheel_stop_data["has_wheel_stop"]):

                    xs.append([wheel_stop_data["wheel_stop_pt1_x"], wheel_stop_data["wheel_stop_pt2_x"]])
                    ys.append([wheel_stop_data["wheel_stop_pt1_y"], wheel_stop_data["wheel_stop_pt2_y"]])
                    
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys,  ts)

class BoxGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        xys = []
        ts = []
        for d in data:
            ts.append(d[1])
            fusion_objects = d[0].perception_fusion_objects_data
            multi_box_corners = [
                getBoxCorners(
                    fb.position.x,
                    fb.position.y,
                    fb.heading_yaw,
                    fb.shape.length,
                    fb.shape.width
                ) for fb in fusion_objects
            ]

            xs = []
            ys = []
            for cs in multi_box_corners:
                xs.append([[cs[0]]])
                ys.append([[cs[1]]])
            
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys, ts)

class PointsGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys, ts)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        xys = []
        ts = []
        for d in data:
            ts.append(d[1])
            points_fusion = d[0].ground_line.ground_line_data
            xs = []
            ys = [] 
            for p_fusion in points_fusion:
                xs.extend([p.x for p in p_fusion.local_points_fusion])
                ys.extend([p.y for p in p_fusion.local_points_fusion])
            
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys, ts)

class UssOdGenerator(DataGeneratorBase):
    def __init__(self, data, park_in_id):
        xys, ts = self._convert(data, park_in_id)
        super().__init__(xys, ts)
    
    def _convert(self, data, park_in_id):
        if data is None:
            return [[], []]
        xys = []
        ts = []
        for d in data:
            ts.append(d[1])
            points_fusion = d[0].ground_line.ground_line_data
            xs = []
            ys = [] 
            for p_fusion in points_fusion:
                if park_in_id > 0 and p_fusion.track_id != park_in_id:
                    continue
                xs.extend([p.x for p in p_fusion.local_points_fusion])
                ys.extend([p.y for p in p_fusion.local_points_fusion])
            
            xys.append((xs, ys))
        
        if len(xys) == 0:
            xys = [([],[])]

        return (xys, ts)

class DataLoader:
    def __init__(self, bag: str):
        self.bag = rosbag.Bag(bag)

    def getTopic(self, topic):
        res = []
        try:
            for tpc, msg, t in self.bag.read_messages(topics=topic):
                if tpc == topic:
                    res.append([msg, t.to_sec()])
        except Exception as exp:
            print("faile to load:", topic)

        return res


# uss

class UssPoint2d:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
    def __str__(self):
        return "[{}, {}]".format(self.x, self.y)

class UssData:
    '''
    1 ------------- 4
    |
    2 ------------- 3
    '''
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.max = 5.0
        self.min = 0.15
        self.fov = []
        self.min_angle = 0.0
        self.max_angle = 0.0
    
    def updateFov(self, fov):
        if len(fov) ==0:
            return
        
        self.fov = fov
        
        self.min_angle = math.atan2(self.fov[2].y - self.fov[1].y, self.fov[2].x - self.fov[1].x)
        self.max_angle = math.atan2(self.fov[3].y - self.fov[0].y, self.fov[3].x - self.fov[0].x)
        
    
    def interpolateDis(self, dis):
        
        l_ratio = (dis - self.min) / (self.max-self.min)
        r_ratio = 1-l_ratio
        x0 = self.fov[0].x * r_ratio + self.fov[3].x * l_ratio
        y0 = self.fov[0].y * r_ratio + self.fov[3].y * l_ratio
        x1 = self.fov[1].x * r_ratio + self.fov[2].x * l_ratio
        y1 = self.fov[1].y * r_ratio + self.fov[2].y * l_ratio
        
        return [UssPoint2d(x0, y0), UssPoint2d(x1, y1)]
        
        
    def __str__(self):
        return "x:{}\ty:{}\tyaw:{}\tmax:{}\tmin:{}\t{}".format(self.x, self.y, self.yaw,self.max, self.min, self.fov)
        
class UssConfig:
    def __init__(self, uss_config_folder) -> None:
        self.uss_config_folder = uss_config_folder
    
    
    def getUssDatas(self, file_names):
        uss_datas = []
        for file_name in file_names:
            with open(file_name, mode='r', encoding='utf-8') as uss_file:
                one_line = uss_file.readline().rstrip('\n').strip()+"_"
                while  one_line[0] != 'x':
                    one_line = uss_file.readline().rstrip('\n').strip()+"_"
                
                uss_data = UssData()
                uss_data.x = float(one_line[3:-1])
                one_line = uss_file.readline().rstrip('\n').strip()
                uss_data.y = float(one_line[3:])
                
                uss_file.readline()
                uss_file.readline()
                
                one_line = uss_file.readline().rstrip('\n').strip()
                uss_data.yaw =  math.radians(float(one_line[5:]))
                
                one_line = uss_file.readline().rstrip('\n').strip()
                uss_data.max = float(one_line[11:])
                one_line = uss_file.readline().rstrip('\n').strip()
                uss_data.min = float(one_line[11:])
                
                
                uss_file.readline()
                uss_file.readline()
                
                one_line = uss_file.readline().rstrip('\n').strip()[6:-1]
                
                fov_points = [float(f.rstrip(',')) for f in one_line.split(" ")]
                
                fov = [UssPoint2d(fov_points[2*i+0], fov_points[2*i+1])  for i in range(4)]
                uss_data.updateFov(fov)
                uss_datas.append(uss_data)
        
        return uss_datas
                
        
    def getFileList(self, uss_config_folder):
        uss_config_file = uss_config_folder + "/uss_set.yaml"
        files = [] 
        with open(uss_config_file, mode='r', encoding='utf-8') as uss_file:
            one_line = uss_file.readline().rstrip('\n').strip()
            while  one_line != "multi_uss_config:":
                one_line = uss_file.readline().rstrip('\n').strip()
            
            if one_line is None:
                return files
            
            one_line = uss_file.readline().rstrip('\n').strip()
            while one_line:
                files.append(uss_config_folder+ "/uss_"+one_line[2:]+".yaml")
                one_line = uss_file.readline().rstrip('\n').strip()
                
        return files

if __name__ == '__main__':
    uss_folder = "../../resources/uss_config"
    uss_config = UssConfig(uss_folder)
    files = uss_config.getFileList(uss_folder)
    uss_datas = uss_config.getUssDatas(files)
    for uss in uss_datas:
        print(uss)
        
    uss_datas[0].interpolateDis(2.5)

        
            
    
        
    

