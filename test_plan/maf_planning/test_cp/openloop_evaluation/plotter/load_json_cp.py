from abc import ABC, abstractmethod
from json.tool import main
from unicodedata import name
# import rosbag
import math
import numpy as np
import json

from easydict import EasyDict

import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool
from bokeh.io import output_notebook, push_notebook,output_file,export_png
from bokeh.layouts import layout, column
from bokeh.plotting import figure, output_file, show, ColumnDataSource


car_params = {
        'length':    4.98,
        'width':     2.11,
        'to_center': 2.5
    }


GEO_TYPE = ["4"]


def rotate(x, y, theta):
  x_rotated = x * math.cos(theta) - y * math.sin(theta)
  y_rotated = x * math.sin(theta) + y * math.cos(theta)
  return x_rotated, y_rotated



def getCarCorner(car_x, car_y, car_theta, car_param):

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
    # if car_theta <= 0.0001:
    #     print(corner[0])
    #     print(corner[1])
        
    car_xx = [item[0] for item in car_corners_data]
    car_yy = [item[1] for item in car_corners_data]
    car_xx.append(car_corners_data[0][0])
    car_yy.append(car_corners_data[0][1])

    return car_xx, car_yy

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
    def __init__(self, xys, ts, accu = False, name = "Default Name"):
        self.xys = xys 
        self.ts = np.array(ts)
        
        self.accu = accu
        self.name = name
    
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


class JsonTextGenerator(DataGeneratorBase):
    def __init__(self, data, text_type, local = False):
        xys, ts = self._convert(data,text_type, local)
        super().__init__(xys, ts)
        self.txt = text_type
    
    def _convert(self, data_jsons, text_type, local):        
        ts = []
        xys = []
        
        if text_type == "ego speed":
            for data_frame in data_jsons:
                ts.append(data_frame['time_stamp'])
                data_json = data_frame['path_planner_input']
                speed = round(data_json['planning_init_state']['v'],2)
                speed_text = "Ego Speed: {} m/s".format(speed)

                cx = data_json['planning_init_state']['x']
                cy = data_json['planning_init_state']['y']
                xys.append(([cx+2],[cy-2],[speed_text]))
            
            if len(xys) == 0:
                xys = [([],[],[])]
        elif text_type == "frame number":
            frame_num = 0
            for data_frame in data_jsons:
                ts.append(data_frame['time_stamp'])
                data_json = data_frame['path_planner_input']
                text = "Frame: {}".format(frame_num)

                cx = data_json['planning_init_state']['x']
                cy = data_json['planning_init_state']['y']
                xys.append(([cx+2],[cy-5],[text]))
                frame_num += 1
        else:
            xys = [([],[],[])]
        return (xys,  ts)


class JsonEgoGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys,ts)

    def _convert(self, data_jsons):
        xys = []
        ts = []
        for data_frame in data_jsons:
            ts.append(data_frame['time_stamp'])
            data_json = data_frame['path_planner_input']
            cx = data_json['planning_init_state']['x']
            cy = data_json['planning_init_state']['y']
            ctheta = math.atan(data_json['planning_init_state']['dy_ds']/data_json['planning_init_state']['dx_ds'])

            car_params_raw = {
                'front_edge_to_rear_real': car_params['to_center'],
                'length': car_params['length'],
                'width' : car_params['width']
            }

            xs, ys = getCarCorner(cx, cy, ctheta, car_params_raw)
            xys.append(([[[xs]]], [[[ys]]]))
        
        if len(xys) == 0:
            xys = [([],[])]

        # return (xys,  ts)
        return xys, ts

class JsonRefline(DataGeneratorBase):
    # not ready by 2022-04-22
    def __init__(self, data, refline_name = None, ego_data = None, local = False):
        xys, ts = self._convert(data, refline_name, ego_data, local)
        super().__init__(xys, ts)
    def _convert(self, data_jsons, refline_name, ego_data, local):
        xys = []  
        ts = []

        for data_json in data_jsons:
            ts.append(data_json['header']['stamp']*1e-9)
            # print(ts[-1])
            print("segment number: ",len(data_json['path_segments']))
            for path_segment in data_json['path_segments']:
                print("quad point number: ",len(path_segment['quad_point_info']))

        return xys, ts



class JsonRERealGenerator(DataGeneratorBase):
    def __init__(self, data, lane_name = None, ego_data = None, local = False):
        xys, ts = self._convert(data, lane_name, ego_data, local)
        super().__init__(xys, ts)
    def _convert(self, data_jsons, lane_name, ego_data, local):
        xys = []  
        ts = []
        for converted_data_slice in data_jsons.converted_data:
            ts.append(converted_data_slice.time_stamp)

        if lane_name == None:
            return [([],[],[])], ts
        if lane_name not in data_jsons.real_env.keys():
            return [([],[],[])], ts

        for t in ts:
            xs = []
            ys = []
            rs = []
            if t == ts[-1]:
                for point in data_jsons.real_env[lane_name]:
                    xs.append(point["x"])
                    ys.append(point["y"])
                    rs.append(0.1) 
                
            xys.append((xs,ys,rs))

        if len(xys) == 0:
            xys = [([],[],[])]
        return xys, ts

class JsonLaneGenerator(DataGeneratorBase):
    def __init__(self, data, lane_name = None, ego_data = None, local = False):
        xys, ts = self._convert(data, lane_name, ego_data, local)
        super().__init__(xys, ts)
    def _convert(self, data_jsons, lane_name, ego_data, local):
        xys = []  
        ts = []

        if lane_name == None:
            return [([],[],[])], ts

        for data_json in data_jsons:
            # print(data_json.keys())
            ts.append(data_json["time_stamp"])
            xs = []
            ys = []
            rs = []
            
            for point in data_json[lane_name]:
                xs.append(point["x"])
                ys.append(point["y"])
                rs.append(0.2) 
            xys.append((xs,ys,rs))
            
        if len(xys) == 0:
            xys = [([],[],[])]
        return xys, ts



class JsonObjGenerator(DataGeneratorBase):
    def __init__(self, data, lane_name, local= False):
        xys, ts = self._convert(data, lane_name, local)
        super().__init__(xys,ts)

    def _convert(self, data_jsons, lane_name, local= False):
        if data_jsons is None:
            return [[], []]
        if lane_name == None:
            return [[], []]
        xys = []
        ts = []
        for data_json in data_jsons:
            ts.append(data_json["time_stamp"])
            fusion_objects = data_json[lane_name]
            # print(fusion_objects[0].items())
            if local:
                 multi_box_corners = [
                    getBoxCorners(
                        fb.relative_position.x, fb.relative_position.y,
                        fb.relative_heading_yaw,
                        fb.shape.length,
                        fb.shape.width
                    ) for fb in fusion_objects
                ]               
            else:
                multi_box_corners = [
                    getBoxCorners(
                        fb.position.x, fb.position.y,
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

class JsonAllLanesGenerator(DataGeneratorBase):
    def __init__(self, data, lane_name = None, ego_data = None, local = False):
        xys, ts = self._convert(data, lane_name, ego_data, local)
        super().__init__(xys, ts)
    def _convert(self, data_jsons, lane_name, ego_data, local):
        xys = []  
        ts = []

        if lane_name == None:
            return [([],[])], ts

        for data_json in data_jsons:
            # print(data_json.keys())
            ts.append(data_json["time_stamp"])
            xs = []
            ys = []
            
            for lane in data_json[lane_name]:
                if len(lane)<=1:
                    return [([],[])], ts
                for ii in range(len(lane)-1):
                    xs.append([lane[ii]["x"],lane[ii+1]["x"]])
                    ys.append([lane[ii]["y"],lane[ii+1]["y"]])
                    
            xys.append((xs,ys))
            
        if len(xys) == 0:
            xys = [([],[])]
        return xys, ts


class JsonQuadGenerator(DataGeneratorBase):
    def __init__(self, data, refline_name = None, ego_data = None, local = False):
        xys, ts = self._convert(data, refline_name, ego_data, local)
        super().__init__(xys, ts)
    def _convert(self, data_jsons, refline_name, ego_data, local):
        xys = []  
        ts = []

        for data_json in data_jsons:
            xs = []
            ys = []
            rs = []
            if 'time_stamp' not in data_json.keys():
                    return [([],[],[])], ts
            ts.append(data_json['time_stamp'])
            # print("segment number: ",len(data_json['path_segments']))
            for path_segment in data_json['path_planner_input']['path_segments']:
                # print("quad point number: ",len(path_segment['quad_point_info']))
                for quad_point_info in path_segment['quad_point_info']:
                    xs.append(quad_point_info['refline_info']['x'])
                    ys.append(quad_point_info['refline_info']['y'])
                    rs.append(0.2)
                      
            xys.append((xs,ys,rs))
        
        if len(xys) == 0:
            xys = [([],[],[])]
        return xys, ts


class JsonPlanPathGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts = self._convert(data)
        super().__init__(xys,ts)

    def _convert(self, data_jsons):
        xys = []
        ts = []
        # for index in range(2):
        for data_json_frame in data_jsons:
            # data_json = data_jsons[0]
            xs = []
            ys = []
            rs = []
            temp = []
            # print(t_str, float(t_str))
            ts.append(data_json_frame.time_stamp)
            data_json = data_json_frame.nb_debug
            if 's_to_samples' not in data_json.keys():
                return [([],[],[])], ts
            for s_to_sample in data_json['s_to_samples']:
                if len(s_to_sample) < 2:
                    return [([],[],[])], ts
                if 'sample_data' not in s_to_sample[1]:
                    return [([],[],[])], ts
                sample_data = s_to_sample[1]['sample_data']
                temp0 = [sample_data['t'],sample_data['x'],sample_data['y'],1]
                temp.append(temp0)
            
            temp = sorted(temp,key=(lambda x:x[0]),reverse=False)
            for temp_array in temp:
                xs.append(temp_array[1])
                ys.append(temp_array[2])
                rs.append(temp_array[3])
            xys.append((xs,ys,rs))

        if len(xys) == 0:
            xys = [([],[],[])]
        # print(xys)
        return xys, ts


# -----------------------------------------------------------------------------------------------#


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

        
            
    
        
    

