# -*- coding: utf-8 -*-

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../../lib'))

    
from plotter.load_bag import DataLoader
from batch_plan.common import *

from easydict import EasyDict
import json
import yaml
import genpy
import scipy.signal as ss
import copy
import hashlib
import traceback

import plotter.utils.common as comm

from config import conf
from evaluator import getVecDistance
from evaluator import r2

import py_parking_plotter as pp

EVENT_LABEL_TYPE_MAP = {
    'wall': 'wall',
    'car': 'car',
    'stone_pier': 'wall',
    'water_safety_barrier': 'wall',
    'telegraph_pole': 'wall',
    'cone': 'wall',
    'fire_extinguisher': 'wall',
    'no_parking_sign': 'wall',
    'traffic_pole': 'wall',
    'trash_can': 'wall',
    'locked_parking_lock': 'wall',
    'fence': 'wall',
    'special': 'wall',
    'vehicle': 'car',
    'square_pillar': 'wall',
    'box': 'wall',
    'circular_pillar': 'wall',
    'bicycle': 'wall',
    'multi_bicycle': 'wall',
    'bush': 'wall',
    'hanging_object': 'wall',

    'pillar': 'pillar',

    # 'roadedge': 'roadedge',
    # 'cordon': 'roadedge',
    # 'auxiliary': 'roadedge',
    # 'curbstone': 'roadedge',
    'roadedge': 'wall',
    'cordon': 'wall',
    # 'auxiliary': 'wall',
    'curbstone': 'wall',
}

class EventBase(object):
    def __init__(self):
        self.name = ""
        self.expert_result = None
        self.plan_result = None
        self.odo_data = None
        self.car_size_params = None
        self.init_point = None
        self.target_point = None
        self.begin_time_offset = 0
        self.event_info = None
        self.slot_corners = []
        self.slot_type = pp.ParkingSlotType.UNKNOWN
        self.slot_side = pp.ParkingSlotSide.SIDE_UNKNOWN
        self.wall_data = None
        self.expert_min_distance = 0
        self.plan_min_distance = 0
        self.gt_json_path = None
        self.dt_json_path = None


    def init(self, event_info):
        self.name = event_info['name']
        self.begin_time_offset = event_info['begin_time_offset']
        self.event_info = copy.deepcopy(event_info)

    def setSlotCorners(self, corners):
        for c in corners:
            c_p = pp.Point2d()
            c_p.x = c["x"]
            c_p.y = c["y"]
            self.slot_corners.append(c_p)

    def initFromGt(self, gt_file):
        try:
            with open(gt_file, "r") as f:
                gt = json.load(f)
                self.name = gt["name"]

                self.odo_data = gt["odo"]
                self.odo_data["points2lines"] = gt["points2lines"]
                self.odo_data = EasyDict(self.odo_data)

                expert_result = copy.deepcopy(gt["expert_result"])
                self.expert_result = EasyDict(expert_result)
                self.car_size_params = EasyDict(gt["car_size_params"])

                self.init_point = pp.Point2d()
                self.target_point = pp.Point2d()

                self.init_point.x = gt["init_points"][0]["x"]
                self.init_point.y = gt["init_points"][0]["y"]
                self.init_point.theta = gt["init_points"][0]["theta"]
                self.target_point.x = gt["target_point"]["x"]
                self.target_point.y = gt["target_point"]["y"]
                self.target_point.theta = gt["target_point"]["theta"]

                if gt["slot_type"] == "ParkingSlotType.VERTICAL":
                    self.slot_type = pp.ParkingSlotType.VERTICAL
                elif gt["slot_type"] == "ParkingSlotType.PARALLEL":
                    self.slot_type = pp.ParkingSlotType.PARALLEL

                self.expert_min_distance = gt["min_obs_distance"]
        except:
            print(traceback.format_exc())
            return False

        sbp_res_list, car_size_params = planOnce([self.odo_data], self.slot_type)
        self.plan_result = sbp_res_list[0]
        return True


    def forkReplanEvents(self):
        return list()

    
    def pointToDict(self, point):
        d = dict()
        d["x"] = point.x
        d["y"] = point.y
        d["theta"] = point.theta
        return d


    def printInitResult(self):
        info = 'event init finished. '
        info += 'expert result size: {}, '.format(len(self.expert_result['x']))
        if self.plan_result != None:
            info += 'plan result size: {}, '.format(len(self.plan_result['x']))
        info += 'init point x:{}, y:{}, theta:{}, '.format(
            self.init_point.x, self.init_point.y, self.init_point.theta)
        info += 'target point x:{}, y:{}, theta:{}, '.format(
            self.target_point.x, self.target_point.y, self.target_point.theta)

        print(info)


    def generateGtDtJson(self, replan=False):
        if not conf['common']['generate_jsons']:
            return

        if not os.path.isdir(conf['common']['output_html']):
            os.makedirs(conf['common']['output_html'])
        
        dump_data = dict()

        if self.gt_json_path == None:
            self.gt_json_path = '{}/{}_gt.json'.format(
                conf['common']['output_html'],
                self.name.replace(' ', '_').replace(':', '_'))

        if self.dt_json_path == None:
            self.dt_json_path = '{}/{}_dt.json'.format(
                conf['common']['output_html'],
                self.name.replace(' ', '_').replace(':', '_'))

        if replan:
            if os.path.exists(self.gt_json_path):
                with open(self.gt_json_path, 'r') as f:
                    dump_data = json.load(f)

                with open(self.gt_json_path, 'w') as f:
                    dump_data["init_points"].append(self.pointToDict(self.init_point))
                    json.dump(dump_data, f)

            if os.path.exists(self.dt_json_path):
                with open(self.dt_json_path, 'r') as f:
                    dump_data = json.load(f)

                with open(self.dt_json_path, 'w') as f:
                    dump_data["plan_results"].append(json.loads(json.dumps(self.plan_result)))
                    dump_data["min_obs_distances"].append(self.expert_min_distance)
                    json.dump(dump_data, f)
            return

        dump_data = dict()

        with open(self.gt_json_path, 'w') as f:

            dump_data["name"] = self.name

            dump_data["odo"] = json.loads(json.dumps(self.odo_data))
            
            if "points2lines" in dump_data["odo"]:
                dump_data["points2lines"] = dump_data["odo"]["points2lines"]
                dump_data["odo"].pop("points2lines")
            
            dump_data["slot_type"] = str(self.slot_type)

            dump_data["car_size_params"] = self.car_size_params

            dump_data["init_points"] = list()
            dump_data["init_points"].append(self.pointToDict(self.init_point))

            dump_data["target_point"] = self.pointToDict(self.target_point)

            dump_data["expert_result"] = json.loads(json.dumps(self.expert_result))

            dump_data["min_obs_distance"] = self.expert_min_distance
            json.dump(dump_data, f)

        
        dump_data = dict()
        
        with open(self.dt_json_path, 'w') as f:

            dump_data["name"] = self.name

            dump_data["plan_results"] = list()
            dump_data["plan_results"].append(json.loads(json.dumps(self.plan_result)))

            dump_data["min_obs_distances"] = list()
            dump_data["min_obs_distances"].append(self.expert_min_distance)

            json.dump(dump_data, f)


    def setMd5(self, file_path):
        if file_path == None:
            return
            
        if not os.path.exists(file_path):
            return

        content_json = dict()
        with open(file_path, 'r') as f:
            content_json = json.load(f)

        m = hashlib.md5()
        m.update(json.dumps(content_json).encode('utf-8'))
        content_json["md5"] = m.hexdigest()
        content_json["dataset"] = "planning_gt"

        with open(file_path, 'w') as f:
            json.dump(content_json, f)

    
    def procExpertResult(self):
        if self.expert_result == None:
            return
        
        self.expert_result['x'].reverse()
        self.expert_result['y'].reverse()
        self.expert_result['phi'].reverse()
        self.expert_result['ts'].reverse()

        self.expert_result = EasyDict(self.expert_result)

        self.init_point = pp.Point2d()
        self.target_point = pp.Point2d()
        
        self.init_point.x = self.expert_result['x'][-1 - self.begin_time_offset]
        self.init_point.y = self.expert_result['y'][-1 - self.begin_time_offset]
        self.init_point.theta = self.expert_result['phi'][-1 - self.begin_time_offset]
        self.target_point.x = self.expert_result['x'][0]
        self.target_point.y = self.expert_result['y'][0]
        self.target_point.theta = self.expert_result['phi'][0]


class EventReal(EventBase):
    def __init__(self):
        super().__init__()

    def init(self, event_info):
        super().init(event_info)
        self.getExpertResult(event_info['real_bag_path'])
        success = self.getPlanResult(event_info['real_bag_path'])
        self.printInitResult()
        return success

    def getExpertResult(self, bag_path):
        self.expert_result = dict()
        self.expert_result['x'] = list()
        self.expert_result['y'] = list()
        self.expert_result['phi'] = list()
        self.expert_result['ts'] = list()

        dataloader = DataLoader(bag_path)
        ego_data = dataloader.getTopic("/mla/egopose")
        cur_point = pp.Point2d()

        for data in ego_data:
            x = data[0].position.position_local.x
            y = data[0].position.position_local.y
            theta = data[0].orientation.euler_local.yaw
            # points between 0.1m do not count, to avoid noise
            if getVecDistance(x - cur_point.x, y - cur_point.y) < 0.1:
                continue
            cur_point.x = x
            cur_point.y = y
            self.expert_result['x'].append(x)
            self.expert_result['y'].append(y)
            self.expert_result['phi'].append(theta)
            self.expert_result['ts'].append(data[1])
        
        self.procExpertResult()
    

    def getPlanResult(self, bag_path):
        odo_list = self.readOdoFromBag(bag_path)
        if len(odo_list) == 0:
            print('no odo')
            return False
        if len(self.slot_corners) == 0:
            init_point = pp.Point2d()
            init_point.x = odo_list[0]['target_state']['path_point']['x']
            init_point.y = odo_list[0]['target_state']['path_point']['y']
            target_point = pp.Point2d()
            target_point.x = odo_list[0]['init_state']['path_point']['x']
            target_point.y = odo_list[0]['init_state']['path_point']['y']

            slot_type = pp.getSlotType(init_point, target_point)
            print('guessed slot_type: {}'.format(slot_type))
            self.slot_type = slot_type
        else:
            slot_type = pp.getSlotTypeFromSlotCorners(self.slot_corners)
            print('guessed slot_type: {}'.format(slot_type))
            self.slot_type = slot_type
        self.slot_type = slot_type
        sbp_res_list, car_size_params = planOnce(odo_list, slot_type)
        self.car_size_params = car_size_params
        self.odo_data = odo_list[0]
        self.plan_result = sbp_res_list[0]
        return True

    def readOdoFromBag(self, bag_file):
        odos = []
        dataloader = DataLoader(bag_file)
        request_msg = dataloader.getTopic("/msd/sbp_request")
        odo_gen = OdoGenerator(request_msg)
        odo_t = odo_gen.real_odo
        odos = [odo[0] for odo in odo_t]
        return odos
    
    def forkReplanEvents(self):
        if self.expert_result == None:
            print('{} fork replan failed cause no expert result.'.format(self.name))
            return list()

        self.generateGtDtJson()

        res_xs = copy.deepcopy(self.expert_result['x'])
        res_ys = copy.deepcopy(self.expert_result['y'])
        res_xs.reverse()
        res_ys.reverse()

        reverse_idxs = comm.getReversePointIndexs(res_xs, res_ys)
        print('replan indexes: {}'.format(reverse_idxs))
        
        event_list = list()

        for i in range(len(reverse_idxs)):
            idx = reverse_idxs[i] + 1

            if self.event_info != None:
                event = EventReal()
                event.slot_corners = self.slot_corners
                event.init(self.event_info)
            else:
                event = copy.deepcopy(self)
                event.name = self.name.split(":replan")[0]
                event.target_point = self.target_point
            
            event.name += ':replan No.{}'.format(i + 1)

            event.expert_result['x'] = event.expert_result['x'][0:-idx]
            event.expert_result['y'] = event.expert_result['y'][0:-idx]
            event.expert_result['phi'] = event.expert_result['phi'][0:-idx]
            event.expert_result['ts'] = event.expert_result['ts'][0:-idx]
            event.expert_result = EasyDict(event.expert_result)

            event.init_point.x = event.expert_result['x'][-1]
            event.init_point.y = event.expert_result['y'][-1]
            event.init_point.theta = event.expert_result['phi'][-1]

            event.slot_type = self.slot_type
            event.odo_data.target_state.path_point.x = event.init_point.x
            event.odo_data.target_state.path_point.y = event.init_point.y
            event.odo_data.target_state.path_point.theta = event.init_point.theta
            
            sbp_res_list, car_size_params = planOnce([event.odo_data], event.slot_type)
            event.car_size_params = car_size_params
            event.plan_result = sbp_res_list[0]

            event.generateGtDtJson(replan=True)

            event_list.append(event)

        return event_list
    
class EventVirtual(EventBase):
    def __init__(self):
        super().__init__()


class EventLidarTagged(EventBase):
    def __init__(self):
        super().__init__()
        self.label_data = None


    def init(self, event_info):
        super().init(event_info)

        self.label_data, self.wall_data = self.readLabelData(event_info['label_result_path'])

        r_s2b, t_s2b = self.readLidarYaml(event_info['lidar_config_path'])
        self.getExpertResultFromLidar(
            event_info['lidar_pose_path'],r_s2b, t_s2b)

        self.planLabelData(self.label_data)
        self.printInitResult()
        self.expert_min_distance = r2(pp.getEgoMinObsDistance(
            self.target_point, json.dumps(self.odo_data)))

    
    def forkReplanEvents(self):
        if self.expert_result == None:
            print('{} fork replan failed cause no expert result.'.format(self.name))
            return list()

        self.generateGtDtJson()
    
        res_xs = copy.deepcopy(self.expert_result['x'])
        res_ys = copy.deepcopy(self.expert_result['y'])
        res_xs.reverse()
        res_ys.reverse()

        reverse_idxs = comm.getReversePointIndexs(res_xs, res_ys)
        print('replan indexes: {}'.format(reverse_idxs))
        
        event_list = list()

        init_speed = 1

        for i in range(len(reverse_idxs)):
            idx = reverse_idxs[i] + 1

            init_speed = -init_speed

            if self.event_info != None:
                event = EventLidarTagged()
                event.slot_corners = self.slot_corners
                event.init(self.event_info)
            else:
                event = copy.deepcopy(self)
                event.name = self.name.split(":replan")[0]

            event.expert_result['x'] = event.expert_result['x'][0:-idx]
            event.expert_result['y'] = event.expert_result['y'][0:-idx]
            event.expert_result['phi'] = event.expert_result['phi'][0:-idx]
            event.expert_result = EasyDict(event.expert_result)

            event.init_point.x = event.expert_result['x'][-1]
            event.init_point.y = event.expert_result['y'][-1]
            event.init_point.theta = event.expert_result['phi'][-1]

            event.slot_type = self.slot_type
            event.slot_side = self.slot_side
            event.planLabelData(event.label_data, event.slot_type, 
                event.slot_side, init_speed)

            event.generateGtDtJson(replan=True)

            event.name += ':replan No.{}'.format(i + 1)

            event_list.append(event)

        
        self.setMd5(self.gt_json_path)

        return event_list

    
    def readLidarYaml(self, lidar_config_path):
        # fix %YAML:1.0 error
        content = ''
        with open(lidar_config_path, 'r') as f:
            content = f.read()

        if content.find('%YAML:1.0') != -1:
            with open(lidar_config_path, 'w') as f:
                content = content.replace('%YAML:1.0', '%YAML 1.0')
                f.write(content)
        
        with open(lidar_config_path, 'r') as f:
            lidar_params = yaml.load(f, Loader=yaml.FullLoader)
            return lidar_params['r_s2b'], lidar_params['t_s2b']

        return None, None


    def readLabelData(self, label_config_file):
        label_dict = dict()
        with open(label_config_file,'r') as f:
            label_dict = json.load(f)

        label_data = list()
        wall_lines = []
        for ch in label_dict['children']:
            points_xyz = None
            obs_type = None
            if 'data' in ch and 'points' in ch:
                points_str = ch['points'].split()
                points_xyz = [tuple(map(float,p.split(','))) for p in points_str]
                obs_type = ch['data']['type'][0]
            elif 'target' in ch and 'points' in ch:
                points_str = ch['points'].split()
                points_xyz = [tuple(map(float,p.split(','))) for p in points_str]
                obs_type = ch['target']

            if points_xyz == None:
                continue
            if obs_type not in EVENT_LABEL_TYPE_MAP:
                continue
            
            obs_type = EVENT_LABEL_TYPE_MAP[obs_type]
            label_points = []
            wall_line_x = []
            wall_line_y = []
            for p in points_xyz:
                label_points.append(pp.LabelPoint(p[0], p[1]))

            if obs_type == 'wall':
                for p in points_xyz:
                    wall_line_x.append(p[0])
                    wall_line_y.append(p[1])
                wall_lines.append([wall_line_x,wall_line_y])
                label_data.append(pp.ObstacleLabelData(
                    pp.ObstacleType.WALL, label_points))
            elif obs_type == 'car':
                label_data.append(pp.ObstacleLabelData(
                    pp.ObstacleType.CAR, label_points))
            elif obs_type == 'pillar':
                label_data.append(pp.ObstacleLabelData(
                    pp.ObstacleType.CAR, label_points))

        return (label_data, wall_lines)


    def getExpertResultFromLidar(self, lidar_pose_path, r_s2b, t_s2b):

        self.expert_result = dict()
        self.expert_result['x'] = list()
        self.expert_result['y'] = list()
        self.expert_result['phi'] = list()
        self.expert_result['ts'] = list()
        
        with open(lidar_pose_path, 'r') as f:
            lines = f.readlines()
            #cur_stamp = genpy.rostime.Time(0)
            cur_point = pp.Point2d()

            for line in lines:
                items = line.split(' ')
                #stamp = genpy.rostime.Time(int(items[0]) / 1000000)
                #if abs(stamp.secs - cur_stamp.secs) < 3:
                #    continue
                #cur_stamp = stamp
                
                lidar_pose = [float(items[1]), float(items[2]), 
                    float(items[3]), float(items[4]), \
                    float(items[5]), float(items[6]), float(items[7])]

                res = pp.convertLidarPose2EgoPose(lidar_pose, r_s2b, t_s2b)

                # points between 0.1m do not count, to avoid noise
                if getVecDistance(res.x - cur_point.x, res.y - cur_point.y) < 0.1:
                    continue
                cur_point = res

                self.expert_result['x'].append(res.x)
                self.expert_result['y'].append(res.y)
                self.expert_result['phi'].append(res.theta)
                self.expert_result['ts'].append(items[0])

        # average filter, not fit in this situation
        #self.expert_result['x'] = ss.medfilt(self.expert_result['x'], 21).tolist()
        #self.expert_result['y'] = ss.medfilt(self.expert_result['y'], 21).tolist()
        #self.expert_result['phi'] = ss.medfilt(self.expert_result['phi'], 21).tolist()
        
        self.procExpertResult()


    def planLabelData(self, label_data, 
        slot_type = pp.ParkingSlotType.UNKNOWN,
        slot_side = pp.ParkingSlotSide.SIDE_UNKNOWN,
        init_speed = 0):
        print('plan p0: {}, p1: {}'.format(self.init_point, self.target_point))

        if slot_type == pp.ParkingSlotType.UNKNOWN:
            if len(self.slot_corners) == 0:
                slot_type = pp.getSlotType(self.init_point, self.target_point)
                print('guessed slot_type: {}'.format(slot_type))
                self.slot_type = slot_type
            else:
                slot_type = pp.getSlotTypeFromSlotCorners(self.slot_corners)
                print('guessed slot_type: {}'.format(slot_type))
                self.slot_type = slot_type
    
        ep_car_param_file = os.path.abspath(
            conf['common']['car_params']['CAR_PARAM_EP'])
        print('ep car param file: {}'.format(ep_car_param_file))

        apa_p_file = os.path.abspath(
            conf['common']['scenario_params']['APA_PARALLEL'])
        print('apa parallel param file: {}'.format(apa_p_file))
        apa_v_file = os.path.abspath(
            conf['common']['scenario_params']['APA_VERTICAL'])
        print('apa vertical param file: {}'.format(apa_v_file))

        apa_o_file = os.path.abspath(
            conf['common']['scenario_params']['APA_OBLIQUE'])

        if slot_type == pp.ParkingSlotType.PARALLEL:
            pp.initSingletonParams(ep_car_param_file, apa_p_file)
        elif slot_type == pp.ParkingSlotType.OBLIQUE:
            pp.initSingletonParams(ep_car_param_file, apa_o_file)
        else:
            pp.initSingletonParams(ep_car_param_file, apa_v_file)

        str_res = pp.perfectScenePlan(self.init_point, self.target_point,
            label_data, slot_type, slot_side, self.slot_corners, init_speed)
        #print('result of perf plan: {}'.format(str_res))
        sbp_res = EasyDict(json.loads(str_res))
        self.plan_result = sbp_res['res']
        #self.odo_data = EasyDict(sbp_res['odo'])
        
        self.odo_data = sbp_res['odo']
        slot_side = sbp_res['slot_side']
        print('slot_side: {}'.format(slot_side))
        if slot_side == 0:
            self.slot_side = pp.ParkingSlotSide.SIDE_LEFT
        elif slot_side == 1:
            self.slot_side = pp.ParkingSlotSide.SIDE_RIGHT

        self.odo_data['points2lines'] = self.wall_data
        self.car_size_params = EasyDict(json.loads(pp.getCarSizeParams()))
