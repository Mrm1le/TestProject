from audioop import reverse
import sys
import os
import json
import numpy as np
import pickle
import re
import copy
import time
import math
import datetime

from easydict import EasyDict
from plotter.load_bag import DataLoader, PointsGenerator, DataGeneratorBase
from plotter.utils.common import GEO_TYPE
from plotter.utils.common import global2local, calcDisPointToLine

import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, Title
from bokeh.io import output_notebook, push_notebook, output_file
from bokeh.layouts import layout
from plotter.event_tools import MeasureTools
from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, PlannerFrameLayer

from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, TableColumn, DataTable
from bokeh.io import show


sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan


CAR_PARAM_RX5 = "../resources/vehicle_param_rx5.yaml"
CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
CAR_PARAM_BCAR = "../resources/vehicle_param_bcar.yaml"
CAR_PARAM_SG = "../resources/vehicle_param_sg.yaml"
CAR_PARAM_UXE = "../resources/vehicle_param_uxe.yaml"
CAR_PARAM_WLS = "../resources/vehicle_param_wls.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
APA_VERTICAL_2ND = "../resources/apa_vertical_2nd.yaml"                  # apa file for vertical scenario 2

GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon


SBP_REQUEST = "/msd/sbp_request"
USS_OBSTACLE = "/perception/fusion/uss_obstacle"
CHASSIS_REPORT = "/vehicle/chassis_report"
EGO_POSE_TPC = "/mla/egopose"
SYSTEM_CMD_TYPES = "/system_manager/cp_planning/request"

def planOnce(odo_list, slot_type):
    '''
    in:
        init_pose:
        target_pose:
        label_datas:
        slot_type:
    out:
        sbp_res: planner output
        odo: planner input
        car_size_params: 
    '''
    if slot_type == apa_plan.ParkingSlotType.PARALLEL:
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)
    else:
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_VERTICAL)
    
    car_size_params = EasyDict(json.loads(apa_plan.getCarSizeParams()))
    
    sbp_res_list = []
    for odo in odo_list:
        odo_str = json.dumps(odo)
        res_str = apa_plan.planInterfaceSerialize(odo_str)
        sbp_res_list.append(EasyDict(json.loads(res_str)))
    
    return (sbp_res_list, car_size_params)

def displayBatchRes(bag_names, odo_list, res_list, collision_set, uss_points, car_size_params, html_path, result_table, id_set = set()):

    figs = []
    
    source = ColumnDataSource(result_table)
    columns = [
        TableColumn(field='Evaluation Result', title='Evaluation Result', width=500),
        TableColumn(field='result', title='', width=500)
        ]
    myTable = DataTable(source=source, columns=columns, width=1200, height=max(125, 25 * (len(result_table['result'])+1)))
    figs.append(myTable)
    
    column_num = 2
    
    output_file(html_path)
    plot_num = 0
    for index in range(len(odo_list)):
        if len(id_set) > 0 and index not in id_set:
            continue

        if plot_num % column_num ==0:
            figs.append([])
        
        plot_num += 1
        plot_title = ("" if index not in collision_set else "COLLISION:") +  "plan "+ str(index) + "; bag keyword: "+bag_names[index]

        # set layers
        fig = bkp.figure(title=plot_title,
                        x_axis_label='x',
                        y_axis_label='y',
                        match_aspect=True,
                        width=700,
                        height=500)

        # fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
        MeasureTools(fig)
        
        frame_layer = PlannerFrameLayer(fig)
        if index in collision_set:
            frame_layer.out_layer = PlannerOutputLayer(fig,  {'line_color': 'red', 'legend_label':'planner path'}, {'line_color': 'pink', 'legend_label':'car polygon'})
            
        uss_frame = None
        if uss_points is not None:
            uss_frame = uss_points[index]
        frame_layer.update(odo_list[index], res_list[index],uss_frame, car_size_params)
        fig.legend.click_policy = 'hide'
        
        figs[-1].append(frame_layer.getLayout())

        
        # -
    # fig_all = list(map(list, zip(*figs)))  # transpose
    bkp.show(layout(figs), notebook_handle=True)

class SystemCmdGenerator(DataGeneratorBase):
    def __init__(self, data):
        cmd_types, ts = self._convert(data)
        super().__init__(cmd_types, ts)
        self.cmd_types = cmd_types

    def _convert(self, data):
        if data is None:
            return [[]]

        cmd_types = []
        ts = []
        for d in data:
            ts.append(d[1])
            cmd_type = [d[1], d[0].cmd.value]
            cmd_types.append(cmd_type)

        if len(cmd_types) == 0:
            cmd_types = [[]]

        return cmd_types, ts

class EgoGenerator(DataGeneratorBase):
    def __init__(self, data):
        xytheta, ts = self._convert(data)
        super().__init__(xytheta, ts)
        self.xytheta = xytheta

    def _convert(self, data):
        if data is None:
            return [[]]

        xytheta = []
        ts = []
        for d in data:
            ts.append(d[1])
            cx = d[0].position.position_local.x
            cy = d[0].position.position_local.y
            ctheta = d[0].orientation.euler_local.yaw
            pos_data = [d[1], cx, cy, ctheta]
            xytheta.append(pos_data)

        if len(xytheta) == 0:
            xytheta = [[]]

        return xytheta, ts

class OdoGenerator(DataGeneratorBase):
    def __init__(self, data):
        xys, ts, real_odo = self._convert(data)
        super().__init__(xys, ts)
        self.real_odo = real_odo
        # self.calcSlotLength(data)
    
    def _convert(self, data):
        if data is None:
            return [[], []]
        
        real_odo = []
        xys = []
        ts = []
        for d in data:
            ts.append(d[1])
            param_str=json.loads(d[0].task_config.problem_config.params_string)
            temp_odo = EasyDict(param_str["OpenspaceDeciderOutput"])
            if d[0].task_info.goal_id.id == "search_problem":
                real_odo.append((temp_odo, d[1]))
                # break
            
            xys.append(temp_odo)

        return (xys, ts, real_odo)
    
    def atT(self, T):
        if len(self.xys) == 0:
            return None
        if self.getMinT() > T:
            return (self.xys[0], self.ts[0])
            
        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return (self.xys[-1], self.ts[-1])
        
        return (self.xys[first_larger_index[0] - 1], self.ts[first_larger_index[0] - 1])

    def calcSlotLength(self, data):
        if data is None:
            return [[], []]
        
        for d in data:
            param_str=json.loads(d[0].task_config.problem_config.params_string)
            temp_odo = EasyDict(param_str["OpenspaceDeciderOutput"])
            if d[0].task_info.goal_id.id == "search_problem":
                slot_left_bound = temp_odo.T_lines.slot_left_bound
                slot_right_bound = temp_odo.T_lines.slot_right_bound
                left_start_point = [slot_left_bound.start_.x_, slot_left_bound.start_.y_]
                left_end_point = [slot_left_bound.end_.x_, slot_left_bound.end_.y_]
                right_start_point = [slot_right_bound.start_.x_, slot_right_bound.start_.y_]
                right_end_point = [slot_right_bound.end_.x_, slot_right_bound.end_.y_]
                dis1 = calcDisPointToLine(left_start_point, right_start_point, right_end_point)
                dis2 = calcDisPointToLine(left_end_point, right_start_point, right_end_point)
                dis3 = calcDisPointToLine(right_start_point, left_start_point, left_end_point)
                dis4 = calcDisPointToLine(right_end_point, left_start_point, left_end_point)
                line_dis = (dis1 + dis2 + dis3 + dis4) / 4
                # print("line_dis: ", line_dis)

def getAllFiles(bag_folder):
    if os.path.isfile(bag_folder):
        print("only one")
        return [bag_folder]
    
    bag_files = []
    pickle_file = None
    if not os.path.isdir(bag_folder):
        print("INVALID ARGV:\n bag_path: {}".format(bag_folder))
        return bag_files

    all_bag_files = os.listdir(bag_folder)
    for bag_name in all_bag_files:
        suffix = bag_name.split('.')[-1]
        if suffix == 'pickle':
            pickle_file = os.path.join(bag_folder, bag_name)
        if suffix != 'bag':
            continue
        bag_files.append(os.path.join(bag_folder, bag_name))
        
    print("find {} files".format(len(bag_files)))
    
    if pickle_file is not None:
        print("find pickle file {}".format(pickle_file))
    
    return bag_files, pickle_file


def loadDataFromBag(batch_files, readOdoFromBag):
    ms = []
    print("load datas from bag ...")
    bag_names = []
    for index, file_bag in enumerate(batch_files):
        print("read {} bag: {}".format(index, file_bag))
        if '/' not in file_bag:
            bag_name = file_bag.split('.')[0]
        else:
            bag_name = file_bag.split('/')[-1].split('.')[0]
        
        match_res = re.search( r'\d{8}-\d{6}_\d{8}-\d{6}', bag_name)
        if  match_res:
            bag_name = match_res.group() 
        msg = readOdoFromBag(file_bag)           
        bag_names.extend([bag_name]*len(msg))
        ms.extend(msg)
    
    return ms, bag_names

def loadData(file_name):
    datas = None
    with open(file_name, 'rb') as file:
        print("load data from {}".format(file_name))
        datas = pickle.load(file)
    
    return datas

def dumpData(datas, pickle_file):
    with open(pickle_file, 'wb') as file:
        print("dump data to {}".format(pickle_file))
        pickle.dump(datas, file)

def getCollisionPlans(sbp_res_list):
    num =0
    collision_sets = set()
    for res in sbp_res_list:
        debug_string =  res.debug_string
        if debug_string:
            debug_list = debug_string.split('|')
            if debug_list[-1] == 'collision':
                collision_sets.add(num)

        num += 1
    
    return collision_sets

def calcPlanningEndOffset(bag_names, odos_list, ego_pose_list, cmd_type_list):
    bag_odo_index = []
    if len(bag_names) != len(odos_list):
        return
    i = 0
    while(i < len(bag_names) - 1):
        front_bag_name = bag_names[i]
        end_bag_name = bag_names[i + 1]
        if front_bag_name != end_bag_name:
            bag_odo_index.append(i)
        i += 1
    bag_odo_index.append(len(bag_names) - 1)
    if len(bag_odo_index) != len(ego_pose_list):
        return
    ego_pose_index_list = getPlanningFinalEgoPose(ego_pose_list, cmd_type_list)
    planning_end_offset_list = []
    planning_end_offset_sum = 0
    for index in range(len(ego_pose_list)):
        ego_poses = ego_pose_list[index]
        global_ego_pose_x = ego_poses[ego_pose_index_list[index]][1]
        global_ego_pose_y = ego_poses[ego_pose_index_list[index]][2]
        global_target_pose_x = odos_list[bag_odo_index[index]].init_state.path_point.x
        global_target_pose_y = odos_list[bag_odo_index[index]].init_state.path_point.y
        global_target_pose_theta = odos_list[bag_odo_index[index]].init_state.path_point.theta
        (dx, dy) = global2local(
            global_ego_pose_x,
            global_ego_pose_y,
            global_target_pose_x,
            global_target_pose_y,
            global_target_pose_theta
        )
        planning_end_offset_list.append((dy + 0.25))
        planning_end_offset_sum += (dy + 0.25)
    if len(planning_end_offset_list) != 0:
        planning_end_offset_ave = planning_end_offset_sum / len(planning_end_offset_list)
        print("planning_end_offset_ave: ", planning_end_offset_ave)
    return

def getPlanningFinalEgoPose(ego_pose_list, cmd_type_list):
    ego_pose_index_list = []
    if len(ego_pose_list) != len(cmd_type_list):
        return ego_pose_index_list
    for index in range(len(ego_pose_list)):
        cmd_over_time = 0
        cmd_types = cmd_type_list[index]
        ego_poses = ego_pose_list[index]
        for cmd_type in cmd_types:
            if cmd_type[1] == 655365:
                cmd_over_time = cmd_type[0]
                break
        ego_pose_index = 0
        min_deta_time = 10000.0
        for jndex in range(len(ego_poses)):
            tmp_deta_time = abs(ego_poses[jndex][0] - cmd_over_time)
            if tmp_deta_time < min_deta_time:
                ego_pose_index = jndex
                min_deta_time = tmp_deta_time
        ego_pose_index_list.append(ego_pose_index)
    return ego_pose_index_list

def processRes(bag_names, odos_no_uss, sbp_res_list,usss, car_size_params, html_path):
    res_num = 0
    fail_ids = []
    index = 0
    for sbp_res in sbp_res_list:
        if len(sbp_res.x) >0:
            res_num += 1
        else:
            fail_ids.append(index)
        
        index += 1
    
    tinys, reverse_times, block_direc = analyseTrajectory(sbp_res_list, odos_no_uss)
    print("tiny:", tinys)
    print('block_direc:', block_direc)
    
    collision_set = getCollisionPlans(sbp_res_list)
    collision_num = len(collision_set)
    print("collision:", collision_set)
    avg_seg_num = 0
    if len(reverse_times) != 0:
        avg_seg_num = sum(reverse_times.values())/len(reverse_times)
        print('reverse times:', avg_seg_num)

    
    id_set = set()
    id_set.update(list(range(len(sbp_res_list))))
    # id_set = set(fail_ids)
    # id_set.update(collision_set)

    success_rate = 0
    if len(odos_no_uss) != 0:
        success_rate = round((res_num-collision_num)/len(odos_no_uss), 4)
    if collision_num >0:
        print("\033[0;31;40m collision num: {}\033[0m".format(collision_num))
    else:
        print("\033[0;32;40m collision num: {}\033[0m".format(collision_num))
    
    if len(fail_ids) > 0:
        print("\033[0;32;40m fail_ids: {}\033[0m".format(fail_ids))
    
    print("\033[0;32;40m success rate: {}/{} = {:.4f}\033[0m".format(res_num-collision_num, len(odos_no_uss),success_rate))

    offset_list = calcEndOffsetList(sbp_res_list, odos_no_uss)
    average_offset_list = []
    num_offset = len(offset_list)
    if num_offset != 0:
        sum_offset = 0
        for tmp_offset in offset_list:
            sum_offset += tmp_offset
        average_offset = sum_offset / num_offset
        average_offset_list.append(average_offset)
    print("average_offset_list: ", average_offset_list)
    
    if html_path is not None:
        now = datetime.datetime.now()
        style_time = now.strftime("%Y-%m-%d %H:%M:%S")
        result_table = {
            "Evaluation Result": ['total num', 'path exists num','collision nums', 'success rate = (path exists num - collision nums)/total num', 'date',
                                  'fail ids', 'tinys', 'average_final_offset', 'avg segment'],
            "result": [
                len(odos_no_uss), 
                res_num, 
                '{}:[{}]'.format(collision_num, ','.join(list(map(str,collision_set)))) if collision_num > 0  else str(collision_num),
                success_rate, 
                style_time,
                '{}:[{}]'.format(len(fail_ids), ','.join(list(map(str,fail_ids)))),
                '{}:{}'.format(len(tinys), '' if len(tinys) ==0 else ','.join([str(t[0])+':'+str(t[1]) for t in tinys])),
                average_offset_list,
                avg_seg_num
            ]
        }
        print("plot results ...")
        displayBatchRes(bag_names, odos_no_uss, sbp_res_list,collision_set,usss, car_size_params, html_path, result_table, id_set)
        print("complete!")

def calcEndOffsetList(sbp_res_list, odo_list):
    offset_list = []
    for index, sbp_res in enumerate(sbp_res_list):
        if len(sbp_res.x) ==0:
            continue
        if len(odo_list) <= index:
            continue
        global_end_offset_x = sbp_res.x[0]
        global_end_offset_y = sbp_res.y[0]
        global_target_pose_x = odo_list[index].init_state.path_point.x
        global_target_pose_y = odo_list[index].init_state.path_point.y
        global_target_pose_theta = odo_list[index].init_state.path_point.theta
        (dx, dy) = global2local(
            global_end_offset_x,
            global_end_offset_y,
            global_target_pose_x,
            global_target_pose_y,
            global_target_pose_theta
        )
        offset_list.append(dy)
        # print("dy: ", dy)
    return offset_list

def analyseTrajectory(sbp_res_list, odo_list):
    tinys = []
    reverse_times = {}
    block_direc = []
    lat_pose_error = []
    for index,sbp_res in enumerate(sbp_res_list):
        if len(sbp_res.x) ==0:
            continue

        xs = sbp_res.x
        ys = sbp_res.y
        segs = getSegmentsLength(xs, ys)
        segs.sort(reverse = False)
        if len(segs) == 0:
            continue
        if segs[0] < 0.15:
            tinys.append((index, segs[0]))
        
        reverse_times[index] = len(segs)

        if len(odo_list) <= index:
            continue
        
        init_v = odo_list[index].target_state.v
        last_direc = 0 if abs(init_v)<1e-6 else (1 if init_v>0 else -1)


        phis = sbp_res.phi
        if len(xs) <2:
            continue
        if abs(phis[0]-phis[-1]) <math.pi/6.0 and math.hypot(xs[0]-xs[-1], ys[0]-ys[-1]) > 5.0:
            continue
        dot_x = math.cos(phis[-1])*(xs[-2]-xs[-1])
        dot_y = math.sin(phis[-1])*(ys[-2]-ys[-1])
        res_dot = dot_x+dot_y
        if res_dot*last_direc > 1e-6:
            block_direc.append((index, last_direc, res_dot))

    return tinys, reverse_times, block_direc
    

def getSegmentsLength(xs, ys):
    seg_length = []
    reverse_num = 0
    if len(xs) != len(ys):
        return []

    if len(xs) < 3:
        return []

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

    if len(dx) ==0:
        return seg_length

    seg_length.append(math.hypot(dx[0], dy[0]))
    for i in range(len(dx)-1):
        if dx[i] * dx[i+1] + dy[i] * dy[i+1] < 0.0:
            seg_length.append(math.hypot(dx[i+1], dy[i+1]))
            reverse_num += 1
            continue
        seg_length[-1] += math.hypot(dx[i+1], dy[i+1])

    return seg_length


if __name__ == '__main__':
    bag_folder = None
    html_path = None
    if len(sys.argv) <2:
        print("PLEASE SPECIFY <BAG_FOLDER or BAG_FILE> <HTML_PATH>")
        exit(0)
        
    if len(sys.argv) >1:
        bag_folder = sys.argv[1]
    if len(sys.argv) >2:
        html_path = sys.argv[2]
    
    bag_files, pickle_file = getAllFiles(bag_folder)
    msg = []
    bag_names = []
    
    if pickle_file is not None:
        msg, bag_names = loadData(pickle_file)
    else:
        msg, bag_names = loadDataFromBag(bag_files)
        dumpData((msg, bag_names), os.path.join(bag_folder, "bag.pickle"))
    
    # odos_no_uss = [m[0] for m in msg]
    # usss = [u[1] for u in msg]
    # odo_list = convertOdo(odos_no_uss, usss)
    
    odo_list = msg
    
    print("run planner with {} frames ...".format(len(odo_list)))
    t1  =time.time()
    sbp_res_list, car_size_params = planOnce(odo_list, apa_plan.ParkingSlotType.PARALLEL)
    t2  =time.time()
    print("\033[0;32;40m planning cost {:.2f}s\033[0m".format(t2-t1))
    
    
    res_num = 0
    for sbp_res in sbp_res_list:
        if len(sbp_res.x) >0:
            res_num += 1
    
    success_rate = round(res_num/len(odo_list), 4)
    print("\033[0;32;40m success rate: {}/{} = {:.4f}\033[0m".format(res_num, len(odo_list),success_rate))
    
    if html_path is not None:
        now = datetime.datetime.now()
        style_time = now.strftime("%Y-%m-%d %H:%M:%S")
        result_table = {
            "Evaluation Result": ['total num', 'success num', 'success rate', 'date'],
            "result": [len(odo_list), res_num, success_rate, style_time]
        }
        print("plot results ...")
        displayBatchRes(odo_list, sbp_res_list,None, car_size_params, html_path, result_table)
        print("complete!")
    
