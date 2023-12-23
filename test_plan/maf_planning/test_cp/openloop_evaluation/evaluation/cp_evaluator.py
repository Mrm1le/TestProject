# -*- coding: utf-8 -*-

import sys
import os
import math
import json
import copy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../../lib'))

from easydict import EasyDict

class PlanPathOutput(object):
    def __init__(self, data=None, name = None, event_name = None):
        if name  == None:
            self.name = "Plan Path Output"
        else:
            self.name = name
        if event_name == None:
            self.event_name = "event_01"
        else:
            self.event_name = event_name
        self.data = EasyDict(origin_data = [], converted_data = [])
        if data != None:
            self.load_data(data)
        
    def load_data(self, data):
        self.data.origin_data = data
        ts = list()
        for data_json_frame in self.data.origin_data:
            ts.append(data_json_frame.time_stamp)
            data_json = data_json_frame.nb_debug
            converted_data_slice = EasyDict()
            converted_data_slice.time_stamp = (data_json_frame.time_stamp)
            converted_data_slice.plan_curv       = list()
            converted_data_slice.plan_yaw        = list()
            converted_data_slice.plan_lat_jerk   = list()
            converted_data_slice.time_array      = list()
            converted_data_slice.plan_path       = list()
            temp_matrix = []
            if 's_to_samples' not in data_json.keys():
                return
            for s_to_sample in data_json['s_to_samples']:
                if len(s_to_sample) < 2:
                    return 
                if 'sample_data' not in s_to_sample[1]:
                    return 
                sample_data = s_to_sample[1]['sample_data']
                if sample_data['t'] > 20:
                    # print(sample_data['t'])
                    continue
                plan_path = EasyDict(x = sample_data['x'], y = sample_data['y'])
                temp_array = [sample_data['t'],plan_path,sample_data['curvature'],sample_data['lat_jerk']]
                temp_matrix.append(temp_array)

            temp_matrix = sorted(temp_matrix, key=(lambda x: x[0]))   
            for temp_array in temp_matrix:
                converted_data_slice.time_array.append(temp_array[0])
                converted_data_slice.plan_path.append(temp_array[1])
                converted_data_slice.plan_curv.append(temp_array[2])
                converted_data_slice.plan_lat_jerk.append(temp_array[3])
            self.data["converted_data"].append(converted_data_slice)
        # print(self.data["converted_data"][0])
        # print(ts)
        return 


class PathPlanInput(object):
    def __init__(self, data = None, name = None, event_name = None):
        if name  == None:
            self.name = "Plan Path Input"
        else:
            self.name = name
        if event_name == None:
            self.event_name = "event_01"
        else:
            self.event_name = event_name
        self.data = EasyDict(origin_data = [], origin_data_real = [], converted_data = [])
        if data != None:
            self.load_data(data)

    def load_data(self, data):
        if len(data) == 0:
            return 
        if "path_planner_input" not in data['frames_info'][0].keys():
            return
        for data_frame in data['frames_info']:
            origin_data_slice = EasyDict()
            origin_data_slice.path_planner_input = data_frame["path_planner_input"]
            origin_data_slice.time_stamp = data_frame["time_stamp"]
            origin_data_slice.professor_pose = data_frame['professor_pose']
            self.data.origin_data.append(origin_data_slice)
        for data_frame in data['frames_info_real']:
            origin_data_real_slice = EasyDict()
            origin_data_real_slice.path_planner_input = data_frame["path_planner_input"]
            origin_data_real_slice.time_stamp = data_frame["time_stamp"]
            origin_data_real_slice.professor_pose = data_frame['professor_pose']
            self.data.origin_data_real.append(origin_data_real_slice)
            
        converted_data = list()
        ts = list()
        for data_frame in data['frames_info']:
            # if 'header' not in data_json.keys():
            #     return
            data_json = data_frame["path_planner_input"]
            converted_data_slice = EasyDict()
            converted_data_slice.refline_info = list()
            converted_data_slice.time_stamp = data_frame["time_stamp"]

            ts.append(data_frame["time_stamp"])
            for path_segment in data_json['path_segments']:
                for quad_point_info in path_segment['quad_point_info']:
                    refline_info  = EasyDict(x = quad_point_info['refline_info']['x'], \
                        y = quad_point_info['refline_info']['y'])
                    converted_data_slice.refline_info.append(refline_info)
            converted_data.append(converted_data_slice)
        self.data["converted_data"] = converted_data
        return 
                    
class EnvInfo(object):
    def __init__(self, data_gt = None, name = None, event_name = None):
        if name  == None:
            self.name = "Env Info"
        else:
            self.name = name
        if event_name == None:
            self.event_name = "event_01"
        else:
            self.event_name = event_name
        self.data = EasyDict(origin_data = [], converted_data = [], \
                            real_env = [])
        if data_gt != None:
            self.load_data(data_gt)
            

    def load_data(self, data):  
        if len(data) == 0:
            return      
        if "processed_map_data" not in data['frames_info'][0].keys():
            return
        # print(data[0].keys())
        for data_frame in data['frames_info']:
            self.data.origin_data.append(data_frame["processed_map_data"])

        converted_data = list()
        # ts = list()
        # print(data['frames_info'][0].keys())
        for data_frame in data['frames_info']:
            data_json = data_frame["processed_map_data"]
            
            # print(data_json.keys())

            converted_data_slice = EasyDict()
            converted_data_slice.lane_point_right = list()
            converted_data_slice.lane_point_left  = list()
            converted_data_slice.re_left      = list()
            converted_data_slice.re_right     = list()
            converted_data_slice.refline_info = list()
            
            converted_data_slice.time_stamp = data_frame['time_stamp']
            converted_data_slice.all_lanes  = list()
            converted_data_slice.all_objs   = data_frame["perception_fusion_objects_data"]
            # ts.append(data_frame.time_stamp)
            if len(data_json['lanes']) == 0:
                return
            
            for lane in data_json['lanes']:
                if lane["relative_id"]!=0:
                    if "right_lane_boundary" in lane.keys():
                        converted_data_slice.all_lanes.append(lane["right_lane_boundary"]['points'])
                    if "left_lane_boundary" in lane.keys():   
                        converted_data_slice.all_lanes.append(lane["left_lane_boundary"]['points'])
                else:
                    if "right_lane_boundary" in lane.keys():
                        converted_data_slice.lane_point_right = lane["right_lane_boundary"]['points']
                    if "left_lane_boundary" in lane.keys():                    
                        converted_data_slice.lane_point_left = lane["left_lane_boundary"]['points']

            converted_data_slice.re_left  = data_json['lanes'][0]['left_road_edge']['points']
            converted_data_slice.re_right = data_json['lanes'][0]['right_road_edge']['points']

            converted_data.append(converted_data_slice)

        if "0" in data['real_env'].keys():
            self.data["real_env"] = data['real_env']["0"]
        self.data["real_env"]["professor_traj"] = []
        self.data["real_env"]["professor_traj_time"] = []
        
        if "professor_traj" in data['real_env'].keys():
            for professor_point in data['real_env']["professor_traj"]:
                self.data["real_env"]["professor_traj"].append(EasyDict(professor_point['professor_traj_point']))
                self.data["real_env"]["professor_traj_time"].append(professor_point['time_stamp'])
                if professor_point['time_stamp'] > data['frames_info'][-1]['time_stamp'] + 5:
                    break
            # self.data["real_env"]["professor_traj"] = data['real_env']["professor_traj"]

        self.data["converted_data"] = converted_data
        return 


class CPCaseBase(object):
    def __init__(self):
        self.name = ''

    def evaluate(self, plan_result, path_result_real=None,
            expert_result=None, target_point=None, odo = None):
        return 0, ''



class CPEvaluator(CPCaseBase):
    def __init__(self, plan_path_info, res_table_keys, env_info = None, path_plan_input = None, evaluate_init = 0):
        # 1. define result table
        self.res_table = dict()
        self.matched   = True
        for key in res_table_keys:
            self.res_table[key] = []
        self.res_table['test case'] = [plan_path_info.event_name]
        self.evaluation_init = evaluate_init        # the frame_number when evaluation initiates
        # 2. match each frame
        self.plan_path_info, self.env_info = self.data_matching(plan_path_info,env_info)
        # self.expert_traj_cut = self.cut_expert_traj(path_plan_input)
        
        # 3. extract key signals for evaluate
        self.pp_curv_rate = self.cal_curv_rate()
        # self.pp_curv = self.cal_curvature()
        # self.pp_curv_rate = self.cal_curv_rate_1s_max()

        # 4. evaluate
        self.evaluate()

    def cut_expert_traj(self, path_plan_input):
        expert_traj_cut = []
        if len(path_plan_input.data.origin_data) == 0:
            return expert_traj_cut
        if "professor_pose" not in path_plan_input.data.origin_data[0].keys():
            return expert_traj_cut
        if not self.matched:
            return expert_traj_cut

        # index_match = 0
        self.expert_traj_full = list()
        ii = 0 
        yaw0 = 0
        for converted_data_slice in path_plan_input.data.origin_data:
            expert_slice = EasyDict()
            expert_slice.expert_pose = converted_data_slice.professor_pose
            expert_slice.time_stamp  = converted_data_slice.time_stamp
            yaw_rate_temp = 0.0
            expert_slice.expert_pose.speed = math.sqrt(pow(converted_data_slice.professor_pose.velocity_x, 2) + \
                                                pow(converted_data_slice.professor_pose.velocity_y, 2))
            if ii == 0:
                pass
            else:
                dt = converted_data_slice.time_stamp - t0
                yaw_rate_temp = converted_data_slice.professor_pose.yaw-yaw0
                yaw_rate_temp = yaw_rate_temp/dt
            # print(expert_slice.expert_pose.speed,yaw_rate_temp)
            ii += 1

            expert_slice.expert_pose.yaw_rate = yaw_rate_temp
            yaw0 = converted_data_slice.professor_pose.yaw
            t0   = converted_data_slice.time_stamp
            # print(type(expert_slice.expert_pose))

            self.expert_traj_full.append(expert_slice)

            # print("matched_index:", index_match)
        ii = 0

        for converted_data_slice in self.plan_path_info.data.converted_data:
            matched_index_start = -1
            matched_index_end = -1
            expert_traj_cut_slice = EasyDict(time_stamp = [], expert_interval = [])
            expert_traj_cut_slice.time_stamp.append(converted_data_slice.time_stamp)

            if converted_data_slice.plan_path == []:
                pass
            else:
                matched_index_start = ii
                matched_index_end   = matched_index_start
                
                x0 = self.expert_traj_full[matched_index_start].expert_pose.x
                y0 = self.expert_traj_full[matched_index_start].expert_pose.y
                p_x_e = converted_data_slice.plan_path[-1].x
                p_y_e = converted_data_slice.plan_path[-1].y
                match_range_square = pow(p_x_e - x0,2) + pow(p_y_e - y0,2)

                while matched_index_end < len(self.expert_traj_full):
                    e_x_e = self.expert_traj_full[matched_index_end].expert_pose.x
                    e_y_e = self.expert_traj_full[matched_index_end].expert_pose.y
                    expert_range_square = pow(e_x_e - x0,2) + pow(e_y_e - y0,2)

                    if expert_range_square < match_range_square:
                        matched_index_end += 1
                    else: 
                        break
            
            # expert_traj_cut.append()
            # print("matched range:", matched_index_start,matched_index_end)
            expert_traj_cut_slice.expert_interval = [matched_index_start,matched_index_end]
            expert_traj_cut.append(expert_traj_cut_slice)

            ii+=1
        
        return expert_traj_cut
        

    def data_matching(self, plan_path_info, env_info):
        matched_env_info = dict()
        # matched_plan_path = PlanPathOutput("Plan Path")
        if env_info == None:
            return plan_path_info, matched_env_info
        # print("Matching? ", len(env_info.data.real_env.professor_traj),\
        #                         len(plan_path_info.data.converted_data), \
        #                             len(env_info.data.real_env.right_road_edge), \
        #                                 len(env_info.data.real_env.left_lane_boundary))
        if len(env_info.data.real_env.professor_traj) == len(plan_path_info.data.converted_data):
            self.matched = True
        else:
            pass
            # self.matched = False
            # print("Error! Data not matched!")

        return plan_path_info, env_info
    
    def cal_TTC(self, plan_path_info, bound_left, bound_right,init_index = 0):
        TTC        = 20
        min_dist2B = 1e6
        t_min_dist2B = 20

        hit_bound = False
        if plan_path_info.time_array == []:
            return (TTC,min_dist2B,t_min_dist2B)
        x0 = plan_path_info.plan_path[0].x
        y0 = plan_path_info.plan_path[0].y

        for ii in range(len(plan_path_info.time_array)):
            t_point = plan_path_info.time_array[ii]
            if t_point <=1:
                continue
            tempx = plan_path_info.plan_path[ii].x
            tempy = plan_path_info.plan_path[ii].y
            d_thres = math.sqrt(pow(tempx-x0, 2) + pow(tempy-y0, 2)) + 15
            d_thres = 15

            start_cal_left  = False
            start_cal_right = False
            
            for point1 in bound_left[init_index:len(bound_left)]:
                dist = math.sqrt(pow(point1['x'] - tempx,2) + pow(point1['y'] - tempy,2))
                # print("left:", dist)
                if dist <= d_thres:
                    start_cal_left = True

                if start_cal_left and dist > d_thres:
                    start_cal_left = False
                    break
                
                if dist < 1.2:
                    hit_bound = True
                    break
                if dist < min_dist2B:
                    min_dist2B   = dist
                    t_min_dist2B = plan_path_info.time_array[ii]

            if hit_bound:
                TTC = plan_path_info.time_array[ii]
                min_dist2B   = 0
                t_min_dist2B = plan_path_info.time_array[ii]
                break
            for point1 in bound_right[init_index:len(bound_right)]:
                dist = math.sqrt(pow(point1['x']  - tempx,2) + pow(point1['y'] - tempy,2))
                
                if dist <= d_thres:
                    start_cal_right = True

                if start_cal_right and dist > d_thres:
                    start_cal_right = False
                    break
                # dist = math.sqrt(pow(point1['x']  - tempx,2) + pow(point1['y'] - tempy,2))
                if dist < 1.2:
                    hit_bound = True
                    break
                if dist < min_dist2B:
                    # print("right:", dist)
                    min_dist2B   = dist
                    t_min_dist2B = plan_path_info.time_array[ii]
            if hit_bound:
                TTC = plan_path_info.time_array[ii]
                min_dist2B   = 0
                t_min_dist2B = plan_path_info.time_array[ii]
                break
        
        return (TTC,max(min_dist2B-1.2,0),t_min_dist2B)

    def cal_traj2traj_min_dist(self, path1, path2):
        min_dis = 1e6
        index_res = 0

        if path1 == [] or path2 == []:
             return min_dis, index_res


        start_cal = False
        index_1 = 0
        dis_thres = 20
        for point1 in path1:
            start_cal = False

            for point2 in path2:
                point1_point2_dist = math.sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y ,2))-1.2
                if point1_point2_dist <= dis_thres:
                    start_cal = True
                if start_cal and point1_point2_dist > dis_thres:
                    start_cal = False
                    break
                if start_cal :
                    if min_dis > point1_point2_dist:
                        min_dis = point1_point2_dist
                        index_res = index_1

            index_1 += 1
            
        return min_dis, index_res

    def evaluate(self):
        
        # Initialization
        
        
        rel_time_curvrate = 0.00
        max_curv_rate = 0
        t_curvrate = 0.00

        rel_time_jerk = 0.00
        t_jerk = 0.00
        max_jerk = 0.00

        if self.plan_path_info.data.converted_data == []:
            self.res_table['summary score'] = [0]
            self.res_table['PLANNING SUCCESS'] = [False]
            return 

        plan_path_info = self.plan_path_info.data.converted_data                 # all frame path plan output result
        time_stamp0 = plan_path_info[0].time_stamp

        t0 = self.env_info.data.converted_data[0].time_stamp

        # Evaluate curv rate
        for ii in range(len(plan_path_info)):
            if ii < self.evaluation_init:
                continue
            if plan_path_info[ii].time_array == []:
                continue
            tp0 = plan_path_info[ii].time_array[0]
            for jj in range(len(self.pp_curv_rate[ii])):
                if self.pp_curv_rate[ii][jj] > max_curv_rate:
                    max_curv_rate = self.pp_curv_rate[ii][jj]
                    t_curvrate = plan_path_info[ii].time_array[jj] - tp0          # plan中的相对时刻
                    rel_time_curvrate = plan_path_info[ii].time_stamp-time_stamp0 # plan_path_info[ii].time_stamp 帧的时间戳

        # Evaluate lateral jerk
        for ii in range(len(plan_path_info)):
            if ii < self.evaluation_init:
                continue
            if plan_path_info[ii].time_array == []:
                continue
            for jj in range(len(self.pp_curv_rate[ii])):
                if abs(plan_path_info[ii].plan_lat_jerk[jj]) > max_jerk:
                    max_jerk = abs(plan_path_info[ii].plan_lat_jerk[jj])
                    t_jerk = plan_path_info[ii].time_array[jj] - \
                        plan_path_info[ii].time_array[0]
                    rel_time_jerk = plan_path_info[ii].time_stamp-time_stamp0

        
        # Evaluate expert jerk
        expert_jerk_max = 0.0
        t_expert_jerk_max = 0.0
        ii = 0
        for expert_traj_slice in self.env_info.data.real_env['professor_traj']:
            if ii < self.evaluation_init:
                ii += 1
                continue
            if expert_jerk_max < abs(expert_traj_slice.jerk):
                expert_jerk_max = abs(expert_traj_slice.jerk)
                t_expert_jerk_max = self.env_info.data.real_env['professor_traj_time'][ii] - t0
            ii += 1

        t_expert_jerk_max = round(t_expert_jerk_max, 2)

        # Evaluate expert rate
        expert_curve_rate_max = 0.0
        t_expert_curve_rate_max = 0.0
        ii = 0
        for expert_traj_slice in self.env_info.data.real_env['professor_traj']:
            if ii == 0:
                expert_curve0 = expert_traj_slice.curvature
                temp_t0 = self.env_info.data.real_env['professor_traj_time'][ii]
                ii += 1
                continue
            if ii < self.evaluation_init:
                ii += 1
                continue     
            dt = self.env_info.data.real_env['professor_traj_time'][ii] - temp_t0
            if dt == 0:
                expert_curve_rate = 0
            else:
                expert_curve_rate = (expert_traj_slice.curvature - expert_curve0)/dt

            if expert_curve_rate_max < abs(expert_curve_rate):
                expert_curve_rate_max = abs(expert_curve_rate)
                t_expert_curve_rate_max = self.env_info.data.real_env['professor_traj_time'][ii] - t0
            
            expert_curve0 = expert_traj_slice.curvature
            temp_t0 = self.env_info.data.real_env['professor_traj_time'][ii]
            ii += 1


        # Cal TTC to lane:
        TTC_lane = list()
        min_dist2lane = list()
        t_min_dist2lane = list()

        if self.env_info != None:
            i_start = 0
            
            for ii in range(len(plan_path_info)):
                i_start = ii
                if ii < self.evaluation_init:
                    continue 
                info_slice = self.cal_TTC(plan_path_info[ii], \
                                        self.env_info.data.real_env['left_lane_boundary'],\
                                        self.env_info.data.real_env['right_lane_boundary'],i_start)
                                
                TTC_lane.append(info_slice[0])
                min_dist2lane.append(info_slice[1])
                t_min_dist2lane.append(info_slice[2])
                
        
        # Cal TTC to RE:
        TTC_re = list()
        min_dist2re = list()
        t_min_dist2re = list()

        if self.env_info != None:
            for ii in range(len(plan_path_info)):
                if ii < self.evaluation_init:
                    continue 
                info_slice = self.cal_TTC(plan_path_info[ii], \
                                    self.env_info.data.real_env['left_road_edge'],\
                                    self.env_info.data.real_env['right_road_edge'],ii)
                TTC_re.append(info_slice[0])
                min_dist2re.append(info_slice[1])
                t_min_dist2re.append(info_slice[2])
                # print(min_dist2re)
                # break

        # Cal expert TTC and min dist:
        if self.env_info != None:
            expert_traj = self.env_info.data.real_env['professor_traj']
            road_edge_left = self.env_info.data.real_env['left_road_edge']
            road_edge_right = self.env_info.data.real_env['right_road_edge']
            dist1,index1 = self.cal_traj2traj_min_dist(expert_traj[self.evaluation_init:-1],road_edge_left)
            dist2,index2 = self.cal_traj2traj_min_dist(expert_traj[self.evaluation_init:-1],road_edge_right)

            expert2RE_min_dis = min(dist1,dist2)
            if expert2RE_min_dis == dist1:
                index_expert2RE_min_dis = index1
            else:
                index_expert2RE_min_dis = index2

        # print(len(plan_path_info), index_expert2RE_min_dis)

        t_expert2RE_min_dis = \
                    round(self.env_info.data.real_env['professor_traj_time'][index_expert2RE_min_dis]\
                                - t0,2)


        # Evaluate min TTC
        TTC_lane_min = 21
        ii = 0
        for TTC in TTC_lane:
            if TTC < TTC_lane_min:
                TTC_lane_min = TTC
                TTC_min_lane_rel_time = self.env_info.data.converted_data[ii+self.evaluation_init].time_stamp - t0
            ii += 1

        TTC_re_min = 21
        ii = 0
        for TTC in TTC_re:
            if TTC < TTC_re_min:
                TTC_re_min = TTC
                TTC_min_re_rel_time = self.env_info.data.converted_data[ii+self.evaluation_init].time_stamp - t0
            ii += 1  

        min_dist2re_min = 1e6
        ii = 0
        
        for min_dist in min_dist2re:
            if min_dist < min_dist2re_min:
                min_dist2re_min = min_dist
                t_min_dist2re_min = self.env_info.data.converted_data[ii+self.evaluation_init].time_stamp - t0
                t_min_dist2re_plan_min = t_min_dist2re[ii]
            ii += 1

        hit_frame_num = 0
        all_eval_frames_num = len(min_dist2re)
        for min_dist in min_dist2re:
            if min_dist == 0:
                hit_frame_num += 1
        print("all_eval_frames_num = ", all_eval_frames_num, "hit_frame_num = ", hit_frame_num)

        # Evaluate mean TTC_re  
        sum_count = 0
        TTC_sum = 0
        for TTC in TTC_re:
            if TTC >=20:
                continue
            TTC_sum += TTC
            sum_count += 1
        TTC_re_mean = 20

        if sum_count !=0:
            TTC_re_mean = TTC_sum/sum_count

        # Cal score:
        event_score       = 100
        re_score          = 20
        ttc_lane_score    = 12
        ttc_re_score      = 20 + 12
        jerk_score        = 10
        curve_rate_score  = 15
        mean_ttc_re_score = 20 
        handover_critical = False
        if (min_dist2re_min > expert2RE_min_dis - 0.3):
            re_dis_lost = 0
        elif (min_dist2re_min > expert2RE_min_dis - 0.8):
            re_dis_lost = re_score *(1 - (min_dist2re_min + 0.8 - expert2RE_min_dis)/0.5)
        else:
            re_dis_lost = re_score
    
        if min_dist2re_min < expert2RE_min_dis - 0.05 and t_min_dist2re_min< 0.2:
            handover_critical = True
            re_dis_lost += 20    

        event_score -= re_dis_lost

        if TTC_lane_min > 3:
            TTC_lane_lost = 0
        elif TTC_lane_min > 1.5:
            TTC_lane_lost = ttc_lane_score*(1 - (TTC_lane_min - 1.5)/1.5)
        else:
            TTC_lane_lost = ttc_lane_score
        # if TTC_lane_min < 1.5 and TTC_min_lane_rel_time< 0.2:
            # TTC_lane_lost += 16 
            # handover_critical = True
        
        # event_score -= TTC_lane_lost

        if TTC_re_min > 5:
            TTC_re_lost = 0
        elif TTC_re_min > 2:
            TTC_re_lost = ttc_re_score*(1 - (TTC_re_min - 2)/3)
        else:
            TTC_re_lost = ttc_re_score

        event_score -= TTC_re_lost
        
        if max_curv_rate < expert_curve_rate_max + 0.02:
            curv_rate_lost = 0
        elif max_curv_rate < expert_curve_rate_max + 0.04:
            curv_rate_lost = curve_rate_score*((max_curv_rate - expert_curve_rate_max - 0.02)/0.02)
        else:
            curv_rate_lost = curve_rate_score
        event_score -= curv_rate_lost

        if max_jerk < expert_jerk_max + 0.3:
            jerk_lost = 0
        elif max_jerk < expert_jerk_max + 1:
            jerk_lost =  jerk_score*((max_jerk - expert_jerk_max - 0.3)/0.7)
        else:
            jerk_lost = jerk_score
        event_score -= jerk_lost


        if TTC_re_mean >= 7:
            ttc_re_mean_lost = 0
        elif TTC_re_mean >= 4:
            ttc_re_mean_lost = mean_ttc_re_score*(1-(TTC_re_mean-4)/3)
        else:
            ttc_re_mean_lost = mean_ttc_re_score
        event_score -= ttc_re_mean_lost

        event_score = max(event_score, 0)


        # Conclude evaluation results:
        self.res_table['summary score'] = [round(event_score,2)]

        self.res_table['MAX CURV RATE'] = ["{} ({}s at {}s) vs {} ({}s)".format\
            (round(max_curv_rate,5), round(rel_time_curvrate,2), round(t_curvrate,2),\
                round(expert_curve_rate_max,5), round(t_expert_curve_rate_max,2))]

        self.res_table['MAX LAT JERK'] = ["{} ({}s at {}s) vs {} ({}s)".format\
            (round(max_jerk,5), round(rel_time_jerk,2), round(t_jerk,2), \
                round(expert_jerk_max,5), t_expert_jerk_max)]
        
        if event_score >= 60:
            self.res_table['PLANNING SUCCESS'] = [True]
        else:
            self.res_table['PLANNING SUCCESS'] = [False]

        if min_dist2re_min <= 0:
            self.res_table['min dis2RE (PvE)'] = ["Hit RE ({}s-{}s) vs {}m ({}s)".format\
                    (round(t_min_dist2re_min,2),round(t_min_dist2re_plan_min, 2), round(expert2RE_min_dis,3),round(t_expert2RE_min_dis,2))] 
        else:
            self.res_table['min dis2RE (PvE)'] = ["{}m ({}s) vs {}m ({}s)".format\
                    (round(min_dist2re_min,2),round(t_min_dist2re_min,2),\
                        round(expert2RE_min_dis,2),round(t_expert2RE_min_dis,2))] 

                    # t_expert2RE_min_dis,expert2RE_min_dis
        if TTC_lane_min >= 20:
            self.res_table['min TTC Lane'] = ["No Lane collision"]   
        else:
            self.res_table['min TTC Lane'] = ["{}s,  TTC: {}s".format\
                (round(TTC_min_lane_rel_time,2),  round(TTC_lane_min,2))]      

        if TTC_re_min>=20:
            self.res_table['min TTC RE'] = ["No RE collision [{}/{}]".format\
                (hit_frame_num, all_eval_frames_num)]   
        else:
            self.res_table['min TTC RE'] = ["{}s,  TTC: {}s [{}/{}]".format\
                (round(TTC_min_re_rel_time,2), round(TTC_re_min,2), hit_frame_num, all_eval_frames_num)]    

        self.res_table['mean re TTC'] = [round(TTC_re_mean,5)]

        # self.res_table['MAX CURV RATE'] = [0.0,0.0]
        self.res_table['more infomation'] = [round(re_dis_lost,1), " ",  \
                                            round(TTC_re_lost,1), " ", \
                                            round(TTC_lane_lost,1), " ", \
                                            round(curv_rate_lost,1), " ", \
                                            round(jerk_lost,1), " ", \
                                            round(ttc_re_mean_lost,1), " ", \
                                                handover_critical]
        # pass

    def preprocess_plan_path(self):
        pass


    def cal_curv_rate(self):
        # calculate the curvature_rate within 1s lookahead
        pp_curv_rates = []
        plan_path_info = self.plan_path_info.data.converted_data
        if len(plan_path_info) == 0:
            return pp_curv_rates
        
        for ii in range(len(plan_path_info)):
            # print(ii, plan_path_info[ii].time_array)
            if plan_path_info[ii].time_array == []:
                continue
            t0 = plan_path_info[ii].time_array[0]
            pp_curv_rate = []
            pp_curv_rate.append(0.0)
            if len(plan_path_info[ii].time_array) <=1:
                return pp_curv_rate
            for index in range(len(plan_path_info[ii].time_array)-1):
                jj = index + 1
                t_rel = plan_path_info[ii].time_array[jj] - t0
                
                if t_rel > 1:
                    break
                
                d_curve = (plan_path_info[ii].plan_curv[jj] - plan_path_info[ii].plan_curv[jj-1])
                d_t = (plan_path_info[ii].time_array[jj] - plan_path_info[ii].time_array[jj-1])
                if d_t == 0:
                    break
                curv_rate = d_curve/d_t
                pp_curv_rate.append(abs(curv_rate))
            pp_curv_rates.append(pp_curv_rate)
        
        return pp_curv_rates
        
    def cal_curv_expert(self):
        pass

    def cal_lat_acc_expert(self):
        pass

    def cal_curv_rate_1s_max(self):
        pp_curv_rate_1s_max = []
        plan_path_info = self.plan_path_info.data.converted_data
        if len(plan_path_info) == 0:
            return pp_curv_rate_1s_max

        for ii in range(len(plan_path_info)):
            if plan_path_info[ii].time_array == []:
                continue
            t0 = plan_path_info[ii].time_array[0]    
            pp_match_index = -1
            for jj in range(len(plan_path_info[ii].time_array)):
                t_rel = plan_path_info[ii].time_array[jj] - t0
                if t_rel >1.0:
                    pp_match_index = jj
                    break
            if pp_match_index != -1:
                pp_curv_rate_1s_max.append( \
                    max( list(map(abs, plan_path_info[ii].plan_curv[0:jj])) )\
                )
            else:
                pp_curv_rate_1s_max.append(0)
        
        return pp_curv_rate_1s_max


    def cal_curvature(self):
        pp_curve = []
        plan_path_info = self.plan_path_info.data.converted_data

        if len(plan_path_info) == 0:
            return pp_curve

        for ii in range(len(plan_path_info)):
            if plan_path_info[ii].time_array == []:
                continue
            t0 = plan_path_info[ii].time_array[0]
            pp_match_index = -1
            for jj in range(len(plan_path_info[ii].time_array)):
                t_rel = plan_path_info[ii].time_array[jj] - t0
                
                if t_rel <= 0.1:
                    pp_match_index = jj
                    break
            
            if pp_match_index != -1:
                pp_curv_init = plan_path_info[ii].plan_curv[pp_match_index]
            else:
                pp_curv_init = 0.0
            
            pp_curve.append(pp_curv_init)
            self.res_table['INDEX 2'] = pp_curve
        return pp_curve      



if __name__ == "__main__":
    print('Start to evaluate...')
    pass
    # plan_path_output = PlanPathOutput()
    # data = dict(x=1,y=2)
    # plan_path_output.load_data(data)

    # cp_evaluator = CPEvaluator(plan_path_output)
