# -*- coding: utf-8 -*-

import sys
import os
import math
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../../lib'))
import plotter.utils.common as comm
from config import conf

MAX_SCORE = 100
PASS_SCORE = 60
MIN_SCORE = 0


def r2(score):
    return round(score, 2)

def getVecDistance(x, y):
    return pow(pow(x, 2) + pow(y, 2), 0.5)


class CaseBase(object):
    def __init__(self):
        self.name = ''

    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, 
        expert_min_distance=None, plan_min_distance=None):
        return 0, ''


class CaseAPASucc(CaseBase):
    def __init__(self):
        super().__init__()
        self.name = "success"
        self.must_pass = True

    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, 
        expert_min_distance=None, plan_min_distance=None):
        score = 100      
        more_info="true"
        if len(plan_result["x"]) == 0 or len(plan_result["y"]) == 0:
            score = 0
            more_info ="false" 
        
        return score, more_info 


class CaseAPAShiftGearTimes(CaseBase):
    def __init__(self):
        super().__init__()
        self.name = "shift gear times"

    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, 
        expert_min_distance=None, plan_min_distance=None):
        score = 0     

        if expert_result == None:
            more_info= 'invalid expert data.'
            return score, more_info
        expert_gear_times = comm.getReverseTime(expert_result['x'], expert_result['y'])

        if len(plan_result['x']) == 0:
            more_info= '{}:none'.format(expert_gear_times)
            return score, more_info
        plan_gear_times = comm.getReverseTime(plan_result['x'], plan_result['y'])
        
        more_info = "{}:{}".format(
            expert_gear_times, plan_gear_times)

        max_gear_times = conf['evaluation']['max_gear_times']
        if expert_gear_times > max_gear_times:
            more_info = 'expert gear times exceeded MAX: {}, score ZERO!'.format(
                max_gear_times)
            return 0, more_info
        
        if plan_gear_times <= expert_gear_times:
            return 100, more_info

        score = MAX_SCORE - (plan_gear_times - expert_gear_times) * \
            (MAX_SCORE - PASS_SCORE) / (max_gear_times - expert_gear_times)

        return score, more_info
        

class CaseAPALocation(CaseBase):
    def __init__(self):
        super().__init__()
        self.name ='parking location offset'


    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, 
        expert_min_distance=None, plan_min_distance=None):
        score = 0
        more_info = ''
        if len(plan_result["x"]) == 0:
            more_info="none"
            return score, more_info

        if target_point == None:
            more_info = 'invalid target point.'
            return score, more_info

        test_point = dict()
        test_point["x"] = plan_result["x"][0]
        test_point["y"] = plan_result["y"][0]
        test_point["phi"] = plan_result["phi"][0]

        #print("test_point: {}".format(test_point))
        #print("target_point: x: {}, y: {}, phi: {}".format(
        #    target_point.x, target_point.y, target_point.theta))

        theta_delta = abs(target_point.theta - test_point["phi"]) * 180 / math.pi

        point_distance = getVecDistance(test_point["x"] - target_point.x, \
            test_point["y"] - target_point.y)

        vertical_offset = math.cos(theta_delta) * point_distance
        parallel_offset = math.sin(theta_delta) * point_distance

        vertical_score = MAX_SCORE - \
            vertical_offset * ((MAX_SCORE - PASS_SCORE) / \
            conf['evaluation']['max_vert_offset'])
        parallel_score = MAX_SCORE - \
            parallel_offset * ((MAX_SCORE - PASS_SCORE) / \
            conf['evaluation']['max_para_offset'])
        theta_score = MAX_SCORE - \
            theta_delta * ((MAX_SCORE - PASS_SCORE) / \
            conf['evaluation']['max_theta_offset'])
        

        more_info = "offset P: {} m, V: {} m, theta: {}(angle)".format(
            r2(parallel_offset), r2(vertical_offset), r2(theta_delta / math.pi * 180))
        score = min(vertical_score, parallel_score, theta_score)

        return score, more_info


class CaseAPAObsMinDistance(CaseBase):
    def __init__(self):
        super().__init__()
        self.name = 'obstacle min distance'
    

    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, 
        expert_min_distance=None, plan_min_distance=None):

        if target_point == None:
            return 0, 'invalid target point'

        if len(plan_result['x']) == 0:
            plan_min_distance = 0

        more_info = ' {}:{}'.format(
            expert_min_distance, plan_min_distance)

        if plan_min_distance >= expert_min_distance:
            return 100, more_info
        
        return 0, more_info


class Evaluator(object):
    def __init__(self):
        self.case_list = list()

    def init(self):
        self.case_list.append(CaseAPASucc())
        self.case_list.append(CaseAPAShiftGearTimes())
        self.case_list.append(CaseAPALocation())
        self.case_list.append(CaseAPAObsMinDistance())


    def evaluate(self, plan_result, expert_result=None, 
        target_point=None, odo=None, expert_min_distance=None, plan_min_distance=None):
        result = dict()
        result['summary score'] = 0
        result['more information'] = dict()

        lowest_score = 100

        for case in self.case_list:

            score, more_info = case.evaluate(
                plan_result, expert_result, target_point, odo,
                expert_min_distance, plan_min_distance)

            score = r2(score)
            score = max(0, score)
            score = min(100, score)

            result[case.name] = score
            lowest_score = min(score, lowest_score)
            result['more information'][case.name] = more_info
            

        result['summary score'] = lowest_score
            
        return result
        
evaluator = Evaluator()