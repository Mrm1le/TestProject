# -*- coding: utf-8 -*-


from json.tool import main
import sys
import os
import numpy as np
import json


from easydict import EasyDict

import ipywidgets
import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, Title
from bokeh.io import output_notebook, push_notebook, output_file
from bokeh.layouts import layout

from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, TableColumn, DataTable
from bokeh.io import show
import pandas as pd

from pyrsistent import v
from plotter.load_bag import DataLoader
from plotter.utils.common import GEO_TYPE
from plotter.event_tools import MeasureTools
from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, FrameLayer

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan


CAR_PARAM_RX5 = "../resources/vehicle_param_rx5.yaml"
CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
APA_PARALLEL = "../resources/apa_parallel.yaml"                  # apa file for parallel scenario
APA_VERTICAL = "../resources/apa_vertical.yaml"                  # apa file for vertical scenario
APA_VERTICAL_2ND = "../resources/apa_vertical_2nd.yaml"                  # apa file for vertical scenario 2

GEO_TYPE[0] = "6"                # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon

def readLabelDatas(json_file):
    '''
    read label_datas from label json file
    
    in: json file
    out: list of laebl datas > ObstacleLabelData
    '''
    if not os.path.exists(json_file):
        raise Exception('json file does not exist:{}'.format(json_file))
    
    load_dict = {}
    with open(json_file,'r') as load_f:
        load_dict = json.load(load_f)


    label_datas = []
    for ch in load_dict['children']:
        points_str = ch['points'].split()
        points_xyz = [tuple(map(float,p.split(','))) for p in points_str]
        
        label_points = []
        for p in points_xyz:
            label_points.append(apa_plan.LabelPoint(p[0], p[1]))

        if ch['data']['type'][0] == 'wall':
            label_datas.append(apa_plan.ObstacleLabelData(apa_plan.ObstacleType.WALL, label_points))

        elif ch['data']['type'][0] == 'car':
            label_datas.append(apa_plan.ObstacleLabelData(apa_plan.ObstacleType.CAR, label_points))

    return label_datas

def planOnce(init_pose, target_pose, label_datas, slot_type = apa_plan.ParkingSlotType.UNKNOWN):
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
    if slot_type == apa_plan.ParkingSlotType.UNKNOWN:
        slot_type = apa_plan.getSlotType(init_pose, target_pose)
    
    if slot_type == apa_plan.ParkingSlotType.PARALLEL:
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)
    else:
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_VERTICAL)
        
    res_str = apa_plan.perfectScenePlan(init_pose, target_pose, label_datas, slot_type)
    sbp_res_all = EasyDict(json.loads(res_str))
    sbp_res = sbp_res_all['res']
    odo = sbp_res_all['odo']
    
    car_size_params = EasyDict(json.loads(apa_plan.getCarSizeParams()))
    
    return (sbp_res, odo, car_size_params)

def batchPlan(slot_type = apa_plan.ParkingSlotType.UNKNOWN):
    pass


from bokeh.models import HTMLTemplateFormatter
def get_html_formatter(my_col):
    template = """
        <div style="background:<%= 
            (function colorfromint(){
                if(result_col == 'Positive'){
                    return('#f14e08')}
                else if (result_col == 'Negative')
                    {return('#8a9f42')}
                else if (result_col == 'Invalid')
                    {return('#8f6b31')}
                }()) %>; 
            color: white"> 
        <%= value %>
        </div>
    """.replace('result_col',my_col)
    
    return HTMLTemplateFormatter(template=template)

def get_html_formatter(my_col):
    template = """
        <div style="background:<%= 
            (function colorfromint(){
                if(result_col < 60){
                    return('#FF5151')}
                else if (result_col >= 80)
                    {return('#C2FF68')}
                else
                    {return('#FFFF6F')}
                }()) %>; 
            color: white"> 
        <%= value %>
        </div>
    """.replace('result_col',my_col)
    
    return HTMLTemplateFormatter(template=template)


def displayBatchRes(odo_list, res_pro_list, res_list, car_size_params_list, \
    plot_titles, texts_list, html_path, result_table = None):
    '''
    plot batch res in html
    
    in:
        odo_list: list of planner in
        res_list: list of planner out
        html_path: html path to display
    out: void
    '''

    figs = []
    
    if result_table != None:
        source = ColumnDataSource(result_table)
        columns = [
            TableColumn(field='test case', title='test case', width=500),
            TableColumn(field='summary score', title='summary score'),
            TableColumn(field='PARKING SUCCESS', title='PARKING SUCCESS', width=500),
            TableColumn(field='SHIFT GEAR TIMES', title='SHIFT GEAR TIMES', width=500),
            TableColumn(field='PARKING LOCATION', title='RPARKING LOCATION', width=500),
            TableColumn(field='more infomation', title='more infomation', width=2000)
            ]
        myTable = DataTable(source=source, columns=columns, width=1200, height=600)
        figs.append(myTable)

    column_num = 2
    output_file(html_path)
    for index in range(len(odo_list)):
        if index % column_num ==0:
            figs.append([])
            
        #plot_title = "plan "+ str(index)
        plot_title = plot_titles[index]

        # set layers
        fig = bkp.figure(title=plot_title,
                        x_axis_label='x',
                        y_axis_label='y',
                        match_aspect=True,
                        width=800,
                        height=600)
        
        titles = texts_list[index]
        for title in titles[::-1]:
            fig.add_layout(Title(text=title, text_font_size="10pt"), 'above')
        # fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)
        MeasureTools(fig)

        car_size_params = None
        if isinstance(car_size_params_list,list):
            car_size_params = car_size_params_list[index]
        else:
            car_size_params = car_size_params_list
        
        frame_layer = FrameLayer(fig)
        frame_layer.update(odo_list[index],res_pro_list[index], res_list[index], car_size_params)
        fig.legend.click_policy = 'hide'
        
        figs[-1].append(frame_layer.getLayout())

        
        # -
    # fig_all = list(map(list, zip(*figs)))  # transpose
    bkp.show(layout(figs), notebook_handle=True)



# ######################### help functions for test ##################################

def getLabelDatasFromBag(bag_file):
    '''
    make fake datas from bag
    '''
    label_datas = []
    p0 = apa_plan.Point2d()
    p1 = apa_plan.Point2d()

    SBP_REQUEST = "/msd/sbp_request"
    
    if not os.path.exists(bag_file):
        raise Exception('bag file does not exist:{}'.format(bag_file))
    
    dataloader = DataLoader(bag_file)
    request_msg = dataloader.getTopic(SBP_REQUEST)
    param_strs = []
    for msg in request_msg:
        if msg[0].task_info.goal_id.id != "search_problem":
            continue
        param_strs.append(msg[0].task_config.problem_config.params_string)
        
    print("read request size:", len(param_strs))
    if len(param_strs) == 0:
        return label_datas
    
    param_json  = json.loads(param_strs[0])
    temp_odo = EasyDict(param_json["OpenspaceDeciderOutput"])
    
    p0.x = temp_odo.target_state.path_point.x
    p0.y = temp_odo.target_state.path_point.y
    p0.theta = temp_odo.target_state.path_point.theta
    p1.x = temp_odo.init_state.path_point.x
    p1.y = temp_odo.init_state.path_point.y
    p1.theta = temp_odo.init_state.path_point.theta
    
    label_points_wall = []
    for p in temp_odo.points:
        sp = apa_plan.LabelPoint(p.x_, p.y_)
        label_points_wall.append(sp)
    for l in temp_odo.obstacle_lines:
        label_points_wall.append(apa_plan.LabelPoint(l.start_.x_, l.start_.y_))
        label_points_wall.append(apa_plan.LabelPoint(l.end_.x_, l.end_.y_))
        
    label_datas.append(apa_plan.ObstacleLabelData(apa_plan.ObstacleType.WALL, label_points_wall))

    for b in temp_odo.obstacle_boxs:
        label_points_car = []
        for p in b.corners_:
            sp = apa_plan.LabelPoint(p.x_, p.y_)
            label_points_car.append(sp)

        label_datas.append(apa_plan.ObstacleLabelData(apa_plan.ObstacleType.CAR, label_points_car))
    
    return (p0, p1, label_datas)

if __name__ == '__main__':
    # bag_file = "/home/ros/Downloads/apa_bag/0224/PLSIV106_recording_no_cam_RVIZ_20220222-152326_20220222-152819_0.bag"  
    # bag_file = "/home/ros/Downloads/apa_bag/0221/4516/PLSNV104_recording_rviz_104-9-7-5_20220219-222839_20220219-223017_1_gt_trimmed.bag" # no plan
    # bag_file = "/home/ros/Downloads/apa_bag/0221/5714/PLSNV104_recording_no_cam_RVIZ_104-10-1-7_20220220-114740_20220220-114855_0.bag"  # parallel
    p_bag_file = "/home/ros/Downloads/apa_bag/0225/PLSVV134_recording_APA_PARKIN_RECORD_no_cam_20220216-170312_20220216-170433_0.bag"  # parallel  right
    # bag_file = "/home/ros/Downloads/apa_bag/0217/PLSVV134_recording_APA_PARKIN_RECORD_no_cam_20220208-165115_20220208-165227_0_gt_trimmed.bag"  # vertical left
    v_bag_file = "/home/ros/Downloads/apa_bag/0228/PLAA75937_recording_no_cam_RVIZ_75937-5-4-4_20220225-233228_20220225-233339_0.bag"  # vertical left

    # bag_file = "/home/ros/Downloads/apa_bag/0221/5714/PLSNV104_recording_no_cam_RVIZ_104-10-1-7_20220220-114740_20220220-114855_0.bag"  # parallel left
    p_p0, p_p1, p_label_datas = getLabelDatasFromBag(p_bag_file)
    v_p0, v_p1, v_label_datas = getLabelDatasFromBag(v_bag_file)
    
    p_sbp_res, p_odo, p_car_size_params = planOnce(p_p0, p_p1, p_label_datas, apa_plan.ParkingSlotType.UNKNOWN)
    v_sbp_res, v_odo, v_car_size_params = planOnce(v_p0, v_p1, v_label_datas, apa_plan.ParkingSlotType.UNKNOWN)
    
    p_sbp_res_pro = p_sbp_res
    v_sbp_res_pro = v_sbp_res
    
    batch_num = 2
    odo_list = [p_odo, v_odo]
    res_list = [p_sbp_res, v_sbp_res]
    res_pro_list = [p_sbp_res_pro, v_sbp_res_pro]
    texts_list = [['haha', 'haha2'], ['haha', 'haha2']]
    plot_titles = ['plan0', 'plan1']

    html_path = "/home/ros/Downloads/perfect_scene.html"
    displayBatchRes(odo_list, res_pro_list, res_list, [p_car_size_params, v_car_size_params], plot_titles, texts_list, html_path)

    


