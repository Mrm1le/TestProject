# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod
# import rosbag
import time
import math
import numpy as np
import json
import sys
import os
import argparse

import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool
from bokeh.io import output_notebook, push_notebook,output_file,export_png
from bokeh.layouts import layout, column
from bokeh.plotting import figure, output_file, show, ColumnDataSource

from plotter.basic_layers import PointsLayer, MultiLinesLayer, MultiPolygonLayer, CurveLayer, MultiLinesLayer, MultiArcsLayer,\
     TextLabelLayer, MultiWedgesLayer,CircleLayer,LanesLayer, TriangleLayer

from plotter.event_tools import MeasureTools

from plotter.load_json_cp import JsonRefline,JsonQuadGenerator,JsonPlanPathGenerator,JsonLaneGenerator,JsonAllLanesGenerator,\
    JsonEgoGenerator,JsonRERealGenerator,JsonObjGenerator,JsonTextGenerator
from plotter.cp_visualizer_config import box_params, circle_params, sample_params,planning_path_params,lanes_param,clane_param,\
    re_params_l,ego_params,triangle_params,lane_real_params,expert_param,obj_params,cfb_params
from bokeh.models import ColumnDataSource, TableColumn, DataTable
from bokeh.layouts import column, row


GEO_TYPE = ["4"]               # display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon
WIDTH = 800
HEIGHT = WIDTH
LOCAL = True
html_file = "/home/ros/Downloads/visu/simu_visu_bag/jsons.html"


def visualize(html_file, layouts):
    output_file(html_file)
    bkp.show(layout(layouts))

def build_result_table(layouts,result_table):
    source = ColumnDataSource(result_table)
    # print(result_table.keys())
    columns = [
        TableColumn(field='test case', title='Test Case', width=1600),
        TableColumn(field='summary score', title='Summary Score',width=350),
        TableColumn(field='PLANNING SUCCESS', title='Success', width=200),
        TableColumn(field='min dis2RE (PvE)', title='min Distance to RE (PvE)', width=800),
        TableColumn(field='min TTC RE', title='min TTC RE', width=500),
        TableColumn(field='min TTC Lane', title='min TTC Lane', width=350),
        TableColumn(field='MAX CURV RATE', title='max Curve Rate (PvE)', width=850),
        TableColumn(field='MAX LAT JERK', title='max Lat Jerk (PvE)', width=850),
        TableColumn(field='mean re TTC', title='Mean RE TTC', width=250),
        TableColumn(field='more infomation', title='More Infomation', width=800),
        ]
    myTable = DataTable(source=source, columns=columns, width=1800, height=200)
    layouts.append(myTable)
    return layouts

class slider_callback_arg():
    def __init__(self, bag_data):
        self.arg = dict()
        self.arg['bag_source'] = bag_data
    
    def AddSource(self, arg_name, layer):
        self.arg[arg_name] = layer.data_source

class LayerManager():
    def __init__(self):
        self.layers = dict()
        self.gds = dict()
        self.data_key = dict()
        self.plotdim = dict()
        self.code = """
            %s
            """
        
    def AddLayer(self, newLayer, layer_label, gd, data_key = None, plotdim = None):
        self.layers[layer_label] = newLayer
        self.plotdim[layer_label] = plotdim

        if gd.xys == []:
            return

        self.gds[layer_label]    = gd

        if gd and data_key and plotdim:
            self.data_key[layer_label] = data_key
            layer_code = """%s
            var {}_index = binarySearch(data['{}ts'][0],data['mt'][0]+step); 
            {}.data['pts_xs'] = data['{}s'][0][{}_index][0];
            {}.data['pts_ys'] = data['{}s'][0][{}_index][1];
            """.format(data_key,data_key, layer_label,data_key, data_key, layer_label,data_key, data_key)

            if plotdim == 3 and not hasattr(gd, 'txt'):
                layer_append = """%s
            {}.data['pts_rs'] = data['{}s'][0][{}_index][2];
            """.format(layer_label,data_key, data_key)
                layer_code = (layer_append)%(layer_code)

            elif plotdim == 3 and hasattr(gd, 'txt'):
                layer_append = """%s
            {}.data['texts'] = data['{}s'][0][{}_index][2];
            """.format(layer_label,data_key, data_key)
                layer_code = (layer_append)%(layer_code)

            else:
                pass    

            layer_append = """%s{}.change.emit();
            console.log({}_index);
            """.format(layer_label,data_key)
            layer_code = (layer_append)%(layer_code)
            
            self.code = (layer_code)%(self.code)


class CPVisualizer(object):
    def __init__(self, plan_path_info, plan_output_info = None, env_info = None, input_type = ""):
        if "origin_data" not in plan_path_info.data.__dict__.keys():
            print("Error! Cannot run visualizer")
            return 
        self.fig = list()
        # self.layouts = list()
        self.event_layout = column([])
        if input_type == "real":
            self.planner_inputs = plan_path_info.data.origin_data_real
        else:
            self.planner_inputs = plan_path_info.data.origin_data
        self.env_info       = env_info.data
        self.planner_output = plan_output_info
        self.event_name = plan_path_info.event_name
        self.build_visualization()

    def build_visualization(self):
        if self.env_info == None:
            return

        quad_g      = JsonQuadGenerator(self.planner_inputs)
        ego_g       = JsonEgoGenerator(self.planner_inputs)
        vel_txt_g   = JsonTextGenerator(self.planner_inputs,"ego speed")
        frame_txt_g = JsonTextGenerator(self.planner_inputs,"frame number")
        pp_g        = JsonPlanPathGenerator(self.planner_output.data.origin_data)

        lane_l_g = JsonLaneGenerator(self.env_info.converted_data,"lane_point_left")
        lane_r_g = JsonLaneGenerator(self.env_info.converted_data,"lane_point_right")

        re_l_g        = JsonLaneGenerator(self.env_info.converted_data,"re_left")
        re_r_g        = JsonLaneGenerator(self.env_info.converted_data,"re_right")
        lane_all_g    = JsonAllLanesGenerator(self.env_info.converted_data,"all_lanes")
        re_real_r_g   = JsonRERealGenerator(self.env_info,"right_road_edge")
        re_real_l_g   = JsonRERealGenerator(self.env_info,"left_road_edge")
        lane_real_r_g = JsonRERealGenerator(self.env_info,"right_lane_boundary")
        lane_real_l_g = JsonRERealGenerator(self.env_info,"left_lane_boundary") 
        expert_traj_g = JsonRERealGenerator(self.env_info,"professor_traj")   

        obj_g         = JsonObjGenerator(self.env_info.converted_data,"all_objs")
        fig = bkp.figure(title=self.event_name,
                            x_axis_label='x',
                            y_axis_label='y',
                            match_aspect=True,
                            width=WIDTH,
                            height=HEIGHT)
        fig.toolbar.active_scroll = fig.select_one(WheelZoomTool)

        fig.background_fill_color = 'midnightblue'
        # fig.border_fill_color = 'midnightblue'
        fig.xgrid.grid_line_color = 'grey'
        fig.xgrid.grid_line_dash = 'dotdash'
        fig.ygrid.grid_line_color ='grey'
        fig.ygrid.grid_line_dash = 'dotdash'

        MeasureTools(fig)

        # +
        quad_layer          = CircleLayer(fig, circle_params)
        plan_res_layer      = CurveLayer(fig, planning_path_params)
        lane_l_layer        = CurveLayer(fig, clane_param)
        lane_r_layer        = CurveLayer(fig, clane_param)
        all_lanes_layer     = MultiLinesLayer(fig,clane_param)

        re_l_layer        = CurveLayer(fig, re_params_l)
        re_r_layer        = CurveLayer(fig, re_params_l)

        re_real_l_layer        = TriangleLayer(fig, triangle_params)
        re_real_r_layer        = TriangleLayer(fig, triangle_params)

        lane_real_l_layer        = CircleLayer(fig, lane_real_params)
        lane_real_r_layer        = CircleLayer(fig, lane_real_params)

        ego_layer         = MultiPolygonLayer(fig, ego_params)
        expert_layer      = CircleLayer(fig, expert_param)
        obj_layer         = MultiPolygonLayer(fig, obj_params)
        ego_speed_layer   = TextLabelLayer(fig, cfb_params)
        frame_txt_layer   = TextLabelLayer(fig, cfb_params)
        # add local_plot
        fig2 = bkp.figure(title="local_view",
                            x_axis_label='x',
                            y_axis_label='y',
                            match_aspect=True,
                            width=int(WIDTH/2),
                            height=HEIGHT,
                            x_range=[30, -30],
                            y_range=[-25,120]
                            )

        MeasureTools(fig2)
        fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
    
        # fig3 = bkp.figure(title = "refline_heading",
        #                     x_axis_label='s_index',
        #                     y_axis_label='heading /rad',
        #                     match_aspect=True,
        #                     width=int(WIDTH/2),
        #                     height=int(HEIGHT/4),
        #                     x_range=[0, 150],
        #                     y_range=[-0.2,0.2]
        #                     )          

        ##--------------------------------------- done build layers ------------------------------------------##
        ##------------------------------ link all objects in layer manager -----------------------------------##

        # layer_manager.AddLayer(pred_l_layer,'predl_source', predg_l,'predl',2)
        layer_manager = LayerManager()

        layer_manager.AddLayer(re_real_l_layer, 're_real_l_source',re_real_l_g,'re_real_l')
        layer_manager.AddLayer(re_real_r_layer, 're_real_r_source',re_real_r_g,'re_real_r')
        layer_manager.AddLayer(lane_real_l_layer, 'lane_real_l_source',lane_real_l_g,'lane_real_l')
        layer_manager.AddLayer(lane_real_r_layer, 'lane_real_r_source',lane_real_r_g,'lane_real_r')
        layer_manager.AddLayer(expert_layer, 'expert_source', expert_traj_g,'expert')

        layer_manager.AddLayer(quad_layer,'quad_source',quad_g,'quad',3)
        layer_manager.AddLayer(plan_res_layer,'pp_source',pp_g,'plan',2)
        layer_manager.AddLayer(lane_l_layer, 'lane_l_source',lane_l_g,'lane_l',2)
        layer_manager.AddLayer(lane_r_layer, 'lane_r_source',lane_r_g,'lane_r',2)
        
        layer_manager.AddLayer(re_l_layer, 're_l_source',re_l_g,'re_l',2)
        layer_manager.AddLayer(re_r_layer, 're_r_source',re_r_g,'re_r',2)
        layer_manager.AddLayer(all_lanes_layer, 'all_lane_source',lane_all_g,'all_lane',2)
        layer_manager.AddLayer(ego_layer, 'ego_source',ego_g,'ego',2)
        layer_manager.AddLayer(obj_layer, 'obj_source',obj_g,'obj_box',2)
        layer_manager.AddLayer(ego_speed_layer, 'ego_txt_source',vel_txt_g,'ego_vel',3)

        layer_manager.AddLayer(frame_txt_layer, 'frame_txt_source', frame_txt_g,'frame_txt',3)

        ##------------------------------ done link all objects in layer manager ---------------------------------##
        min_t = sys.maxsize
        max_t = 0
        min_f = 0
        max_f = 0
        for gdlabel in layer_manager.gds.keys():
            gd = layer_manager.gds[gdlabel]
            min_t = min(min_t, gd.getMinT())
            max_t = max(max_t, gd.getMaxT())    
            # min_f = min(min_f, len(gd.xys))
            max_f = max(max_f, len(gd.xys))
            

            # min_t = min(min_t, min(gd.ts))
            # max_t = max(max_t, max(gd.ts)) 

        data_tmp = {'mt':[min_t]}

        for gdlabel in layer_manager.data_key.keys():
            gd = layer_manager.gds[gdlabel]
            data_label = layer_manager.data_key[gdlabel]
            data_tmp[data_label+'s'] = [gd.xys]
            data_tmp[data_label+'ts'] = [gd.ts]
    
        bag_data = ColumnDataSource(data = data_tmp)
        
        callback_arg = slider_callback_arg(bag_data)

        for layerlabels in layer_manager.layers.keys():
            callback_arg.AddSource(layerlabels, layer_manager.layers[layerlabels])

        # find the front one of ( the first which is larger than k)
        binary_search = """
                function binarySearch(ts, k){
                    if(ts.length == 0){
                        return 0;
                    }
                    var left = 0;
                    var right = ts.length -1;
                    while(left<right){
                        var mid = Math.floor((left + right) / 2);
                        if(ts[mid]<=k){
                            left = mid + 1;
                        }else{
                            right = mid;
                        }
                    }
                    if(left == 0){
                        return 0;
                    }
                    return left-1;
                }
        """
        # print(min_t, max_t)
        # print(max_t - min_t)
        car_slider = Slider(start = 0, end = max_t-min_t, value = 0, step = 0.01, title = "time")
        # car_slider = Slider(start = 0, end = max_f-min_f, value = 0, step = 1, title = "frame")

        code0 = """
        %s
                const step = cb_obj.value;
                const data = bag_source.data;
        """

        codes = (layer_manager.code)%(code0)%(binary_search)
        callback = CustomJS(args=callback_arg.arg, code = codes)

        car_slider.js_on_change('value', callback)

        for gdlabel in layer_manager.gds.keys():
            gd = layer_manager.gds[gdlabel]
            if gdlabel in ['ep_source', 're_real_l_source', 're_real_r_source',\
                             'lane_real_l_source', 'lane_real_r_source','expert_source']:
                gd_frame = gd.atT(max_t)
            else:
                gd_frame = gd.atT(min_t)
            if gdlabel in ['online_obj_source','onlinel_obj_source','re_real_l_source', \
                            're_real_r_source','lane_real_l_source', 'lane_real_r_source','expert_source']:
                layer_manager.layers[gdlabel].update(gd_frame[0],gd_frame[1],gd_frame[2])
            elif gdlabel is 'cfb_source':
                pass
            else:
                if layer_manager.plotdim[gdlabel] == 3: 
                    layer_manager.layers[gdlabel].update(gd_frame[0],gd_frame[1],gd_frame[2])
                else:
                    layer_manager.layers[gdlabel].update(gd_frame[0],gd_frame[1])
        

        # print(fig.renderers[1])
        # print(fig2.renderers[1])
        # print(fig3.renderers)
        
        fig.legend.click_policy="hide"

        # self.build_result_table()
        
        self.fig.append(fig)
        self.event_layout = column(car_slider,row(self.fig))

        