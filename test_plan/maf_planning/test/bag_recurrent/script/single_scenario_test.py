import json
import time

import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import tf2d
import os
from tf2d import Point2D as Point2D
import copy
import parse_bagjson_worldmodel as pbw
import shutil

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan

if(os.path.exists('../build/combined_trajectory.txt')):
  os.remove('../build/combined_trajectory.txt')
if(os.path.exists('../build/escape_path.txt')):
  os.remove('../build/escape_path.txt')
if(os.path.exists('../build/out_slot_path.txt')):
  os.remove('../build/out_slot_path.txt')
if(os.path.exists('../build/rs_path.txt')):
  os.remove('../build/rs_path.txt')
if(os.path.exists('../build/outslot_escape_path.txt')):
  os.remove('../build/outslot_escape_path.txt')
# shutil.rmtree('../result')
# os.mkdir('../result')
# save_success_plot = False #是否保存成功场景，默认关闭
# save_fail_plot = True

if __name__ == '__main__':
    min_filename_num = 0
    max_filename_num = 0
    if len(sys.argv) == 3:
        min_filename_num = sys.argv[1]
        max_filename_num = sys.argv[2]
    elif len(sys.argv) == 2:  
        filename_num = sys.argv[1] 
    else:
      print("请提供1个或2个JSON文件名序号")  
      sys.exit(1) 
    path_base = "../scenario/"
    file_name_base = "new_"
    if max_filename_num > min_filename_num:
        for filename_num in range(int(min_filename_num), int(max_filename_num)):
            file_path_base = path_base + file_name_base + str(filename_num) + ".json"
            current_dir = os.getcwd()
            file_name = path_base + file_name_base + str(filename_num) + ".json"
            with open(file_name) as f:
                data_sbp = json.load(f)

            vehicle_start_x = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["x"]
            vehicle_start_y = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["y"] 
            vehicle_start_theta = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["theta"] 
            slot_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["x"]
            slot_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["y"]

            data = pbw.ParseBagJson() 
            data.init(file_name)
            data.file_name = file_name_base + str(filename_num) + ".json"
            print(data.file_name)
            success = data.plot_trajectory(data.target.x, data.target.y, data.target.theta)

            if (success) :
                vs_x = vehicle_start_x
                vs_y = vehicle_start_y
                ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
                ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
                print("sucess:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
                print ("success:", math.cos(vehicle_start_theta), math.sin(vehicle_start_theta))
                # plt.ion()
                data.plot_obstacle_box()
                data.plot_points()
                data.plot_lines()
                data.plot_obstacle_lines()   
                data.plot_car(data.target.x, data.target.y, data.target.theta)
                data.plot_car(data.init_point.x, data.init_point.y, data.init_point.theta, True)
                plt.xlim((slot_start_x - 15, slot_start_x + 15))
                plt.ylim((slot_start_y - 15, slot_start_y + 15))
                if(os.path.exists('../build/out_slot_path.txt')):
                  trajectory = np.loadtxt('../build/out_slot_path.txt', delimiter=' ')  
                  # 提取x和y坐标  
                  # 提取x和y坐标  
                  if trajectory.size > 3:
                    x = trajectory[:, 0]  
                    y = trajectory[:, 1]  
                    # 绘制散点图  
                    plt.plot(x, y, 'r', linewidth = 0.25) 
                if(os.path.exists('../build/rs_path.txt')):
                  trajectory = np.loadtxt('../build/rs_path.txt', delimiter=' ')  
                  # 提取x和y坐标  
                  if trajectory.size > 3:
                    x = trajectory[:, 0]  
                    y = trajectory[:, 1]  
                    # 绘制散点图  
                    plt.plot(x, y, 'k', linewidth = 0.25) 
                if(os.path.exists('../build/rubbing_path.txt')):
                  trajectory_rb = np.loadtxt('../build/rubbing_path.txt', delimiter=' ')  
                  # 提取x和y坐标
                  if trajectory_rb.size > 3:  
                    x = trajectory_rb[:, 0]  
                    y = trajectory_rb[:, 1]  
                    # 绘制散点图  
                    plt.plot(x, y, 'm', linewidth = 0.25)
                if(os.path.exists('../build/outslot_escape_path.txt')):
                  trajectory_b = np.loadtxt('../build/outslot_escape_path.txt', delimiter=' ')  
                  # 提取x和y坐标  
                  if trajectory_b.size > 3:
                    x = trajectory_b[:, 0]  
                    y = trajectory_b[:, 1]  
                  # 绘制散点图  
                    plt.plot(x, y, 'k', linewidth = 0.25)
                if(os.path.exists('../build/escape_path.txt')):
                  trajectory = np.loadtxt('../build/escape_path.txt', delimiter=' ')  
                  # 提取x和y坐标  
                  if trajectory.size > 3:
                    x = trajectory[:, 0]  
                    y = trajectory[:, 1]  
                    # 绘制散点图  
                    plt.plot(x, y, 'c', linewidth = 0.25) 

                plt.savefig(file_name_base + str(filename_num) + ".png", dpi=1200)
                # plt.show()

                # plt.ioff()   
            else :
                vs_x = vehicle_start_x
                vs_y = vehicle_start_y
                ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
                ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
                print("fail:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
                print ("fail:", math.cos(vehicle_start_theta), math.sin(vehicle_start_theta))
                plt.ion()
                data.plot_obstacle_box()
                data.plot_points()
                data.plot_lines()
                data.plot_obstacle_lines()   
                data.plot_car(data.target.x, data.target.y, data.target.theta)
                data.plot_car(data.init_point.x, data.init_point.y, data.init_point.theta, True)
                plt.xlim((slot_start_x - 15, slot_start_x + 15))
                plt.ylim((slot_start_y - 15, slot_start_y + 15))
                # plt.show()
                plt.savefig(file_name_base + str(filename_num) + ".png", dpi=1200)
                # plt.ioff()       
            
            print ("finish")
            print ("*******************************************")
    else:
        file_path_base = path_base + file_name_base + filename_num + ".json"
        current_dir = os.getcwd()
        file_name = path_base + file_name_base + filename_num + ".json"
        with open(file_name) as f:
            data_sbp = json.load(f)

        vehicle_start_x = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["x"]
        vehicle_start_y = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["y"] 
        vehicle_start_theta = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["theta"] 
        slot_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["x"]
        slot_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["y"]

        data = pbw.ParseBagJson() 
        data.init(file_name)
        data.file_name = file_name_base + filename_num + ".json"
        print(data.file_name)
        success = data.plot_trajectory(data.target.x, data.target.y, data.target.theta)

        if (success) :
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("sucess:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
            print ("success:", math.cos(vehicle_start_theta), math.sin(vehicle_start_theta))
            # plt.ion()
            data.plot_obstacle_box()
            data.plot_points()
            data.plot_lines()
            data.plot_obstacle_lines()   
            data.plot_car(data.target.x, data.target.y, data.target.theta)
            data.plot_car(data.init_point.x, data.init_point.y, data.init_point.theta, True)
            plt.xlim((slot_start_x - 15, slot_start_x + 15))
            plt.ylim((slot_start_y - 15, slot_start_y + 15))
            
            if(os.path.exists('../build/out_slot_path.txt')):
              trajectory = np.loadtxt('../build/out_slot_path.txt', delimiter=' ')  
              # 提取x和y坐标  
              # 提取x和y坐标  
              if trajectory.size > 3:
                x = trajectory[:, 0]  
                y = trajectory[:, 1]  
                # 绘制散点图  
                plt.plot(x, y, 'r', linewidth = 0.25) 
            if(os.path.exists('../build/rs_path.txt')):
              trajectory = np.loadtxt('../build/rs_path.txt', delimiter=' ')  
              # 提取x和y坐标  
              if trajectory.size > 3:
                x = trajectory[:, 0]  
                y = trajectory[:, 1]  
                # 绘制散点图  
                plt.plot(x, y, 'k', linewidth = 0.25) 
            if(os.path.exists('../build/rubbing_path.txt')):
              trajectory_rb = np.loadtxt('../build/rubbing_path.txt', delimiter=' ')  
              # 提取x和y坐标
              if trajectory_rb.size > 3:  
                x = trajectory_rb[:, 0]  
                y = trajectory_rb[:, 1]  
                # 绘制散点图  
                plt.plot(x, y, 'm', linewidth = 0.25)
            if(os.path.exists('../build/outslot_escape_path.txt')):
              trajectory_b = np.loadtxt('../build/outslot_escape_path.txt', delimiter=' ')  
              # 提取x和y坐标  
              if trajectory_b.size > 3:
                x = trajectory_b[:, 0]  
                y = trajectory_b[:, 1]  
              # 绘制散点图  
                plt.plot(x, y, 'k', linewidth = 0.25)
            if(os.path.exists('../build/escape_path.txt')):
              trajectory = np.loadtxt('../build/escape_path.txt', delimiter=' ')  
              # 提取x和y坐标  
              if trajectory.size > 3:
                x = trajectory[:, 0]  
                y = trajectory[:, 1]  
                # 绘制散点图  
                plt.plot(x, y, 'c', linewidth = 0.25) 
            plt.savefig(file_name_base + filename_num + ".png", dpi=1200)
              # plt.show()

            # plt.ioff()   
        else :
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("fail:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
            print ("fail:", math.cos(vehicle_start_theta), math.sin(vehicle_start_theta))
            plt.ion()
            data.plot_obstacle_box()
            data.plot_points()
            data.plot_lines()
            data.plot_obstacle_lines()   
            data.plot_car(data.target.x, data.target.y, data.target.theta)
            data.plot_car(data.init_point.x, data.init_point.y, data.init_point.theta, True)
            plt.xlim((slot_start_x - 15, slot_start_x + 15))
            plt.ylim((slot_start_y - 15, slot_start_y + 15))
            # plt.show()
            plt.savefig(file_name_base + filename_num + ".png", dpi=1200)
            # plt.ioff()       
        
        print ("finish")
        print ("*******************************************")    