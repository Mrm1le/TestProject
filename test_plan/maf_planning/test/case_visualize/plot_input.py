#!/usr/bin/python

import sys
import yaml
import matplotlib.pyplot as plt
from math import cos, sin, hypot
import argparse
import os
class Vec2d:
  def __init__(self, x = 0, y = 0):
    self.x = x
    self.y = y

class Box:
  def __init__(self, x, y, heading, length, width):
    self.center = Vec2d(x, y)
    self.heading = heading
    self.length = length
    self.width = width

def rotate(x, y, theta):
  x_rotated = x * cos(theta) - y * sin(theta)
  y_rotated = x * sin(theta) + y * cos(theta)
  return x_rotated, y_rotated

def drawBox(axes, param_dict, box_data):
  corners_data = [
    [box_data.length/2, box_data.width/2],
    [-box_data.length/2, box_data.width/2],
    [-box_data.length/2, -box_data.width/2],
    [box_data.length/2, -box_data.width/2]
  ]
  for item in corners_data:
    item[0], item[1] = rotate(item[0], item[1], box_data.heading)
    item[0] += box_data.center.x
    item[1] += box_data.center.y
  
  x = [item[0] for item in corners_data]
  y = [item[1] for item in corners_data]
  x.append(corners_data[0][0])
  y.append(corners_data[0][1])

  out = axes.plot(x, y, **param_dict)
  return out

def plotCar(axes, axes_param_dict, traj_point, car_param, **kwargs):
  car_x = traj_point['path_point'][0]
  car_y = traj_point['path_point'][1]
  car_theta = traj_point['path_point'][2]
  car_front_length = car_param['front_edge_to_rear_real']
  car_back_length = car_param['vehicle_length'] - car_front_length
  car_width = car_param['vehicle_width']

  car_corners_data = [
    [car_front_length, car_width / 2],
    [-car_back_length, car_width / 2],
    [-car_back_length, -car_width / 2],
    [car_front_length, -car_width / 2]
  ]

  for corner in car_corners_data:
    corner[0], corner[1] = rotate(corner[0], corner[1], car_theta)
    corner[0] += car_x
    corner[1] += car_y

  car_xx = [item[0] for item in car_corners_data]
  car_yy = [item[1] for item in car_corners_data]
  car_xx.append(car_corners_data[0][0])
  car_yy.append(car_corners_data[0][1])
  
  out = axes.plot(car_xx, car_yy, axes_param_dict, label=kwargs['label'])
  return out

def plotAll(axes, data):
  if 'obstacle_lines' in data:
    obstacle_lines = data['obstacle_lines']
    if obstacle_lines is not None:
      for line in obstacle_lines:
        axes.plot([line['start_x'], line['end_x']], [line['start_y'], line['end_y']], 'r')

  if 'T_lines' in data:
    t_lines = data['T_lines']['lines']
    if t_lines is not None:
      for line in t_lines:
        axes.plot([line['start_x'], line['end_x']], [line['start_y'], line['end_y']], 'kp:', linewidth=2)
  
  if 'lines' in data:
    lines = data['lines']
    if lines is not None:
      for line in lines:
        axes.plot([line['start_x'], line['end_x']], [line['start_y'], line['end_y']], 'r--')
  
  obstacle_boxes = [data['map_boundary']]
  if data['obstacle_boxs'] is not None:
    obstacle_boxes.extend(data['obstacle_boxs'])
  for box in obstacle_boxes:
      drawBox(axes, {'color': 'g'}, Box(box['center_x'], box['center_y'], box['heading'], box['length'], box['width']))
  
  init_target_x = [data['init_state']['path_point'][0], data['target_state']['path_point'][0]]
  init_target_y = [data['init_state']['path_point'][1], data['target_state']['path_point'][1]]
  axes.plot(init_target_x, init_target_y, 'g+')

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("folder", help="the starting number of cases in the folder",
                      type = str, default = ".")
  parser.add_argument("-p", "--prefix", help="the prefix of file in the folder",
                      type = str, default = "")
  parser.add_argument("-m", "--mode",help = "choose whether to show of save image",
                      type = str, default = 'show')
  # parser.add_argument("-v", "--vehicle",help = "param file of car",
  #                     type = str, default = 'vehicle_param.yaml')
  parser.add_argument("-c", "--config",help = "config file of vehicle",
                      type = str, default = 'apa.yaml')
  args = parser.parse_args()

  folder = args.folder
  prefix = args.prefix
  mode = args.mode

  case_file_list = []
  if os.path.isfile(folder):
    case_file_list.append(folder)
  elif os.path.isdir(folder):
    for root, dirs, files in os.walk(folder, topdown=False):
      for f in files:
        case_file_list.append(folder+"/"+f)
  else:
    raise Exception("no such file or directory")

  case_file_list = [file_name for file_name in case_file_list if prefix in file_name]
  print case_file_list
    
  for case_file in case_file_list:
    with open(case_file, 'r') as f:
      file_data = f.read()
      data = yaml.load(file_data)
    
    plotAll(plt, data)

    param_file = args.config
    # read params
    with open(param_file, 'r') as f:
      params = yaml.load(f)
    # plot shape of ego car
    plotCar(plt, "b-", data['init_state'], params['param'])
    plotCar(plt, "g--", data['target_state'], params['param'])
    
    plt.axis('equal')

    if mode == 'save':
      case_name = case_file.split('/')[-1]
      if not os.path.exists('./result/'):
        os.makedirs('./result/')
      plt.savefig('./result/' + case_name.split('.')[0]+'.png')
    else:
      plt.show()
