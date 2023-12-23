#!/usr/bin/python
# -*- coding=utf8 -*-

import os
import sys
import yaml
import matplotlib.pyplot as plt
from math import cos, sin, hypot
import numpy as np
from matplotlib.animation import FuncAnimation
from plot_input import plotAll, plotCar

DISPLAY_SEARCH_POINTS = False
DYNAMIC = False
DUMP_RESULT = False
FORCE_SWAP_INIT_AND_GOAL = False


# visualize problem definition
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

def plotCarAt(axes, axes_param_dict, car_x, car_y, car_theta, car_param):
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
  
  out = axes.plot(car_xx, car_yy, axes_param_dict)
  return out

def plotTrajShape(axes, axes_param_dict, trajectory):
  x = []
  y = []
  for traj_point in trajectory:
    x.append(traj_point.path_point_.x_)
    y.append(traj_point.path_point_.y_)
  
  out = axes.plot(x, y, axes_param_dict)
  
  return out

def plotSearchPoints(axes, axes_param_dict, search_points):
  x = []
  y = []
  cz = np.linspace(0.,1.,len(search_points))
  for p in search_points:
    x.append(p.x)
    y.append(p.y)
  
  out = axes.scatter(x, y, c=cz)
  print(cz)
  plt.colorbar(out)
  return out

def dynamicPlotSearchPoints(fig, axes, search_points):
  if(len(search_points)==0):
    return
  cz = np.linspace(0.,1.,len(search_points))
  xdata, ydata = [], []

  def update(frame):
    xdata.append(frame.x)
    ydata.append(frame.y)
    sc = axes.scatter(xdata, ydata)
    
    return sc,


  ani = FuncAnimation(fig,update,frames = list(search_points),interval=1,blit=True, repeat=False)
  #ani.save('play.mp4')
  # plt.show()
  #ani.save('lines.mp4')
  return ani


def plotTrajDictListShape(axes, axes_param_dict, trajectory):
  x = []
  y = []
  for traj_point in trajectory:
    x.append(traj_point['x'])
    y.append(traj_point['y'])
  
  out = axes.plot(x, y, axes_param_dict)
  return out

def plotTrajControls(axes, trajectory):
  t = [0]
  v = [0]
  dv = [0]
  a = [0]
  steer = [0]
  wheel_base_offset = [0]
  for traj_point in trajectory:
    t.append(traj_point.relative_time_)
    v.append(traj_point.v_)
    a.append(traj_point.a_)
    dv.append(v[-1] - v[-2])
    steer.append(traj_point.steer_)
    wheel_base_offset.append(traj_point.wheel_base_offset_)

  axes.plot(t,a,'g-', label='a')
  axes.plot(t,dv,'g--', label='dv')
  axes.plot(t,v,'r-', label='v')
  axes.plot(t,steer,'b-', label='steer')
  axes.plot(t,wheel_base_offset,'y+-', label='wheel_base_offset')

  ds = [0]
  try:
    ds.append(0)
    for i in range(1, len(traj_dict_list)):
      ds_rough = hypot(traj_dict_list[i]['x'] - traj_dict_list[i - 1]['x'], \
        traj_dict_list[i]['path_point_'].y_ - traj_dict_list[i - 1]['path_point_'].y_)
      ds.append(ds_rough)
    axes.plot(t,ds, 'r--', label='ds')
  except Exception:
    pass

def checkDuplication(data):
  obstacle_line_list = []
  cnt = 0
  for i in range(0, int(len(data['obstacle_lines']['x']) / 2)):
    obstacle_line_list.append(
      ((data['obstacle_lines']['x'][i * 2], data['obstacle_lines']['y'][i * 2]),
        (data['obstacle_lines']['x'][i * 2 + 1], data['obstacle_lines']['y'][i * 2 + 1])
      )
    )
    if obstacle_line_list.count(obstacle_line_list[-1]) > 1:
      print "[Warning]: duplicated lines occured at line[{}].".format(i)
      cnt += 1
  print "duplication count:", cnt

def loadTrajectory(data):
  try:
    traj = data['result_hybridastar']
    traj_size = len(traj)
    trajectory = ffi.new("TrajectoryPoint[]", traj_size)
    for i in range(traj_size):
      trajectory[i].path_point_.x_ = traj[i]['x']
      trajectory[i].path_point_.y_ = traj[i]['y']
      trajectory[i].v_ = traj[i]['v']
  except Exception:
    trajectory = []
  return trajectory

def loadTrajectoryDictList(traj):
  try:
    traj_size = len(traj)
    trajectory = []
    for i in range(traj_size):
      tp = {}
      tp['x'] = traj[i]['x']
      tp['y'] = traj[i]['y']
      tp['v_'] = traj[i]['v']
      trajectory.append(tp)
  except Exception as ex:
    print ex
    trajectory = []
  print "loadTrajectoryDictList size is",len(trajectory)
  return trajectory

def planOnce(ax2, ax1, case_file = 'case.yaml', default_config_param_file = 'apa.yaml', config_param_file = 'apa.yaml', car_param_file = 'apa.yaml', is_rerun=True):
  ## init files
  # read log
  with open(case_file, 'r') as f:
    file_data = f.read()
    data = yaml.load(file_data)
  plotAll(ax1, data)

  # read params
  with open(car_param_file, 'r') as f:
    car_params = yaml.load(f)
  with open(config_param_file, 'r') as f:
    config_params = yaml.load(f)
  
  # plot shape of ego car
  plotCar(ax1, "b-", data['init_state'], car_params['param'], label='init_state')
  plotCar(ax1, "g-", data['target_state'], car_params['param'], label='target_state')

  # load obstacles
  try:
    data['obstacle_boxs']
  except Exception:
    data['obstacle_boxs'] = []

  try:
    data['obstacle_lines']
  except Exception:
    data['obstacle_lines'] = []

  # plot trajectory
  traj_d_l = []
  try:
    traj_d_l = loadTrajectoryDictList(data['result']["points"])
  except Exception as ex:
    print "exception:", ex
  plotTrajDictListShape(ax1, 'r--', traj_d_l)
  
  # visualize points
  if 'points' in data and data['points'] is not None:
    x = []
    y = []
    for p in data['points']:
      x.append(p['x'])
      y.append(p['y'])
    ax1.plot(x, y, '+r')

  result_dict = {}
  
  if is_rerun:
    sys.path.append("..")
    from lib_hobca_planner import lib_hobca_planner, ffi

    ## load problem definition
    # load init and target_state
    start_state = ffi.new("TrajectoryPoint *")
    target_state = ffi.new("TrajectoryPoint *")

    start_state[0].path_point_.x_ = float(data['init_state']['path_point'][0])
    start_state[0].path_point_.y_ = float(data['init_state']['path_point'][1])
    start_state[0].path_point_.theta_ = float(data['init_state']['path_point'][2])
    start_state[0].path_point_.s_ = 0.0
    start_state[0].path_point_.kappa_ = 0.0
    start_state[0].v_ = float(data['init_state']['v'])
    start_state[0].a_ = 0
    start_state[0].da_ = 0
    start_state[0].steer_ = 0

    target_state[0].path_point_.x_ = float(data['target_state']['path_point'][0])
    target_state[0].path_point_.y_ = float(data['target_state']['path_point'][1])
    target_state[0].path_point_.theta_ = float(data['target_state']['path_point'][2])
    target_state[0].path_point_.s_ = 0.0
    target_state[0].path_point_.kappa_ = 0.0
    target_state[0].v_ = float(data['target_state']['v'])
    target_state[0].a_ = 0
    target_state[0].da_ = 0
    target_state[0].steer_ = 0

    # for map boundary
    map_boundary = ffi.new("Box2d *")
    map_boundary[0].center.x = data['map_boundary']['center_x']
    map_boundary[0].center.y = data['map_boundary']['center_y']
    map_boundary[0].heading = data['map_boundary']['heading']
    map_boundary[0].length = data['map_boundary']['length']
    map_boundary[0].width = data['map_boundary']['width']

    ## rerun optimization
    arg_config_file = ffi.new("char[]", config_param_file)
    arg_stra_odca_file = ffi.new("char[]", default_config_param_file)
    arg_car_param_file = ffi.new("char[]", car_param_file)
    
    if not lib_hobca_planner.construct_planner(arg_config_file, arg_stra_odca_file, arg_car_param_file, target_state[0], map_boundary[0]):
      raise Exception("Failed to construct planner.")

    trajectory = []
    search_points = []
    num_segments = [0]

    time_used=lib_hobca_planner.plan(arg_config_file)
    result_dict['time']=time_used
    trajectory = ffi.new("TrajectoryPoint[]", int(lib_hobca_planner.get_result_size()))
    search_points = ffi.new("Pose2d[]",int(lib_hobca_planner.get_search_size()))
    num_segments = ffi.new("int*")
    iteration_times = ffi.new("int*")
    debug_string = ffi.new("char[]", 128)
    #edges = ffi.new("Line2d[]",int(lib_hobca_planner.get_edge_size()))
    lib_hobca_planner.get_result(trajectory, num_segments, iteration_times, debug_string)
    lib_hobca_planner.get_search_progress(search_points)

    if not lib_hobca_planner.destruct_planner():
      raise Exception("Failed to destruct planner.")

    plotTrajShape(ax1, 'g', trajectory)

    for tp in trajectory:
      plotCarAt(ax1, 'y--', tp.path_point_.x_, tp.path_point_.y_, tp.path_point_.theta_, car_params['param'])

    ani = None
    if(DISPLAY_SEARCH_POINTS):
      if(DYNAMIC):
        ani = dynamicPlotSearchPoints(fig, ax1,search_points)
      else:
        plotSearchPoints(ax1,'k',search_points)
    
    
      

    # plot velocity and acceleration
    plotTrajControls(ax2, trajectory)
    
    tp_array = []
    for i in range(0, len(trajectory)):
      tp = {}
      tp["x"] = trajectory[i].path_point_.x_
      tp["y"] = trajectory[i].path_point_.y_
      tp["theta"] = trajectory[i].path_point_.theta_
      tp["v"] = trajectory[i].v_
      tp_array.append(tp)
    result_dict["points"] = tp_array
    result_dict["num_segments"] = num_segments[0]
    result_dict["iteration_times"] = iteration_times[0]
    result_dict["debug_string"] = ffi.string(debug_string)

    if DUMP_RESULT:
      data["result"] = result_dict
      with open(case_file, 'w') as f:
        yaml.dump(data, f)

    # release the memory which is allocated dynamically
    if trajectory:
      ffi.release(trajectory)
    if search_points:
      ffi.release(search_points)
    ffi.release(arg_config_file)
    ffi.release(start_state)
    ffi.release(target_state)
    ffi.release(arg_car_param_file)

  return result_dict, car_params
  
def rename_log(case_file):
  if not os.path.exists('/home/ros/Downloads/logs/'):
      os.makedirs('/home/ros/Downloads/logs/')
  from shutil import copyfile
  case_name = case_file.split('/')[-1]
  copyfile('/home/ros/Downloads/log_debug.txt', '/home/ros/Downloads/logs/'+ case_name.split('.')[0]+'_log.txt')

def append_case_debug(case_file, debug):
  f = open("/home/ros/Downloads/log.txt", "a")
  f.write(case_file+": "+debug+'\n')
  f.close()

def write_time(used_times, config_param_file):
  f = open("/home/ros/Downloads/time_log.txt", "a+")
  avg=sum(used_times)/len(used_times)
  line='\t'.join(map(str,used_times))

  config_params={}
  with open(config_param_file, 'r') as config_file:
    config_params=yaml.load(config_file)

  if config_params['STRATEGY_PARAM']['use_t_line']:
    f.write("USE_GRID:")
  else:
    f.write("NO__GRID:")
        
  f.write(" avg:{}\t".format(avg)+line+"\n")
  f.close()

if __name__ == "__main__":
  import argparse
  default_case_file = "test.yaml"
  car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/config/scenario_configs_json/parking/vehicle_param.yaml"
  default_config_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/config/scenario_configs_json/parking/apa.yaml"

  parser = argparse.ArgumentParser()
  parser.add_argument("folder", help="folder/file, if is folder, all files will be checked recursively.",
                      type = str, default = ".")
  parser.add_argument("-s", "--start", help="the starting number of cases in the folder",
                      type = int, default = None)
  parser.add_argument("-e", "--end", help="the ending number of cases in the folder; if not specified, only one file will be checked.",
                      type = int, default = None)
  parser.add_argument("-m", "--mode",help = "show; save; silent",
                      type = str, default = 'show')
  parser.add_argument("-r", "--replay",help = "whether replay",
                      type = str, default = 'n')
  parser.add_argument("-c", "--config", help="whether load param from config file, read from case by default;\
    d: default config file",
                      type = str, default="")
  parser.add_argument("-k", "--keyword", help="the keyword of file in the folder",
                        type = str, default = "")
  parser.add_argument("-l", "--log_mode", help="log_mode for debug",
                        type = str, default = "no")
  args = parser.parse_args()


  folder = args.folder
  kw = args.keyword

  case_file_list = []
  if args.folder == '.':
    case_file_list.append(default_case_file)
  else:
    if os.path.isfile(folder):
      case_file_list.append(folder)
    elif os.path.isdir(folder):
      for root, dirs, files in os.walk(folder, topdown=False):
        for f in files:
          case_file_list.append(os.path.join(root, f))
    else:
      raise Exception("no such file or directory")
  case_file_list = [f for f in case_file_list if kw in f]

  start = args.start
  end = args.end
  mode = args.mode
  is_replay = args.replay
  log_mode = args.log_mode

  if mode not in ['save','show', 'silent']:
    raise Exception("invalid mode")
  if is_replay not in ['y', 'n']:
    raise Exception("invalid --replay argument")
  if log_mode not in ['no','batch','verbose']:
    raise Exception("invalid debug mode")

  case_file_to_run = []
  if start is None:
    case_file_to_run = case_file_list
  elif end is None:
    case_file_to_run = case_file_list[start-1:start]
  else:
    case_file_to_run = case_file_list[start-1:end]

  for u in case_file_to_run:
    print(u)

  if mode == 'save' and not os.path.exists('/home/ros/Downloads/images/'):
      os.makedirs('/home/ros/Downloads/images/')
  
  if log_mode == 'batch':
    if os.path.exists("/home/ros/Downloads/log.txt"):
      os.remove("/home/ros/Downloads/log.txt")
    f = open("/home/ros/Downloads/log.txt", "w")
    f.close()

  used_times=[]
  
  for case_file in case_file_to_run:
    print "----8<---- processing file :" , case_file, "----8<----"
    config_param_file = args.config
    if args.config == "":
      config_param_file = case_file
    elif args.config == "d":
      config_param_file = default_config_param_file
    ## init figure
    fig, (ax2, ax1) = plt.subplots(1,2,figsize=(18,9))
    result_dict, car_param = planOnce(ax2, ax1, case_file, default_config_param_file, config_param_file, car_param_file, is_replay is 'y')
    ax1.axis('equal')
    with open(case_file, 'r') as f:
      file_data = f.read()
      data = yaml.load(file_data)
    half_x_bound = data['map_boundary']['length']
    half_y_bound = data['map_boundary']['width']
    ax1.set_xbound(data['map_boundary']['center_x'] - half_x_bound, data['map_boundary']['center_x'] + half_x_bound)
    ax1.set_ybound(data['map_boundary']['center_y'] - half_y_bound, data['map_boundary']['center_y'] + half_y_bound)
    ax1.grid()
    ax1.legend()
    ax2.legend()
    
    if mode == 'save':
      case_name = case_file.split('/')[-1]
      plt.savefig('/home/ros/Downloads/images/'+ ".".join(case_name.split('.')[:-1])+'.png')
      plt.close(fig)
    elif mode == 'show':
      plt.legend()
      plt.show()
    else:
      plt.clf()
      plt.close()
    
    if log_mode == "batch":
      append_case_debug(case_file, result_dict['debug_string'])
    
    if log_mode == 'verbose':
      rename_log(case_file)
    print "----8<---- file ", case_file, "is processed!----8<----\n\n"
    
    used_times.append(result_dict['time'])
  
  print "avg time: ", np.mean(used_times)
  print used_times
  write_time(used_times, config_param_file)
