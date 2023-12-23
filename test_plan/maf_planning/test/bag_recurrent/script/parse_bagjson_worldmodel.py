import json
import time

import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import tf2d
import os
from tf2d import Point2D as Point2D
from easydict import EasyDict
import copy

sys.path.insert(0, os.path.join(os.getcwd(), "../lib"))
import py_parking_plotter as apa_plan

CAR_PARAM_EP = "../resources/vehicle_param_l.yaml"
APA_PARALLEL = "../resources/epa.yaml"  # apa file for parallel scenario


class Init(Point2D):
  def __init__(self, data=None):
    if data is not None:
      super(Init, self).__init__(
        data['OpenspaceDeciderOutput']["init_state"]["path_point"]["x"],
        data['OpenspaceDeciderOutput']["init_state"]["path_point"]["y"],
        data['OpenspaceDeciderOutput']["init_state"]["path_point"]["theta"])


class Target(Point2D):
  def __init__(self, data=None):
    if data is not None:
      super(Target, self).__init__(
        data['OpenspaceDeciderOutput']["target_state"]["path_point"]["x"],
        data['OpenspaceDeciderOutput']["target_state"]["path_point"]["y"],
        data['OpenspaceDeciderOutput']["target_state"]["path_point"]["theta"])


class ObstacleBox(object):
  def __init__(self, data=None):
    if data is not None:
      self.length = len(data['OpenspaceDeciderOutput']["obstacle_boxs"])
      self.center_list, self.corner_list, self.corner_point_list_x, self.corner_point_list_y, self.heading_list \
          = [[] for i in range(5)]

      for i in range(self.length):
        self.center_list.append(
          data['OpenspaceDeciderOutput']["obstacle_boxs"][i]["center_"])
        self.corner_list.append(
          data['OpenspaceDeciderOutput']["obstacle_boxs"][i]["corners_"])
        for j in range(4):
          self.heading_list.append(
            data['OpenspaceDeciderOutput']["obstacle_boxs"][i]["heading_"])

      # print(self.heading_list)

      for corner in self.corner_list:
        for point in corner:
          self.corner_point_list_x.append(point["x_"])
          self.corner_point_list_y.append(point["y_"])

      step = 4
      self.corner_point_list_x = \
          [self.corner_point_list_x[i:i + step] for i in range(0, len(self.corner_point_list_x), step)]
      self.corner_point_list_y = \
          [self.corner_point_list_y[i:i + step] for i in range(0, len(self.corner_point_list_y), step)]
      self.heading_list = \
          [self.heading_list[i:i + step] for i in range(0, len(self.heading_list), step)]


class Points(object):
  def __init__(self, data=None):
    if data is not None:
      self.point_length = len(data['OpenspaceDeciderOutput']["points"])
      self.point_list_x, self.point_list_y = [[] for n in range(2)]

      for i in range(self.point_length):
        self.point_list_x.append(
          data['OpenspaceDeciderOutput']["points"][i]["x_"])
        self.point_list_y.append(
          data['OpenspaceDeciderOutput']["points"][i]["y_"])


class Lines(object):
  def __init__(self, data=None):
    if data is not None:
      self.lines_length = len(data['OpenspaceDeciderOutput']["lines"])
      self.line_list_end_x, self.line_list_end_y, self.line_list_end_yaw, self.line_list_start_x, \
      self.line_list_start_y, self.line_list_start_yaw = [[] for n in range(6)]

      for i in range(self.lines_length):
        self.line_list_end_x.append(
          data['OpenspaceDeciderOutput']["lines"][i]["end_"]["x_"])
        self.line_list_end_y.append(
          data['OpenspaceDeciderOutput']["lines"][i]["end_"]["y_"])
        self.line_list_end_yaw.append(
          data['OpenspaceDeciderOutput']["lines"][i]["heading_"])
        self.line_list_start_x.append(
          data['OpenspaceDeciderOutput']["lines"][i]["start_"]["x_"])
        self.line_list_start_y.append(
          data['OpenspaceDeciderOutput']["lines"][i]["start_"]["y_"])
        self.line_list_start_yaw.append(
          data['OpenspaceDeciderOutput']["lines"][i]["heading_"])


class ObstacleLines(object):
  def __init__(self, data=None):
    if data is not None:
      self.lines_length = len(data['OpenspaceDeciderOutput']["obstacle_lines"])
      self.line_list_end_x, self.line_list_end_y, self.line_list_end_yaw, self.line_list_start_x, \
      self.line_list_start_y, self.line_list_start_yaw = [[] for n in range(6)]

      for i in range(self.lines_length):
        self.line_list_end_x.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["end_"]["x_"])
        self.line_list_end_y.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["end_"]["y_"])
        self.line_list_end_yaw.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["heading_"])
        self.line_list_start_x.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["start_"]["x_"])
        self.line_list_start_y.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["start_"]["y_"])
        self.line_list_start_yaw.append(
          data['OpenspaceDeciderOutput']["obstacle_lines"][i]["heading_"])


# void CalNextOutlinePosition()
# {
#   float back_center_x = self_position_next.x - vehicle_params_.rear_axle_to_back * std::cos(self_position_next.yaw);
#   float back_center_y = self_position_next.y - vehicle_params_.rear_axle_to_back * std::sin(self_position_next.yaw);
#   left_back_position.x = back_center_x - vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
#   left_back_position.y = back_center_y + vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
#   right_back_position.x = back_center_x + vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
#   right_back_position.y = back_center_y - vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);

#   float head_center_x = self_position_next.x + vehicle_params_.rear_axle_to_head * std::cos(self_position_next.yaw);
#   float head_center_y = self_position_next.y + vehicle_params_.rear_axle_to_head * std::sin(self_position_next.yaw);
#   left_head_position.x = head_center_x - vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
#   left_head_position.y = head_center_y + vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
#   right_head_position.x = head_center_x + vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
#   right_head_position.y = head_center_y - vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
# }


class ParseBagJson:
    def __init__(self):
        self.obstacle_box = ObstacleBox()
        self.init_point = Init()
        self.target = Target()
        self.points = Points()
        self.lines = Lines()
        self.obstacle_lines = ObstacleLines()

        self.delta_x, self.delta_y, self.delta_yaw = 0, 0, 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)

        self.ax.autoscale(enable=True, axis="both", tight=True)
        self.ax.set_aspect(1)
        self.file_name = "names.json"
        print (self.file_name)

    def init(self, path):
        with open(path) as f:
            data = json.load(f)
            self.odo_json = data["OpenspaceDeciderOutput"]
            self.init_point = Init(data)
            self.target = Target(data)
            self.obstacle_box = ObstacleBox(data)
            self.points = Points(data)
            self.lines = Lines(data)
            self.obstacle_lines = ObstacleLines(data)
            # self.obstacle = Obstacle(data["obstacle"])

    def tf_point2d(self):  # , local_frame: Point2D, pose: Point2D):
        self.target = tf2d.tf2d(self.init_point, self.target)  # 先转自车坐标
        self.init_point = tf2d.tf2d(self.init_point, self.init_point)
        #
        # print(self.init_point.x)
        # print(self.target.x)

    def tf_normal_point(self, point_coord: list):  # 返回列表
        # tmp_point = Point2D(point_coord[0], point_coord[1], point_coord[2])
        #
        # point_coord_point2d = tf2d.tf2d(tmp_point, self.init_point)
        #
        # point_coord[0] = point_coord_point2d.x
        # point_coord[1] = point_coord_point2d.y
        # point_coord[2] = point_coord_point2d.theta
        # print("init_point", self.init_point.y)
        return point_coord

    def plot_obstacle_box(self):
        cord = []
        # self.fig = plt.figure()
        # self.ax = self.fig.add_subplot(1, 1, 1)

        for i in range(len(self.obstacle_box.corner_point_list_x)):
            for j in range(len(self.obstacle_box.corner_point_list_x[i])):
                cord.append([self.obstacle_box.corner_point_list_x[i][j], self.obstacle_box.corner_point_list_y[i][j],
                             self.obstacle_box.heading_list[i][j]])

        step = 4
        cord = [cord[i:i + step] for i in range(0, len(cord), step)]

        for i in range(len(cord)):
            for j in range(len(cord[i])):
                cord[i][j] = self.tf_normal_point(cord[i][j])

        for i in range(len(cord)):
            for j in range(len(cord[i])):
                cord[i][j] = cord[i][j][:2]

        for i in cord:
            pgon = plt.Polygon(i, color='r', alpha=0.9)
            self.ax.add_patch(pgon)

        # plt.show()

    def plot_car(self, x, y, yaw, is_slot = False):
        # im_ddtool.InitVehiclePosition(x, y, yaw)
        # front_left_init = np.arange(2, dtype="float32")
        # front_right_init = np.arange(2, dtype="float32")
        # rear_left_init = np.arange(2, dtype="float32")
        # rear_right_init = np.arange(2, dtype="float32")

        # im_ddtool.GetOutlinePosition(front_left_init, front_right_init, rear_left_init, rear_right_init)

        outline_x, outline_y = [], []

        front_to_rear = 4.015
        back_to_rear = 1.083
        half_width = 1.059


        front_left_init = []
        front_right_init = []
        rear_left_init = []
        rear_right_init = []

        move_list = [[1,1],[1, -1],[-1, -1],[-1, 1]]

        for i in range (4):
            tmp_x = 0
            tmp_y = 0
            if i == 0:
                tmp_x = x + move_list[i][0] * front_to_rear * math.cos(yaw) + move_list[i][1] * half_width * math.sin(yaw)
                tmp_y = y + move_list[i][0] * front_to_rear * math.sin(yaw) - move_list[i][1] * half_width * math.cos(yaw)
                front_right_init.append(tmp_x)
                front_right_init.append(tmp_y)
            elif i == 1:
                tmp_x = x + move_list[i][0] * front_to_rear * math.cos(yaw) + move_list[i][1] * half_width * math.sin(yaw)
                tmp_y = y + move_list[i][0] * front_to_rear * math.sin(yaw) - move_list[i][1] * half_width * math.cos(yaw) 
                front_left_init.append(tmp_x)
                front_left_init.append(tmp_y)
            elif i == 2:
                tmp_x = x + move_list[i][0] * back_to_rear * math.cos(yaw) + move_list[i][1] * half_width * math.sin(yaw)
                tmp_y = y + move_list[i][0] * back_to_rear * math.sin(yaw) - move_list[i][1] * half_width * math.cos(yaw)    
                rear_left_init.append(tmp_x)
                rear_left_init.append(tmp_y)                        
            else:
                tmp_x = x + move_list[i][0] * back_to_rear * math.cos(yaw) + move_list[i][1] * half_width * math.sin(yaw)
                tmp_y = y + move_list[i][0] * back_to_rear * math.sin(yaw) - move_list[i][1] * half_width * math.cos(yaw)
                rear_right_init.append(tmp_x)
                rear_right_init.append(tmp_y)


        # im_ddtool.InitVehiclePosition(x, y, yaw)
        # # im_ddtool.InitVehiclePosition(-5.38, 6.84, 0.09)
        # front_left_init = np.arange(2, dtype="float32")
        # front_right_init = np.arange(2, dtype="float32")
        # rear_left_init = np.arange(2, dtype="float32")
        # rear_right_init = np.arange(2, dtype="float32")

        # outline = im_ddtool.GetOutlinePosition(front_left_init, front_right_init, rear_left_init, rear_right_init)

        outline_x.append(front_left_init[0])
        outline_x.append(front_right_init[0])
        outline_x.append(rear_right_init[0])
        outline_x.append(rear_left_init[0])
        # outline_x.append(front_left_init[0])

        outline_y.append(front_left_init[1])
        outline_y.append(front_right_init[1])
        outline_y.append(rear_right_init[1])
        outline_y.append(rear_left_init[1])
        # outline_y.append(front_left_init[1])

        # a = plt.plot(outline_x, outline_y)
        car_line = []
        for i in range(len(outline_x)):
            car_line.append([outline_x[i], outline_y[i]])
        if is_slot:
            car = plt.Polygon(car_line, color="g", alpha=0.5, fill=False)
        else:
            car = plt.Polygon(car_line, color="b", alpha=0.5, fill=False)
        self.ax.add_patch(car)

        return front_left_init, front_right_init, rear_left_init, rear_right_init

    def plot_trajectory(self, x, y, yaw):
        success = False
        for i in range (1):
            apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)

            odo_dict = EasyDict(self.odo_json)
            cur_odo = copy.deepcopy(odo_dict)

            odo_str = json.dumps(cur_odo)
            res_str = apa_plan.planInterfaceSerialize(odo_str)
            res_dict = EasyDict(json.loads(res_str))

            x_list = res_dict["x"]
            y_list = res_dict["y"]

            num_p = len(x_list)

            if num_p > 0:
                success = True
                # for i in range (num_p):
                    # point = plt.Circle((x_list[i], y_list[i]),  0.02, color = 'r')
                    # self.ax.add_patch(point)
                    
        return success

    def plot_astar(self):
        # 打开文件并读取数据
        if ~os.access("../build/fpq.txt", os.R_OK):
            return
        with open("../build/fpq.txt", "r") as f:
            data = f.readlines()
        

        # 初始化绘图
        # fig = plt.figure()
        # self.ax = fig.add_subplot(111)
        # self.ax.set_xlim(-15, 15)
        # self.ax.set_ylim(-5, 15)
        j = 0
        x = []
        y = []
        # 循环绘制点
        if (len(data) < 1000):
            for i in data:
                nums = i.strip().split()
                x.append(float(nums[0]))
                y.append(float(nums[1]))
                self.ax.scatter(x, y)
            return
        for i in data:
            j = j+1
            # 从文件中读取坐标数据
            nums = i.strip().split()
            x.append(float(nums[0]))
            y.append(float(nums[1]))
            if j % 1000 == 0:
                self.ax.scatter(x, y)
                plt.pause(0.001)
                x.clear()
                y.clear()
            # 绘制点并暂停
    
    

# plt.show()
    
        # print(worker_trajectory_x)
            # trajectory_park = plt.Polygon(parkingin_path, color="r", alpha=0.9)
            # trajectory_worker = plt.Polygon(worker_path, color="b", alpha=0.9)

            # # trajectory_worker = plt.plot(worker_trajectory_x, worker_trajectory_y)
            # # trajectory_park = plt.Poly(parking_in_x, parking_in_y)
            # self.ax.add_patch(trajectory_park)
            # self.ax.add_patch(trajectory_worker)

    # def plot_car(self, x, y, yaw, w, h):
    #     x0 = x + (h / 2 * math.sin(yaw) - w / 2 * math.cos(yaw))
    #     y0 = y + (h / 2 * math.cos(yaw) + w / 2 * math.sin(yaw))
    #
    #     x1 = x + (h / 2 * math.sin(yaw) + w / 2 * math.cos(yaw))
    #     y1 = y + (h / 2 * math.cos(yaw) - w / 2 * math.sin(yaw))
    #
    # print(self.init_point.x)
    # print(self.target.x)

    def tf_normal_point(self, point_coord: list):  # 返回列表
      # tmp_point = Point2D(point_coord[0], point_coord[1], point_coord[2])
      #
      # point_coord_point2d = tf2d.tf2d(tmp_point, self.init_point)
      #
      # point_coord[0] = point_coord_point2d.x
      # point_coord[1] = point_coord_point2d.y
      # point_coord[2] = point_coord_point2d.theta
      # print("init_point", self.init_point.y)
      return point_coord

    def plot_obstacle_box(self):
      cord = []
      # self.fig = plt.figure()
      # self.ax = self.fig.add_subplot(1, 1, 1)

      for i in range(len(self.obstacle_box.corner_point_list_x)):
        for j in range(len(self.obstacle_box.corner_point_list_x[i])):
          cord.append([
            self.obstacle_box.corner_point_list_x[i][j],
            self.obstacle_box.corner_point_list_y[i][j],
            self.obstacle_box.heading_list[i][j]
          ])

      step = 4
      cord = [cord[i:i + step] for i in range(0, len(cord), step)]

      for i in range(len(cord)):
        for j in range(len(cord[i])):
          cord[i][j] = self.tf_normal_point(cord[i][j])

      for i in range(len(cord)):
        for j in range(len(cord[i])):
          cord[i][j] = cord[i][j][:2]

      for i in cord:
        pgon = plt.Polygon(i, color='r', alpha=0.9)
        self.ax.add_patch(pgon)

      # plt.show()

    def plot_car(self, x, y, yaw, init = False):
      # im_ddtool.InitVehiclePosition(x, y, yaw)
      # front_left_init = np.arange(2, dtype="float32")
      # front_right_init = np.arange(2, dtype="float32")
      # rear_left_init = np.arange(2, dtype="float32")
      # rear_right_init = np.arange(2, dtype="float32")

      # im_ddtool.GetOutlinePosition(front_left_init, front_right_init, rear_left_init, rear_right_init)

      outline_x, outline_y = [], []

      # front_to_rear = 4.015
      # back_to_rear = 1.083
      # half_width = 1.059

      front_to_rear = 3.913
      back_to_rear = 0.984
      half_width = 0.9935
      front_corner_width = 0.595
      front_corner_length = 3.645

      front_left_init = []
      front_right_init = []
      rear_left_init = []
      rear_right_init = []

      cc = math.cos(yaw)
      cs = math.sin(yaw)

      outline_x.append(x + front_to_rear * cc - front_corner_width * cs)
      outline_y.append(y + front_to_rear * cs + front_corner_width * cc)

      outline_x.append(x + front_corner_length * cc - half_width * cs)
      outline_y.append(y + front_corner_length * cs + half_width * cc)

      outline_x.append(x - back_to_rear * cc - half_width * cs)
      outline_y.append(y - back_to_rear * cs + half_width * cc)

      outline_x.append(x - back_to_rear * cc + half_width * cs)
      outline_y.append(y - back_to_rear * cs - half_width * cc)

      outline_x.append(x + front_corner_length * cc + half_width * cs)
      outline_y.append(y + front_corner_length * cs - half_width * cc)

      outline_x.append(x + front_to_rear * cc + front_corner_width * cs)
      outline_y.append(y + front_to_rear * cs - front_corner_width * cc)

      outline_x.append(x + front_to_rear * cc - front_corner_width * cs)
      outline_y.append(y + front_to_rear * cs + front_corner_width * cc)

      car_line = []
      for i in range(len(outline_x)):
        car_line.append([outline_x[i], outline_y[i]])
      if init:
        # car = plt.Polygon(car_line, color="b", alpha=0.5)
        car = plt.Polygon(car_line, edgecolor='green', facecolor='none', alpha=0.5) 
      else:
        car = plt.Polygon(car_line, edgecolor='blue', facecolor='none', alpha=0.5)
      self.ax.add_patch(car)

      return front_left_init, front_right_init, rear_left_init, rear_right_init

    def plot_trajectory(self, x, y, yaw):
      success = False
      for i in range(1):
        apa_plan.initSingletonParams(CAR_PARAM_EP, APA_PARALLEL)

        odo_dict = EasyDict(self.odo_json)
        cur_odo = copy.deepcopy(odo_dict)

        odo_str = json.dumps(cur_odo)
        # print(odo_str)
        res_str = apa_plan.planInterfaceSerialize(odo_str)
        res_dict = EasyDict(json.loads(res_str))

        x_list = res_dict["x"]
        y_list = res_dict["y"]

        num_p = len(x_list)
        # print(x_list)

        if num_p > 0:
          success = True
          for i in range(num_p):
            point = plt.Circle((x_list[i], y_list[i]), 0.02, color='r')
            self.ax.add_patch(point)

      return success

    def plot_astar(self):
      # 打开文件并读取数据
      if ~os.access("../build/fpq.txt", os.R_OK):
        return
      with open("../build/fpq.txt", "r") as f:
        data = f.readlines()

      # 初始化绘图
      # fig = plt.figure()
      # self.ax = fig.add_subplot(111)
      # self.ax.set_xlim(-15, 15)
      # self.ax.set_ylim(-5, 15)
      j = 0
      x = []
      y = []
      # 循环绘制点
      if (len(data) < 1000):
        for i in data:
          nums = i.strip().split()
          x.append(float(nums[0]))
          y.append(float(nums[1]))
          self.ax.scatter(x, y)
        return
      for i in data:
        j = j + 1
        # 从文件中读取坐标数据
        nums = i.strip().split()
        x.append(float(nums[0]))
        y.append(float(nums[1]))
        if j % 1000 == 0:
          self.ax.scatter(x, y)
          plt.pause(0.001)
          x.clear()
          y.clear()
        # 绘制点并暂停

  # plt.show()

  # print(worker_trajectory_x)
  # trajectory_park = plt.Polygon(parkingin_path, color="r", alpha=0.9)
  # trajectory_worker = plt.Polygon(worker_path, color="b", alpha=0.9)

  # # trajectory_worker = plt.plot(worker_trajectory_x, worker_trajectory_y)
  # # trajectory_park = plt.Poly(parking_in_x, parking_in_y)
  # self.ax.add_patch(trajectory_park)
  # self.ax.add_patch(trajectory_worker)

  # def plot_car(self, x, y, yaw, w, h):
  #     x0 = x + (h / 2 * math.sin(yaw) - w / 2 * math.cos(yaw))
  #     y0 = y + (h / 2 * math.cos(yaw) + w / 2 * math.sin(yaw))
  #
  #     x1 = x + (h / 2 * math.sin(yaw) + w / 2 * math.cos(yaw))
  #     y1 = y + (h / 2 * math.cos(yaw) - w / 2 * math.sin(yaw))
  #
  #     x2 = x - (h / 2 * math.sin(yaw) - w / 2 * math.cos(yaw))
  #     y2 = y - (h / 2 * math.cos(yaw) + w / 2 * math.sin(yaw))
  #
  #     x3 = x - (h / 2 * math.sin(yaw) + w / 2 * math.cos(yaw))
  #     y3 = y - (h / 2 * math.cos(yaw) - w / 2 * math.sin(yaw))
  #
  #     print(x0, y0, x1, y1, x2, y2, x3, y3)
  #     car_outline = [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
  #     car = plt.Polygon(car_outline, color="b", alpha=0.9)
  #     self.ax.add_patch(car)
  #     return [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]

    def cal_slot_outline(self, x, y, yaw, h, w):
      '''
          假设：
          车位长度5.5m,宽度2.5m,终点坐标在车位中心点
          '''
      x0 = x + (h / 2 * math.sin(-yaw) - w / 2 * math.cos(-yaw))
      y0 = y + (h / 2 * math.cos(-yaw) + w / 2 * math.sin(-yaw))

      x1 = x + (h / 2 * math.sin(-yaw) + w / 2 * math.cos(-yaw))
      y1 = y + (h / 2 * math.cos(-yaw) - w / 2 * math.sin(-yaw))

      x2 = x - (h / 2 * math.sin(-yaw) - w / 2 * math.cos(-yaw))
      y2 = y - (h / 2 * math.cos(-yaw) + w / 2 * math.sin(-yaw))

      x3 = x - (h / 2 * math.sin(-yaw) + w / 2 * math.cos(-yaw))
      y3 = y - (h / 2 * math.cos(-yaw) - w / 2 * math.sin(-yaw))

      # print(x0, y0, x1, y1, x2, y2, x3, y3)
      return [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]

    def plot_slot(self, x, y, yaw, h, w):  # 水平与垂直车为目前通过长宽区分
      slot_outline = self.cal_slot_outline(x, y, yaw, h, w)
      slot = plt.Polygon(slot_outline,
                        alpha=0.9,
                        facecolor='none',
                        edgecolor='g')
      self.ax.add_patch(slot)

    def plot_points(self):  # points缺少heading值，目前采用0，需澄清
      for i in range(len(self.points.point_list_x)):
        p = [
          self.points.point_list_x[i], self.points.point_list_y[i],
          self.target.theta
        ]
        p1 = self.tf_normal_point(p)

        point = plt.Circle((p1[0], p1[1]), 0.05)
        self.ax.add_patch(point)

    def plot_lines(self):
      p, p1, p_s, p_s1 = [], [], [], []
      start_x, start_y, end_x, end_y = [], [], [], []
      # print("len(self.lines.line_list_end_x)", len(self.lines.line_list_end_x))
      for i in range(len(self.lines.line_list_end_x)):
        p = [
          self.lines.line_list_end_x[i], self.lines.line_list_end_y[i],
          self.lines.line_list_end_yaw[i]
        ]
        p1 = self.tf_normal_point(p)
        start_x.append(p1[0])
        start_y.append(p1[1])

        p_s = [
          self.lines.line_list_start_x[i], self.lines.line_list_start_y[i],
          self.lines.line_list_start_yaw[i]
        ]
        p_s1 = self.tf_normal_point(p_s)
        end_x.append(p_s1[0])
        end_y.append(p_s1[1])

      for j in range(len(start_x)):
        # print(p1[j])
        line = plt.Polygon([[start_x[j], start_y[j]], [end_x[j], end_y[j]]],
                          color='red',
                          linewidth=1,
                          alpha=0.9)
        self.ax.add_patch(line)

    def plot_obstacle_lines(self):
      p, p1, p_s, p_s1 = [], [], [], []
      start_x, start_y, end_x, end_y = [], [], [], []

      for i in range(len(self.obstacle_lines.line_list_end_x)):
        p = [
          self.obstacle_lines.line_list_end_x[i],
          self.obstacle_lines.line_list_end_y[i],
          self.obstacle_lines.line_list_end_yaw[i]
        ]
        p1 = self.tf_normal_point(p)
        start_x.append(p1[0])
        start_y.append(p1[1])

        p_s = [
          self.obstacle_lines.line_list_start_x[i],
          self.obstacle_lines.line_list_start_y[i],
          self.obstacle_lines.line_list_start_yaw[i]
        ]
        p_s1 = self.tf_normal_point(p_s)
        end_x.append(p_s1[0])
        end_y.append(p_s1[1])

      for j in range(len(start_x)):
        # print(p1[j])
        line = plt.Polygon([[start_x[j], start_y[j]], [end_x[j], end_y[j]]],
                          color='yellow',
                          linewidth=3,
                          alpha=0.9)
        self.ax.add_patch(line)

    def plot_possible_pose(self, sucess_poses, fail_poses):
      for pose in sucess_poses:
        arrow = plt.arrow(pose[0],
                          pose[1],
                          pose[2],
                          pose[3],
                          head_width=0.01,
                          head_length=0.01,
                          fc="green",
                          ec="green")
        self.ax.add_patch(arrow)

      for pose in fail_poses:
        arrow = plt.arrow(pose[0],
                          pose[1],
                          pose[2],
                          pose[3],
                          head_width=0.01,
                          head_length=0.01,
                          fc="red",
                          ec="red")
        self.ax.add_patch(arrow)

    def dym_plot(self):
      self.plot_obstacle_box()
      self.plot_points()
      self.plot_lines()

      self.tf_point2d()

      self.plot_car(data.target.x, data.target.y, data.target.theta)
      self.plot_slot(data.init_point.x, data.init_point.y, data.init_point.theta,
                    2.5, 5.5)
      plt.show()


  # void CalNextOutlinePosition()
  # {
  #   float back_center_x = self_position_next.x - vehicle_params_.rear_axle_to_back * std::cos(self_position_next.yaw);
  #   float back_center_y = self_position_next.y - vehicle_params_.rear_axle_to_back * std::sin(self_position_next.yaw);
  #   left_back_position.x = back_center_x - vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
  #   left_back_position.y = back_center_y + vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
  #   right_back_position.x = back_center_x + vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
  #   right_back_position.y = back_center_y - vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
  #
  #   float head_center_x = self_position_next.x + vehicle_params_.rear_axle_to_head * std::cos(self_position_next.yaw);
  #   float head_center_y = self_position_next.y + vehicle_params_.rear_axle_to_head * std::sin(self_position_next.yaw);
  #   left_head_position.x = head_center_x - vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
  #   left_head_position.y = head_center_y + vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
  #   right_head_position.x = head_center_x + vehicle_params_.half_width_with_rearview_mirror * std::sin(self_position_next.yaw);
  #   right_head_position.y = head_center_y - vehicle_params_.half_width_with_rearview_mirror * std::cos(self_position_next.yaw);
  # }

if __name__ == '__main__':
  data = ParseBagJson()
  # data.init("/home/thj/01_PROJECT/S11L/MPU/im_ddtool/scenario/apa_real_0331_local_3.json")
  data.init("/home/fjw/control/third_measure/im_ddtool/scenario/names.json")
  # # 先绘制除了init和target外所有物体，自带坐标转化
  plt.ion()
  data.plot_obstacle_box()
  data.plot_points()
  data.plot_lines()
  data.plot_obstacle_lines()

  # 转换init和target并绘制
  # data.tf_point2d()

  data.plot_car(data.target.x, data.target.y, data.target.theta)
  # data.plot_car(-5.376595384171512, 6.837324255485564, 0.0945302929806795)
  data.plot_slot(data.init_point.x, data.init_point.y, data.init_point.theta,
                 2.5, 5.5)
  time.sleep(1)
  # data.plot_trajectory()
  # plt.ioff()
  # plt.show()
  # if os.access("../build/fpq.txt", 1):
  if os.access("../build/fpq.txt", os.F_OK):
    os.remove("../build/fpq.txt")
  data.plot_trajectory(data.target.x, data.target.y, data.target.theta)
  data.plot_astar()
  plt.ioff()
  plt.show()

  # for i in range(71):
  #     data = ParseBagJson()
  #     data.init("./JsonbyBag/"+str(i)+".json")
  #
  #     # 先绘制除了init和target外所有物体，自带坐标转化
  #     data.plot_obstacle_box()
  #     data.plot_points()
  #     data.plot_lines()
  #     data.plot_obstacle_lines()
  #
  #     # 转换init和target并绘制
  #     # data.tf_point2d()
  #
  #     data.plot_car(data.target.x, data.target.y, data.target.theta)
  #     data.plot_slot(data.init_point.x, data.init_point.y, data.init_point.theta, 2.5, 5.5)
  #
  #     plt.show()
  #
  #     time.sleep(0.1)
  # plt.clf()
  #
  # plt.ioff()

  # data = ParseBagJson()
  # data.init("./JsonbyBag/10.json")
  # data.dym_plot()
