import json
import time
import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import os
from tf2d import Point2D as Point2D
plot_astar_node = True

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

    def init(self, id):
        path_base = "../scenario/"
        file_name_base = "new_"
        file_name = path_base + file_name_base + id + ".json"
        with open(file_name) as f:
            data = json.load(f)
            self.odo_json = data["OpenspaceDeciderOutput"]
            self.init_point = Init(data)
            self.target = Target(data)
            self.obstacle_box = ObstacleBox(data)
            self.points = Points(data)
            self.lines = Lines(data)
            self.obstacle_lines = ObstacleLines(data)
            # self.obstacle = Obstacle(data["obstacle"])

    def tf_normal_point(self, point_coord: list):  # 返回列表
      return point_coord

    def plot_obstacle_box(self):
      cord = []
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
      outline_x, outline_y = [], []
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
        car = plt.Polygon(car_line, edgecolor='green', facecolor='none', alpha=0.5) 
      else:
        car = plt.Polygon(car_line, edgecolor='blue', facecolor='none', alpha=0.5)
      self.ax.add_patch(car)

      return front_left_init, front_right_init, rear_left_init, rear_right_init

    def plot_astar(self):
      # 打开文件并读取数据
      if not os.access("../build/search_node.txt", os.R_OK):
        return
      with open("../build/search_node.txt", "r") as f:
        data = f.readlines()
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
            if j % 10000 == 0:
                self.ax.scatter(x, y)
                plt.pause(0.001)
                x.clear()
                y.clear()

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

if __name__ == '__main__':
    data = ParseBagJson()
    filename_num = sys.argv[1] 
    data.init(filename_num)
    # # 先绘制除了init和target外所有物体，自带坐标转化
    plt.ion()
    data.plot_obstacle_box()
    data.plot_points()
    data.plot_lines()
    data.plot_obstacle_lines()
    data.plot_car(data.target.x, data.target.y, data.target.theta)
    data.plot_car(data.init_point.x, data.init_point.y, data.init_point.theta, True)
    time.sleep(1)
    if(plot_astar_node):
        data.plot_astar()
    plt.ioff()
    plt.show()