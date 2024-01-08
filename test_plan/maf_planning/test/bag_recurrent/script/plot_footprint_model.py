import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math


def read_yaml(file_path):
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    return yaml_data
class footprint_model:
    def __init__(self):
        self.delta_x, self.delta_y, self.delta_yaw = 0, 0, 0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.autoscale(enable=True, axis="both", tight=True)
        self.ax.set_aspect(1)
        self.circles_list = []

    def plot_octagon(self, x, y, yaw):
        file_path = "../../../resource/config/scenario_configs_json/parking/vehicle_param_ls6.yaml"
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            front_to_rear = data["param"]["vehicle_length"] - data["param"]["distance_from_rear_bumper_to_rear_axle"]
            back_to_rear = data["param"]["distance_from_rear_bumper_to_rear_axle"]
            half_width = data["param"]["vehicle_width"] / 2
            front_corner_width = 0.595
            front_corner_length = 3.645
            lat = 0
            lon = 0 
            temp_half_width = half_width + lat
            temp_front_corner_width = front_corner_width + 0.02
            temp_front_to_rear = front_to_rear + lon
            temp_back_to_rear = back_to_rear + lon
            temp_front_corner_length = front_corner_length + lon + 0.05
            real_width = half_width
            real_back_rear = back_to_rear

            back_light_len = 0.32
            back_light_height = 0.39
            cc = math.cos(yaw)
            cs = math.sin(yaw)

            p1_x =  x + temp_front_to_rear * cc - temp_front_corner_width * cs
            p1_y =   y + temp_front_to_rear * cs + temp_front_corner_width * cc
            p2_x = x + temp_front_corner_length * cc - temp_half_width * cs
            p2_y =  y + temp_front_corner_length * cs + temp_half_width * cc
            p3_x = x - (real_back_rear - back_light_len) * cc - temp_half_width * cs
            p3_y = y - (real_back_rear - back_light_len) * cs + temp_half_width * cc
            p4_x = x - temp_back_to_rear * cc - (real_width - back_light_height) * cs
            p4_y = y - temp_back_to_rear * cs + (real_width - back_light_height) * cc
            p5_x = x - temp_back_to_rear * cc + (real_width - back_light_height) * cs
            p5_y = y - temp_back_to_rear * cs - (real_width - back_light_height) * cc
            p6_x = x - (real_back_rear - back_light_len) * cc + temp_half_width * cs
            p6_y = y - (real_back_rear - back_light_len) * cs - temp_half_width * cc
            p7_x = x + temp_front_corner_length * cc + temp_half_width * cs
            p7_y = y + temp_front_corner_length * cs - temp_half_width * cc
            p8_x = x + temp_front_to_rear * cc + temp_front_corner_width * cs
            p8_y = y + temp_front_to_rear * cs - temp_front_corner_width * cc

            car_line = []
            car_line.append([p1_x, p1_y])
            car_line.append([p2_x, p2_y])
            car_line.append([p3_x, p3_y])
            car_line.append([p4_x, p4_y])
            car_line.append([p5_x, p5_y])
            car_line.append([p6_x, p6_y])
            car_line.append([p7_x, p7_y])
            car_line.append([p8_x, p8_y])
            # print("plot_octagon", car_line)
            # car = plt.Polygon(car_line, color="b", alpha=1)
            car = plt.Polygon(car_line, edgecolor='green', facecolor='none', alpha=1) 
            self.ax.add_patch(car)

    def plot_circles(self, circles):
        self.ax.set_aspect('equal', 'box')
        for circle in circles:
            x = circle['x']
            y = circle['y']
            r = circle['r']

            circle_patch = Circle((x, y), r, edgecolor='b', facecolor='none')
            self.ax.add_patch(circle_patch)

    def plot_wheelbase(self, x, y, yaw):
        file_path = "../../../resource/config/scenario_configs_json/parking/vehicle_param_ls6.yaml"
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            front_to_rear = data["param"]["vehicle_length"] - data["param"]["distance_from_rear_bumper_to_rear_axle"]
            back_to_rear = data["param"]["distance_from_rear_bumper_to_rear_axle"]
            half_width = data["param"]["vehicle_width"] / 2
            axis_add_temp = data["param"]["wheel_radius_front_left"] / 4
            wheel_base = data["param"]["wheel_base_distance"]
            front_corner_width = 0.595
            front_corner_length = 3.645
            lat = 0
            lon = 0 
            axis_width = half_width + lat
            back_axis_length = - lon
            front_axis_length = wheel_base + lon
            cc = math.cos(yaw)
            cs = math.sin(yaw)
            p1_x = x - axis_width * cs + back_axis_length * cc
            p1_y = y + axis_width * cc + back_axis_length * cs
            p2_x = x + axis_width * cs + back_axis_length * cc
            p2_y = y - axis_width * cc + back_axis_length * cs
            p3_x = x + axis_width * cs + front_axis_length * cc
            p3_y = y - axis_width * cc + front_axis_length * cs
            p4_x = x - axis_width * cs + front_axis_length * cc
            p4_y = y + axis_width * cc + front_axis_length * cs
            car_line = []
            car_line.append([p1_x, p1_y])
            car_line.append([p2_x, p2_y])
            car_line.append([p3_x, p3_y])
            car_line.append([p4_x, p4_y])
            # print("plot_wheelbase ", car_line)
            # car = plt.Polygon(car_line, color="b", alpha=1)
            car = plt.Polygon(car_line, edgecolor='red', facecolor='none', alpha=1) 
            self.ax.add_patch(car)

    def plot_rawshape(self, x, y, yaw):
        file_path = "../../../resource/config/scenario_configs_json/parking/vehicle_param_ls6.yaml"
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            front_to_rear = data["param"]["vehicle_length"] - data["param"]["distance_from_rear_bumper_to_rear_axle"]
            back_to_rear = data["param"]["distance_from_rear_bumper_to_rear_axle"]
            half_width = data["param"]["vehicle_width"] / 2
            front_corner_width = 0.595
            front_corner_length = 3.645
            lat = 0
            lon = 0 
            temp_half_width = half_width + lat
            temp_front_corner_width = front_corner_width + lat
            temp_front_to_rear = front_to_rear + lon
            temp_back_to_rear = back_to_rear + lon
            temp_front_corner_length = front_corner_length + lon
            cc = math.cos(yaw)
            cs = math.sin(yaw)
            p1_x = x + temp_front_to_rear * cc - temp_front_corner_width * cs
            p1_y = y + temp_front_to_rear * cs + temp_front_corner_width * cc
            p2_x = x + temp_front_corner_length * cc - temp_half_width * cs
            p2_y = y + temp_front_corner_length * cs + temp_half_width * cc
            p3_x = x - temp_back_to_rear * cc - temp_half_width * cs
            p3_y = y - temp_back_to_rear * cs + temp_half_width * cc
            p4_x = x - temp_back_to_rear * cc + temp_half_width * cs
            p4_y = y - temp_back_to_rear * cs - temp_half_width * cc
            p5_x = x + temp_front_corner_length * cc + temp_half_width * cs
            p5_y = y + temp_front_corner_length * cs - temp_half_width * cc
            p6_x = x + temp_front_to_rear * cc + temp_front_corner_width * cs
            p6_y = y + temp_front_to_rear * cs - temp_front_corner_width * cc
            car_line = []
            car_line.append([p1_x, p1_y])
            car_line.append([p2_x, p2_y])
            car_line.append([p3_x, p3_y])
            car_line.append([p4_x, p4_y])
            car_line.append([p5_x, p5_y])
            car_line.append([p6_x, p6_y])
            # print(car_line)
            # car = plt.Polygon(car_line, color="b", alpha=1)
            car = plt.Polygon(car_line, edgecolor='m', facecolor='none', alpha=1) 
            self.ax.add_patch(car)
            
    
    def plot_rotateshape(self, x, y, yaw):
        file_path = "../../../resource/config/scenario_configs_json/parking/vehicle_param_ls6.yaml"
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            front_to_rear = data["param"]["vehicle_length"] - data["param"]["distance_from_rear_bumper_to_rear_axle"]
            back_to_rear = data["param"]["distance_from_rear_bumper_to_rear_axle"]
            half_width = data["param"]["vehicle_width"] / 2
            front_corner_width = 0.595
            front_corner_length = 3.645
            lat = 0
            lon = 0
            temp_half_width = half_width + lat
            temp_front_corner_width = front_corner_width + 0.02
            temp_front_to_rear = front_to_rear + lon
            temp_back_to_rear = back_to_rear + lon
            temp_front_corner_length = front_corner_length + lon + 0.05
            real_width = half_width
            real_back_rear = back_to_rear
            cc = math.cos(yaw)
            cs = math.sin(yaw)
            p1_x = x + temp_front_to_rear * cc - temp_front_corner_width * cs
            p1_y = y + temp_front_to_rear * cs + temp_front_corner_width * cc
            p2_x = x + temp_front_corner_length * cc - temp_half_width * cs
            p2_y = y + temp_front_corner_length * cs + temp_half_width * cc
            p3_x = x - real_back_rear * cc - temp_half_width * cs
            p3_y = y - real_back_rear * cs + temp_half_width * cc
            p4_x = x - temp_back_to_rear * cc - real_width * cs
            p4_y = y - temp_back_to_rear * cs + real_width * cc
            p5_x = x - temp_back_to_rear * cc + real_width * cs
            p5_y = y - temp_back_to_rear * cs - real_width * cc
            p6_x = x - real_back_rear * cc + temp_half_width * cs
            p6_y = y - real_back_rear * cs - temp_half_width * cc
            p7_x = x + temp_front_corner_length * cc + temp_half_width * cs
            p7_y = y + temp_front_corner_length * cs - temp_half_width * cc
            p8_x = x + temp_front_to_rear * cc + temp_front_corner_width * cs
            p8_y = y + temp_front_to_rear * cs - temp_front_corner_width * cc
            car_line = []
            car_line.append([p1_x, p1_y])
            car_line.append([p2_x, p2_y])
            car_line.append([p3_x, p3_y])
            car_line.append([p4_x, p4_y])
            car_line.append([p5_x, p5_y])
            car_line.append([p6_x, p6_y])
            car_line.append([p7_x, p7_y])
            car_line.append([p8_x, p8_y])
            # print("plot_rotateshape", car_line)
            # car = plt.Polygon(car_line, color="b", alpha=1)
            car = plt.Polygon(car_line, edgecolor='k', facecolor='none', alpha=1) 
            self.ax.add_patch(car)

if __name__ == "__main__":
    data = footprint_model()
    yaml_file_path = "../../../resource/config/scenario_configs_json/parking/car_configs/multi_circle_footprint_model_lccar.yaml"  # 替换为实际的YAML文件路径
    circles_data = read_yaml(yaml_file_path)
    for i in range(21):
        key = f'circle{i}'
        if key in circles_data:
            data.circles_list.append(circles_data[key])
        else:
            print(f"Error: '{key}' key not found in the YAML file.")
    data.plot_circles(data.circles_list)
    data.plot_octagon(0,0,0)
    data.plot_wheelbase(0,0,0)
    data.plot_rawshape(0,0,0)
    data.plot_rotateshape(0,0,0)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.xlim(-2, 5)
    plt.ylim(-2, 2)
    plt.grid(True)
    plt.show()
    plt.savefig("ccc.png")