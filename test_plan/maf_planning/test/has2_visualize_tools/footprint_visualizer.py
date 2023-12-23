import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import pandas as pd
import yaml
import time
import math



def extractPoint(row : int, table):
    point = table.values[row, 1:3]
    #convert 1d array to 2d
    return point


def extractPoints(rows : list, table):
    result = np.empty([0,2])
    for i in range(0,len(rows)):
        point = extractPoint(rows[i], table)
        result = np.append(result, np.reshape(point,[1,2]), axis=0)
    
    return result

def transformWorldToVehhicle(points : list, R_v2w, t_v2w):
    result = np.empty([0,2])
    for i in range(0,len(points)):

        p_vehicle = R_v2w.T @ (points[i]-t_v2w)
        #print("p_vehicle = ", p_vehicle, p_vehicle.shape)
        result = np.append(result, np.reshape(p_vehicle, [1,2]),axis=0)
    return result


measure_data = pd.read_excel('/home/dozen/Workspace/planning/vehicle_param_measured/ETcar_9871.xlsx')
circle_data_filename = "/home/dozen/Workspace/planning/maf_planning_forked/resource/config/scenario_configs_json/parking/car_configs/multi_circle_footprint_model_wls_c03.yaml"



#inverse Y coordinate because measure data use left-hand coordinate system
measure_data.values[1:36,2] = -measure_data.values[1:36,2]


rows_front_edge = [1,26,27,28,29,30]
rows_rear_edge = [2,31,32,33,34,35]

rows_left_edge = [10,14,16,12]
rows_right_edge = [11,15,17,13]

rows_left_mirror = [4,5,8]
rows_right_mirror = [6,7,9]

rows_front_left_wheel = [18,19]

row_front_left_wheel = 18
row_front_right_wheel = 19
row_rear_left_wheel = 20
row_rear_right_wheel = 21

rows_wheel_ground = [22, 23, 24, 25]

print(measure_data.values[rows_front_edge,1:4])

fl_wheel = extractPoint(row_front_left_wheel, measure_data)
fr_wheel = extractPoint(row_front_right_wheel, measure_data)
rl_wheel = extractPoint(row_rear_left_wheel, measure_data)
rr_wheel = extractPoint(row_rear_right_wheel, measure_data)

front_shaft_center = 0.5*(fl_wheel + fr_wheel)
rear_shaft_center = 0.5*(rl_wheel + rr_wheel)

print("front_shaft_center", front_shaft_center)
print("rear_shaft_center", rear_shaft_center)

vehicle_origin = rear_shaft_center
vehicle_x_axis = front_shaft_center - rear_shaft_center
vehicle_x_axis = vehicle_x_axis / np.linalg.norm(vehicle_x_axis)
print("vehicle_x_axis", vehicle_x_axis)

vehicle_y_axis = np.array([-vehicle_x_axis[1], vehicle_x_axis[0]])
print("vehicle_y_axis", vehicle_y_axis)


R_vehicle_to_world = np.array([vehicle_x_axis,vehicle_y_axis]).T
print("R_vehicle_to_world", R_vehicle_to_world)


front_edge_measured = extractPoints(rows_front_edge, measure_data)
front_edge_measured = transformWorldToVehhicle(front_edge_measured, R_vehicle_to_world, vehicle_origin)
front_edge_measured = front_edge_measured[front_edge_measured[:,1].argsort()]

rear_edge_measured = extractPoints(rows_rear_edge, measure_data)
rear_edge_measured = transformWorldToVehhicle(rear_edge_measured, R_vehicle_to_world, vehicle_origin)
rear_edge_measured = rear_edge_measured[rear_edge_measured[:,1].argsort()]


left_edge_measured = extractPoints(rows_left_edge, measure_data)
left_edge_measured = transformWorldToVehhicle(left_edge_measured, R_vehicle_to_world, vehicle_origin)


right_edge_measured = extractPoints(rows_right_edge, measure_data)
right_edge_measured = transformWorldToVehhicle(right_edge_measured, R_vehicle_to_world, vehicle_origin)

left_mirror_measured = extractPoint(rows_left_mirror, measure_data)
left_mirror_measured = transformWorldToVehhicle(left_mirror_measured, R_vehicle_to_world, vehicle_origin)
left_mirror_measured = np.append(left_mirror_measured, np.reshape(left_mirror_measured[0,:],(1,2)), axis=0)

right_mirror_measured = extractPoint(rows_right_mirror, measure_data)
right_mirror_measured = transformWorldToVehhicle(right_mirror_measured, R_vehicle_to_world, vehicle_origin)
right_mirror_measured = np.append(right_mirror_measured, np.reshape(right_mirror_measured[0,:],(1,2)), axis=0)

front_shaft_measured = np.array([fl_wheel, fr_wheel])
front_shaft_measured = transformWorldToVehhicle(front_shaft_measured, R_vehicle_to_world, vehicle_origin)
rear_shaft_measured = np.array([rl_wheel , rr_wheel])
rear_shaft_measured = transformWorldToVehhicle(rear_shaft_measured, R_vehicle_to_world, vehicle_origin)

mid_shaft_measued = np.array([front_shaft_center , rear_shaft_center])
mid_shaft_measued = transformWorldToVehhicle(mid_shaft_measued, R_vehicle_to_world, vehicle_origin)


front_edge_symed = front_edge_measured.copy()
front_edge_symed[:,1] = - front_edge_symed[:,1]

rear_edge_symed = rear_edge_measured.copy()
rear_edge_symed[:,1] = - rear_edge_symed[:,1]

wheel_center_measured = np.array([fl_wheel, fr_wheel, rl_wheel, rr_wheel])
wheel_center_measured = transformWorldToVehhicle(wheel_center_measured, R_vehicle_to_world, vehicle_origin)

def symWheelBox(p_wheel_to_ground, box_width,  box_length):

    p0 = p_wheel_to_ground + np.array([0.5*box_length, 0])
    p1 = p_wheel_to_ground + np.array([-0.5*box_length, 0])
    p2 = p_wheel_to_ground + np.array([-0.5*box_length, -math.copysign(0.5*box_width, p_wheel_to_ground[1])])
    p3 = p_wheel_to_ground + np.array([0.5*box_length, -math.copysign(0.5*box_width, p_wheel_to_ground[1])])
    
    return np.array([p0,p1,p2,p3,p0])


wheel_width = 0.3
wheel_radius = 0.35
wheel_sym_height = 0.15
wheel_box_length = 2*math.sqrt(wheel_radius*wheel_radius-(wheel_radius-wheel_sym_height)*(wheel_radius-wheel_sym_height))
front_left_wheel_box_symed = symWheelBox(wheel_center_measured[0,:], wheel_width, wheel_box_length)
front_right_wheel_box_symed = symWheelBox(wheel_center_measured[1,:], wheel_width, wheel_box_length)
rear_left_wheel_box_symed = symWheelBox(wheel_center_measured[2,:], wheel_width, wheel_box_length)
rear_right_wheel_box_symed = symWheelBox(wheel_center_measured[3,:], wheel_width, wheel_box_length)



figs = []
model_num = 3
for i in range(model_num):
    fig = plt.figure(figsize = (9,5))
    fig.gca().set_xlim(-1.5,4.5)
    fig.gca().set_ylim(-1.5,1.5)
    fig.gca().set_aspect('equal')
    fig.gca().set_title('model'+str(i))
    
    plt.plot(front_edge_measured[:,0], front_edge_measured[:,1],'r.-')
    plt.plot(front_edge_symed[:,0], front_edge_symed[:,1], 'r.--')

    plt.plot(rear_edge_measured[:,0], rear_edge_measured[:,1],'g.-')
    plt.plot(rear_edge_symed[:,0], rear_edge_symed[:,1], 'g.--')


    plt.plot(left_edge_measured[:,0], left_edge_measured[:,1],'g.-')
    plt.plot(right_edge_measured[:,0], right_edge_measured[:,1], 'g.-')


    plt.plot(left_mirror_measured[:,0], left_mirror_measured[:,1],'b.-')
    plt.plot(right_mirror_measured[:,0], right_mirror_measured[:,1], 'b.-')

    plt.plot(front_shaft_measured[:,0], front_shaft_measured[:,1],'k.-')
    plt.plot(rear_shaft_measured[:,0], rear_shaft_measured[:,1],'k.-')
    plt.plot(mid_shaft_measued[:,0], mid_shaft_measued[:,1],'k.-')


    plt.plot(front_left_wheel_box_symed[:,0], front_left_wheel_box_symed[:,1],'k.-')
    plt.plot(front_right_wheel_box_symed[:,0], front_right_wheel_box_symed[:,1],'k.-')
    plt.plot(rear_left_wheel_box_symed[:,0], rear_left_wheel_box_symed[:,1],'k.-')
    plt.plot(rear_right_wheel_box_symed[:,0], rear_right_wheel_box_symed[:,1],'k.-')


    plt.grid()
    
    figs.append(fig)
    fig.show()

def readMultiCircleData(filename):
    circles = np.empty([0,3])
    models = []
    with open(filename,'rb') as f:
        docs = yaml.safe_load(f)
        circle_num = docs["circle_num"]
        for i in range(circle_num):
            circle_x = docs["circle"+str(i)]["x"]
            circle_y = docs["circle"+str(i)]["y"]
            circle_r = docs["circle"+str(i)]["r"]
            circles = np.append(circles, np.array([[circle_x, circle_y, circle_r]]), axis=0)

        model_num = docs["model_num"]
        for i in range(model_num):
            model = {}
            model["include_model"] = docs["model"+str(i)]["include_model"]
            model["index"] = docs["model"+str(i)]["index"]
            model["height"] = docs["model"+str(i)]["height"]
            model["description"] = docs["model"+str(i)]["description"]
            
            
            models.append(model)
    return circles, models


readMultiCircleData(circle_data_filename)


lat_inflation = 0.0
lon_inflation = 0.0
lon_inflation_adjuster = lon_inflation - lat_inflation

while True:
    circle_data, model_data = readMultiCircleData(circle_data_filename)

    for i in range(model_num):
        circle_artists = []
        fig = figs[i]
        ax = figs[i].gca()

        ax.set_title('model'+str(i) + ": " + model_data[i]["description"] + "   height:" + model_data[i]["height"] + "   include: " + str(model_data[i]["include_model"]))


        model_circles = model_data[i]["index"]
        model_include_circles = []

        if len(model_data[i]["include_model"]) > 0:
            include_idx = model_data[i]["include_model"][0]
            model_include_circles = model_data[include_idx]["index"]
                
        for ci in range(np.size(circle_data,0)):
            
            if(ci in model_circles):
                circle = plt.Circle((circle_data[ci,0] + lon_inflation_adjuster, circle_data[ci,1]), circle_data[ci,2] + lat_inflation, color='y', alpha=0.5)
                ax.add_artist(circle)
                circle_artists.append(circle)
            
            if(ci in model_include_circles):
                circle = plt.Circle((circle_data[ci,0] + lon_inflation_adjuster, circle_data[ci,1]), circle_data[ci,2] + lat_inflation, color='blue', alpha=0.3)
                ax.add_artist(circle)
                circle_artists.append(circle)


        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(0.1)

        for ci in range(len(circle_artists)):
            circle_artists[ci].remove()
    