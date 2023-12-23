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

if (os.path.exists('../result')):
    shutil.rmtree('../result')
os.mkdir('../result')
if (os.path.exists('../build/combined_trajectory.txt')):
    os.remove('../build/combined_trajectory.txt')
save_success_plot = True  # 是否保存成功场景，默认关闭
save_fail_plot = True
disabled_scenario = [160, 218, 270, 271, 284, 285, 286, 297, 299, 300, 301, 302, 306, 308, 313, 316,
                     317, 318, 362, 362, 365, 379, 380, 484, 485, 486, 487, 488, 489, 490, 1015, 1016,
                     1017, 1018, 1106, 1107, 1265, 1266, 1267, 1345, 1346, 1406, 1407, 1408, 1549, 1550,
                     1551, 1602, 1688, 1689, 1690, 1695, 1696, 1708, 1709, 1724, 1798, 1799, 1802, 1801,
                     1803, 1865, 2000, 2001, 2002, 2003, 2013, 2014, 2039, 2040, 2043, 2067, 2068, 2069,
                     2106, 2107, 2108, 2134, 2135, 2138, 2155, 2156, 2157, 2164, 2165, 2166, 2180, 2204,
                     2205, 2206, 2208, 2209, 2210, 2235, 2345, 2348, 2349, 2353, 2357, 2358, 2363, 2364, 
                     2365, 2376, 2379, 2383, 2385, 2388, 2390, 2397, 2400, 2401, 2426, 2427, 2428, 2461, 
                     2426, 2427, 2428, 2462, 2463, 2484, 2485, 2486, 2487, 2788,
                     2648, 2666, 2667, 2668, 2670, 2679, 2680,
                     2687, 2688, 2689, 2690, 2822, 2823, 2538, 2540, 2541,
                     2862, 2870, 2871, 2872, 2877, 2888, 2895, 2903, 2904, 2913, 2917, 2918, 2919, 2958,
                     2959, 2960, 2962, 2963, 2964, 2966, 2968, 2969, 2970, 2971, 2975]

classical_scenario = [345, 1734, 1735, 1736, 1737, 1738, 1739, 1744, 1745, 1746, 1840, 1842, 1844, 1845, 
                    1850, 1853, 1975,  2051, 2299, 2457, 2911, 3130, 3133, 3141, 3154, 3251]
if __name__ == '__main__':
    path_base = "../scenario/"
    file_name_base = "new"
    file_path_base = path_base + file_name_base + ".json"
    current_dir = os.getcwd()
    success_scene_cache = []
    fail_scene_cache = []
    if os.path.exists(path_base) and os.path.isdir(path_base):
        file_counter = len([f for f in os.listdir(path_base)
                           if os.path.isfile(os.path.join(path_base, f))])
        print(f"folder '{path_base}' file num: ", file_counter)
    else:
        print(f"folder '{path_base}' does not exit")
    file_counter -= 1
    for i in range(0, file_counter):
        if i not in classical_scenario:
            continue
        suffix = "_" + str(i)
        file_name = path_base + file_name_base + suffix + ".json"

        with open(file_name) as f:
            data_sbp = json.load(f)

        vehicle_start_x = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["x"]
        vehicle_start_y = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["y"]
        vehicle_start_theta = data_sbp['OpenspaceDeciderOutput']["target_state"]["path_point"]["theta"]
        slot_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["x"]
        slot_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["y"]

        data = pbw.ParseBagJson()
        data.init(file_name)
        data.file_name = file_name_base + suffix + ".json"
        print(data.file_name)
        success = data.plot_trajectory(
            data.target.x, data.target.y, data.target.theta)

        if (success):
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("sucess:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
            print("success:", math.cos(vehicle_start_theta),
                  math.sin(vehicle_start_theta))
            success_scene_cache.append([vs_x, vs_y, ve_x - vs_x, ve_y - vs_y])
            if save_success_plot:
                plt.ion()
                data.plot_obstacle_box()
                data.plot_points()
                data.plot_lines()
                data.plot_obstacle_lines()
                data.plot_car(data.target.x, data.target.y, data.target.theta)
                data.plot_car(data.init_point.x, data.init_point.y,
                              data.init_point.theta, True)
                plt.xlim((slot_start_x - 15, slot_start_x + 15))
                plt.ylim((slot_start_y - 15, slot_start_y + 15))
                # time.sleep(1)
                if(os.path.exists('../build/combined_trajectory.txt')):
                    final_trajectory = np.loadtxt(
                        '../build/combined_trajectory.txt', delimiter=' ')
                    # 提取x和y坐标
                    x = final_trajectory[:, 0]
                    y = final_trajectory[:, 1]

                    # 绘制折线图，并设置线的粗细
                    plt.plot(x, y, linewidth=0.25)
                # if (success):
                plt.savefig("../result/" + file_name_base +
                            suffix + ".png", dpi=1200)
                plt.ioff()
        else:
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("fail:", suffix,  [vs_x, vs_y,
                  ve_x, ve_y], vehicle_start_theta)
            print("fail:", math.cos(vehicle_start_theta),
                  math.sin(vehicle_start_theta))
            fail_scene_cache.append(
                [suffix, vs_x, vs_y, ve_x - vs_x, ve_y - vs_y])
            if save_fail_plot:
                plt.ion()
                data.plot_obstacle_box()
                data.plot_points()
                data.plot_lines()
                data.plot_obstacle_lines()
                data.plot_car(data.target.x, data.target.y, data.target.theta)
                data.plot_car(data.init_point.x, data.init_point.y,
                              data.init_point.theta, True)
                plt.xlim((slot_start_x - 15, slot_start_x + 15))
                plt.ylim((slot_start_y - 15, slot_start_y + 15))
                # if (success):
                plt.savefig("../result/" + file_name_base +
                            suffix + ".png", dpi=1200)
                plt.ioff()

        print("finish")
        print("*******************************************")
        print()
        plt.close("all")
        # plt.show()

    plt.close("all")
    print("finish")
    # for pose in success_scene_cache:
    #     print(pose)

    # suffix_to_draw = "_" + str(0).zfill(4)
    # file_name_to_draw = path_base + file_name_base + suffix_to_draw + ".json"

    # with open(file_name) as f:
    #     data_last = json.load(f)
    # slot_todraw_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["x"]
    # slot_todraw_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"]["path_point"]["y"]

    # data_to_draw = pbw.ParseBagJson()
    # data_to_draw.init(file_name_to_draw)
    # data_to_draw.file_name = file_name_base + suffix_to_draw + ".json"

    # plt.ion()
    # data_to_draw.plot_obstacle_box()
    # data_to_draw.plot_points()
    # data_to_draw.plot_lines()
    # data_to_draw.plot_obstacle_lines()
    # # data_to_draw.plot_car(data_to_draw.target.x, data_to_draw.target.y, data_to_draw.target.theta)
    # data_to_draw.plot_slot(data_to_draw.init_point.x, data_to_draw.init_point.y, data_to_draw.init_point.theta, 2.5, 5.5)
    # data_to_draw.plot_possible_pose(success_scene_cache, fail_scene_cache)
    # plt.xlim((slot_todraw_start_x - 15, slot_todraw_start_x + 15))
    # plt.ylim((slot_todraw_start_y - 15, slot_todraw_start_y + 15))
    # plt.savefig("./result/" + file_name_base + "last_result" + ".png" , dpi = 1200)
    # plt.ioff()

    # result = {"sucess_scenario" : success_scene_cache, "fail_scenario" : fail_scene_cache}

    # file_name_last = "./result/" + file_name_base + "_last_result" + ".json"
    # with open(file_name_last,'w') as file_obj:
    #     json.dump(result, file_obj, indent = 4)

    print("total number: ", len(success_scene_cache) + len(fail_scene_cache))
    print("success number: ", len(success_scene_cache))
    print("fail number: ", len(fail_scene_cache))
    for fail_scene in fail_scene_cache:
        print(fail_scene[0])
    print("success rate: ", len(success_scene_cache) /
          (len(success_scene_cache) + len(fail_scene_cache)))
    # plt.show()
    exit(0)

    # with open(file_path_base) as f:
    #     data = json.load(f)

    # vehicle_start_x = data['OpenspaceDeciderOutput']["target_state"]["path_point"]["x"]
    # vehicle_start_y = data['OpenspaceDeciderOutput']["target_state"]["path_point"]["y"]
    # count = 0

    for i in range(0, 10000):
        suffix = "_" + str(i)
        file_name = path_base + file_name_base + suffix + ".json"

        with open(file_name) as f:
            data_sbp = json.load(f)

        vehicle_start_x = data_sbp['OpenspaceDeciderOutput']["target_state"][
            "path_point"]["x"]
        vehicle_start_y = data_sbp['OpenspaceDeciderOutput']["target_state"][
            "path_point"]["y"]
        vehicle_start_theta = data_sbp['OpenspaceDeciderOutput']["target_state"][
            "path_point"]["theta"]
        slot_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"][
            "path_point"]["x"]
        slot_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"][
            "path_point"]["y"]

        data = pbw.ParseBagJson()
        data.init(file_name)
        data.file_name = file_name_base + suffix + ".json"
        print(data.file_name)
        success = data.plot_trajectory(data.target.x, data.target.y,
                                       data.target.theta)

        if (success):
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("sucess:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
            print("success:", math.cos(vehicle_start_theta),
                  math.sin(vehicle_start_theta))
            success_scene_cache.append([vs_x, vs_y, ve_x - vs_x, ve_y - vs_y])
            plt.ion()
            data.plot_obstacle_box()
            data.plot_points()
            data.plot_lines()
            data.plot_obstacle_lines()
            data.plot_car(data.target.x, data.target.y, data.target.theta)
            # data.plot_slot(data.init_point.x, data.init_point.y, data.init_point.theta, 2.5, 5.5)
            data.plot_car(data.init_point.x, data.init_point.y,
                          data.init_point.theta, False)
            plt.xlim((slot_start_x - 15, slot_start_x + 15))
            plt.ylim((slot_start_y - 15, slot_start_y + 15))
            # if (success):
            plt.savefig("../result/" + file_name_base +
                        suffix + ".png", dpi=1200)
            plt.ioff()
        else:
            vs_x = vehicle_start_x
            vs_y = vehicle_start_y
            ve_x = vehicle_start_x + math.cos(vehicle_start_theta) * 0.09
            ve_y = vehicle_start_y + math.sin(vehicle_start_theta) * 0.09
            print("fail:", [vs_x, vs_y, ve_x, ve_y], vehicle_start_theta)
            print("fail:", math.cos(vehicle_start_theta),
                  math.sin(vehicle_start_theta))
            fail_scene_cache.append([vs_x, vs_y, ve_x - vs_x, ve_y - vs_y])
            plt.ion()
            data.plot_obstacle_box()
            data.plot_points()
            data.plot_lines()
            data.plot_obstacle_lines()
            data.plot_car(data.target.x, data.target.y, data.target.theta)
            # data.plot_slot(data.init_point.x, data.init_point.y,
            #                data.init_point.theta, 2.5, 5.5)
            data.plot_car(data.init_point.x, data.init_point.y,
                          data.init_point.theta, False)
            plt.xlim((slot_start_x - 15, slot_start_x + 15))
            plt.ylim((slot_start_y - 15, slot_start_y + 15))
            # if (success):
            plt.savefig("../result/" + file_name_base +
                        suffix + ".png", dpi=1200)
            plt.ioff()

        print("finish")
        print("*******************************************")
        print()
        plt.close("all")
        # plt.show()

    plt.close("all")
    print("finish")
    for pose in success_scene_cache:
        print(pose)

    suffix_to_draw = "_" + str(0).zfill(4)
    file_name_to_draw = path_base + file_name_base + suffix_to_draw + ".json"

    with open(file_name) as f:
        data_last = json.load(f)
    slot_todraw_start_x = data_sbp['OpenspaceDeciderOutput']["init_state"][
        "path_point"]["x"]
    slot_todraw_start_y = data_sbp['OpenspaceDeciderOutput']["init_state"][
        "path_point"]["y"]

    data_to_draw = pbw.ParseBagJson()
    data_to_draw.init(file_name_to_draw)
    data_to_draw.file_name = file_name_base + suffix_to_draw + ".json"

    plt.ion()
    data_to_draw.plot_obstacle_box()
    data_to_draw.plot_points()
    data_to_draw.plot_lines()
    data_to_draw.plot_obstacle_lines()
    # data_to_draw.plot_car(data_to_draw.target.x, data_to_draw.target.y, data_to_draw.target.theta)
    data_to_draw.plot_slot(data_to_draw.init_point.x, data_to_draw.init_point.y,
                           data_to_draw.init_point.theta, 2.5, 5.5)
    data_to_draw.plot_possible_pose(success_scene_cache, fail_scene_cache)
    plt.xlim((slot_todraw_start_x - 15, slot_todraw_start_x + 15))
    plt.ylim((slot_todraw_start_y - 15, slot_todraw_start_y + 15))
    plt.savefig("./result/" + file_name_base +
                "last_result" + ".png", dpi=1200)
    plt.ioff()

    result = {
        "sucess_scenario": success_scene_cache,
        "fail_scenario": fail_scene_cache
    }

    file_name_last = "./result/" + file_name_base + "_last_result" + ".json"
    with open(file_name_last, 'w') as file_obj:
        json.dump(result, file_obj, indent=4)

    print("success number:", len(success_scene_cache))
    print("fail number:", len(fail_scene_cache))
    plt.show()
