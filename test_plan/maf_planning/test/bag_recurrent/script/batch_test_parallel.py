import sys
import os
import time
import datetime

from plotter.load_bag import DataLoader
from batch_plan.common import *

def readEgoPoseFromBag(bag_file):
    ego_pose = []
    cmd_types = []
    try:
        dataloader = DataLoader(bag_file)
    except Exception as exp:
        print(exp)
        return ego_pose, cmd_types
    request_msg = dataloader.getTopic(EGO_POSE_TPC)
    ego_info = EgoGenerator(request_msg)
    ego_pose = ego_info.xytheta
    request_cmd = dataloader.getTopic(SYSTEM_CMD_TYPES)
    cmd_info = SystemCmdGenerator(request_cmd)
    cmd_types = cmd_info.cmd_types
    return ego_pose, cmd_types

def readOdoFromBag(bag_file):
    odos = []
    
    try:
        dataloader = DataLoader(bag_file)
    except Exception as exp:
        print(exp)
        return odos
    
    request_msg = dataloader.getTopic(SBP_REQUEST)
    # uss_msg = dataloader.getTopic(USS_OBSTACLE)
    # chass_msg = dataloader.getTopic(CHASSIS_REPORT)
    # uss_gen = PointsGenerator(uss_msg)
    odo_gen = OdoGenerator(request_msg, -1)
    
    odo_t = odo_gen.real_odo
    odos = [odo[0] for odo in odo_t]
    
    return odos
    
    
    # # get odo from chassis
    # chass_t = [(m[0].gear_report.gear_report_data.current_state.value, m[1]) for m in chass_msg]
    # chass_dt = []
    # if len(chass_t) > 1:
    #     for index in range(1,len(chass_t)):
    #         if chass_t[index][0] != chass_t[index-1][0]:
    #             chass_dt.append(chass_t[index])
    
    # for ch in chass_dt:
    #     one_odo_t = odo_gen.atT(ch[1])
    #     if one_odo_t is None or isinstance(one_odo_t, list):
    #         continue
    #     odo_t.append(one_odo_t)
        
    
    # # get uss obs from odo
    # odo_uss = []
    # for odo_one in odo_t:
    #     uss_obs = uss_gen.atT(odo_one[1])
    #     if isinstance(uss_obs, list):
    #         print("no uss data")
    #     odo_uss.append((odo_one[0], uss_obs))
    
    # return odo_uss


if __name__ == '__main__':
    bag_folder = None
    html_path = None
    if len(sys.argv) <2:
        print("PLEASE SPECIFY <BAG_FOLDER or BAG_FILE> <HTML_PATH>")
        exit(0)
        
    if len(sys.argv) >1:
        bag_folder = sys.argv[1]
    if len(sys.argv) >2:
        html_path = sys.argv[2]
    
    bag_files, pickle_file = getAllFiles(bag_folder)
    msg = []
    bag_names = []
    
    if pickle_file is not None:
        msg, bag_names = loadData(pickle_file)
    else:
        msg, bag_names = loadDataFromBag(bag_files, readOdoFromBag)
        dumpData((msg, bag_names), os.path.join(bag_folder, "bag.pickle"))
    
    # ego_pose_list = []
    # cmd_type_list = []
    # for index, file_bag in enumerate(bag_files):
    #     ego_msg, cmd_msg = readEgoPoseFromBag(file_bag)
    #     ego_pose_list.append(ego_msg)
    #     cmd_type_list.append(cmd_msg)

    # odos_no_uss = [m[0] for m in msg]
    # usss = [u[1] for u in msg]
    # odo_list = convertOdo(odos_no_uss, usss)
    
    odo_list = msg
    
    print("run planner with {} frames ...".format(len(odo_list)))
    t1  =time.time()
    sbp_res_list, car_size_params = planOnce(odo_list, apa_plan.ParkingSlotType.PARALLEL)
    t2  =time.time()
    print("\033[0;32;40m planning cost {:.2f}s\033[0m".format(t2-t1))
    
    # calcPlanningEndOffset(bag_names, odo_list, ego_pose_list, cmd_type_list)
    processRes(bag_names, odo_list, sbp_res_list,None, car_size_params, html_path)
