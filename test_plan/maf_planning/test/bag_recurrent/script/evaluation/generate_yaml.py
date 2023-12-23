# -*- coding: utf-8 -*-

import os
import yaml

CUR_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = '{}/event_config.yaml'.format(CUR_DIR)


def findFilesRecursively(cur_dir, file_list):
    sub_list = os.listdir(cur_dir)
    d = dict()
    for sub_file in sub_list:
        if sub_file == 'event_config.yaml':
            continue
        sub_path = os.path.join(cur_dir, sub_file)
        if os.path.isfile(sub_path):
            if sub_path.endswith('.yaml'):
                d['lidar_config_path'] = sub_path
            elif sub_path.endswith('.json'):
                d['label_result_path'] = sub_path
            elif sub_path.endswith('.txt'):
                d['lidar_pose_path'] = sub_path
        else:
            findFilesRecursively(sub_path, file_list)
    if len(d) != 0:
        file_list.append(d)


def main():
    conf = dict()
    conf['lidar_tagged'] = list()
    file_list = list()
    findFilesRecursively(CUR_DIR, file_list)
    file_list.sort(key=lambda ele: ele['label_result_path'])
    
    index = 1
    for d in file_list:
        d['name'] = 'good event parallel No.{}'.format(index)
        d['begin_time_offset'] = 0
        conf['lidar_tagged'].append(d)
        index += 1
    
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(conf, f)

if __name__ == '__main__':
    main()