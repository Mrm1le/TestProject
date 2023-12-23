import os
import time
import json
import sys
import copy
import math
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point
from autolab_core import RigidTransform
import pymongo
from pymongo.mongo_client import MongoClient
from plotter.utils.common import GEO_TYPE
from evaluation.event import EventLidarTagged
from evaluation.event import EventReal

GEO_TYPE[0] = '14'
MONGO_SETTING = "mongodb://developer:123456#ABCDEF@10.10.2.153:8635,10.10.3.29:8635/test?authSource=admin&readPreference=nearest&localThresholdMS=999999"
GT_COLLECTION_NAME = 'planning_gt'
gt_save_rootpath = '/home/ros/Downloads/test_results/'
mongo_client = MongoClient(MONGO_SETTING)

class BagToGt(object):
    def __init__(self, bag, bag_path, label_path=None, lidar_poses_path=None, lidar_config_path=None):
        self.bag = bag
        self.bag_path = bag_path
        self.label_path = label_path
        self.lidar_poses_path = lidar_poses_path
        self.lidar_config_path = lidar_config_path
        self.mongo_client = MongoClient(MONGO_SETTING)
        self.labelset_metas = {}
        self.datasets_effective_path = './'

    def process(self):
        print('{} process begin'.format(self.bag['_id']))
        if self.label_path != None and self.lidar_poses_path != None and self.lidar_config_path != None:
            return self.process_perfect_gt()
        else:
            return self.process_real_gt()
    
    def process_perfect_gt(self):
        print('{} process_perfect_gt'.format(self.bag['_id']))
        labelset_meta = {}
        if 'ego_lidar_transform' not in self.bag['bag_metas']:
            labelset_meta['gt_error'] = 'not ego_lidar_transform'
            labelset_meta['has_error'] = True
            return labelset_meta

        ego_lidar_transform = self.bag['bag_metas']['ego_lidar_transform']
        base_ego_pose = ego_lidar_transform['ego']
        base_lidar_pose = ego_lidar_transform['lidar']

        ego_pose_quaternion = np.asarray([
            base_ego_pose['q_w'], base_ego_pose['q_x'], base_ego_pose['q_y'], base_ego_pose['q_z']
        ])
        ego_pose_translation = np.asarray(
            [base_ego_pose['x'], base_ego_pose['y'], base_ego_pose['z']])
        ladar_pose_quaternion = np.asarray([
            base_lidar_pose['q_w'], base_lidar_pose['q_x'], base_lidar_pose['q_y'], base_lidar_pose['q_z']
        ])
        ladar_pose_translation = np.asarray(
            [base_lidar_pose['x'], base_lidar_pose['y'], base_lidar_pose['z']])

        ego_qua2rota = RigidTransform(
            ego_pose_quaternion, ego_pose_translation)
        ladar_qua2rota = RigidTransform(
            ladar_pose_quaternion, ladar_pose_translation)
        # self.ladar_ego_transformer = ego_qua2rota * ladar_qua2rota.inverse()
        self.ego_ladar_transformer = ladar_qua2rota * ego_qua2rota.inverse()

        corners = copy.deepcopy(self.bag['bag_metas']['target_slot']['corners'])
        for c in corners:
            ego_local_pose = np.asarray(
                [c['x'], c['y'], c['z']])
            ego_global_pose = self.ego_ladar_transformer.rotation.dot(
                ego_local_pose) + self.ego_ladar_transformer.translation
            c['x'] = ego_global_pose[0]
            c['y'] = ego_global_pose[1]
            c['z'] = ego_global_pose[2]

        ei = {}
        ei['name'] = self.bag['_id']
        ei['label_result_path'] = self.label_path
        ei['lidar_pose_path'] = self.lidar_poses_path
        ei['lidar_config_path'] = self.lidar_config_path
        ei['begin_time_offset'] = 0

        begin_time = time.time()
        event_perfect = EventLidarTagged()
        event_perfect.setSlotCorners(corners)
        success = event_perfect.init(ei)
        if success == False:
            labelset_meta['gt_error'] = 'EventLidarTagged error'
            labelset_meta['has_error'] = True
            return labelset_meta

        gt_file_path = os.path.join(
            gt_save_rootpath, bag['_id'] + '_gt.json')
        if os.path.exists(gt_file_path):
            os.remove(gt_file_path)

        event_perfect.forkReplanEvents()

        dt_file_path = os.path.join(
            gt_save_rootpath, bag['_id'] + '_dt.json')
        if os.path.exists(dt_file_path):
            os.remove(dt_file_path)

        gt_json = json.load(open(gt_file_path, 'r'))

        if 'odo' not in gt_json:
            labelset_meta['gt_error'] = 'no odo'
            labelset_meta['has_error'] = True
            return labelset_meta

        gt_json['_id'] = bag['_id']
        gt_json['md5'] = bag['md5']
        gt_json['is_expert'] = True
        gt_json['is_real'] = True
        mongo_client['perception_experiment'][GT_COLLECTION_NAME].delete_one({
            '_id': gt_json['_id']})
        mongo_result = mongo_client['perception_experiment'][GT_COLLECTION_NAME].insert_one(
            gt_json)
        if mongo_result != None and mongo_result.inserted_id != None:
            labelset_meta['gt_id'] = mongo_result.inserted_id
        else:
            labelset_meta['gt_error'] = 'save gt error'
            labelset_meta['has_error'] = True
        return labelset_meta
        
    def process_real_gt(self):
        labelset_meta = {}
        ei = {}
        ei['name'] = self.bag['_id']
        ei['real_bag_path'] = self.bag_path
        ei['begin_time_offset'] = 0

        event_real = EventReal()
        if 'bag_metas' in self.bag and 'target_slot' in self.bag['bag_metas']:
            event_real.setSlotCorners(self.bag['bag_metas']['target_slot']['corners'])
        success = event_real.init(ei)
        if success == False:
            labelset_meta['gt_error'] = 'EventReal error'
            labelset_meta['has_error'] = True
            return labelset_meta

        gt_file_path = os.path.join(
            gt_save_rootpath, bag['_id'] + '_gt.json')
        if os.path.exists(gt_file_path):
            os.remove(gt_file_path)

        event_real.forkReplanEvents()

        dt_file_path = os.path.join(
            gt_save_rootpath, bag['_id'] + '_dt.json')
        if os.path.exists(dt_file_path):
            os.remove(dt_file_path)

        gt_json = json.load(open(gt_file_path, 'r'))

        if 'odo' not in gt_json:
            labelset_meta['gt_error'] = 'no odo'
            labelset_meta['has_error'] = True
            return labelset_meta

        gt_json['_id'] = bag['_id']
        gt_json['md5'] = bag['md5']
        gt_json['is_expert'] = False
        gt_json['is_real'] = True
        mongo_client['perception_experiment'][GT_COLLECTION_NAME].delete_one({
            '_id': gt_json['_id']})
        mongo_result = mongo_client['perception_experiment'][GT_COLLECTION_NAME].insert_one(
            gt_json)
        if mongo_result != None and mongo_result.inserted_id != None:
            labelset_meta['gt_id'] = mongo_result.inserted_id
        else:
            labelset_meta['gt_error'] = 'save gt error'
            labelset_meta['has_error'] = True
        return labelset_meta

if __name__ == '__main__':

    bags_existed_ = mongo_client['perception_experiment'][GT_COLLECTION_NAME].find({
        "slot_type":"ParkingSlotType.OBLIQUE"
    })

    bag_ids_existed = []
    for b in bags_existed_:
        bag_ids_existed.append(b['_id'])

    mongo_client = pymongo.MongoClient(MONGO_SETTING, connect=False)
    results = mongo_client['perception_experiment']['apa_events_dev/first'].find(
        {
            'apa_tags.parkingslot_type': 'oblique',
            'bag_metas.target_slot.type': 'oblique',
            'bag_metas.target_slot.yaw': {'$exists':True},
            'bag_metas.parking_in': {'$exists':True},
            'no_cam_md5': {'$exists': True},
            '_id': {'$nin': bag_ids_existed}
        }
    ).sort([
        ("_id", pymongo.ASCENDING)])
    
    bags = []
    for r in results:
        yaw1 = r['bag_metas']['target_slot']['yaw']
        yaw2 = r['bag_metas']['parking_in']['begin_pose']['yaw']
        yaw_delta = int(abs((yaw1-yaw2) * 180) / math.pi) % 360
        if yaw_delta > 180:
            yaw_delta = 360 - yaw_delta
        if yaw_delta >=0 and yaw_delta<= 90:
            bags.append(r)
    
    print(len(bags))
    
    cache_dirs=[
            '/home/ros/Downloads/sdb/bags_ori', '/home/ros/Downloads/sdc/bags_ori']

    perfect_count = 0
    real_count = 0
    invalid_count = 0
    for bag in bags:
        bag_path = None
        label_path = None
        lidar_poses_path = None
        lidar_config_path = None

        for cd in cache_dirs:
            _bag_path = os.path.join(cd, bag['_id'], 'trim.bag')
            if os.path.exists(_bag_path):
                bag_path = _bag_path
                break
        for cd in cache_dirs:
            _label_path = os.path.join(cd, bag['_id'], '{}.json'.format(bag['_id']))
            if os.path.exists(_label_path):
                label_path = _label_path
                break
        for cd in cache_dirs:
            _lidar_poses_path = os.path.join(cd, bag['_id'], 'lidar_poses.txt')
            if os.path.exists(_lidar_poses_path):
                lidar_poses_path = _lidar_poses_path
                break
        for cd in cache_dirs:
            _lidar_config_path = os.path.join(cd, bag['_id'], 'lidar.yaml')
            if os.path.exists(_lidar_config_path):
                lidar_config_path = _lidar_config_path
                break

        if label_path != None and lidar_poses_path != None and lidar_config_path != None:
            p = BagToGt(bag=bag, bag_path=bag_path, label_path=label_path, lidar_poses_path=lidar_poses_path, lidar_config_path=lidar_config_path)
            labelset_meta = p.process()
            if 'has_error' in labelset_meta:
                print('{} : {}'.format(bag['_id'], labelset_meta['gt_error']))
            else:
                perfect_count += 1
        elif bag_path != None and bag['is_expert'] == False:
            p = BagToGt(bag=bag, bag_path=bag_path)
            labelset_meta = p.process()
            if 'has_error' in labelset_meta:
                print('{} : {}'.format(bag['_id'], labelset_meta['gt_error']))
            else:
                real_count += 1
        else:
            invalid_count += 1
    
    print('bags: {}, perfect-gt: {}, real-gt: {}, invalid_count: {}, existed: {}'.format(len(bags), perfect_count, real_count, invalid_count, len(bag_ids_existed)))