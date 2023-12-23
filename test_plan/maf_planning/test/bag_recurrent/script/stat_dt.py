# -*- coding: utf-8 -*-

from pymongo.mongo_client import MongoClient
import json
import copy

setting = "mongodb://developer:123456#ABCDEF@10.10.2.153:8635,10.10.3.29:8635/test?authSource=admin"

PLANNING_GT_SET = "planning_gt"
PLANNING_DT_SET = "planning_dt"

client = MongoClient(setting)

tagged_list = list()

tag_table0 = client['perception_experiment']["apa_90sp_gt/first"]
for item in tag_table0.find():
    tagged_list.append(item)
tag_table1 = client['perception_experiment']["apa_raw_data_zzh_0329_gt/first"]
for item in tag_table1.find():
    tagged_list.append(item)
tag_table2 = client['perception_experiment']["apa_construction_scene_gt/first"]
for item in tag_table2.find():
    tagged_list.append(item)

print("all tag count: {}".format(len(tagged_list)))

gt_name_list = list()
gt_list = list()

tagged_name_list = list()

for gt in client['perception_experiment'][PLANNING_GT_SET].find():
    for ele in tagged_list:
        if ele["_id"].find(gt["name"]) != -1:
            tagged_name_list.append(gt["name"])
            break
    gt_name_list.append(gt["name"])
    gt_list.append(gt)

print("all gt name count: {}".format(len(gt_name_list)))
print("tagged name count: {}".format(len(tagged_name_list)))

dt_list = list()
for dt in client['perception_experiment'][PLANNING_DT_SET].find():
    dt_list.append(dt)

print("all dt count: {}".format(len(dt_list)))

data_list = list()

for dt in dt_list:
    if dt["name"] not in gt_name_list:
        print("ERROR: {} not in gt name list".format(dt["name"]))
        continue

    d = dict()
    d["name"] = dt["name"]
    d["succ"] = len(dt["plan_results"][0]["x"]) != 0
    d["slot_type"] = dt["slot_type"]

    d["tags"] = dict()

    for ele in tagged_list:
        if ele["_id"].find(d["name"]) == -1:
            continue
        if "apa_tags" not in ele:
            continue
        for k, v in ele["apa_tags"].items():
            d["tags"][k] = v

    data_list.append(d)

    
with open("dt.json", "w") as f:
    json.dump(dt_list, f)

example = dict()
example["total"] = 0
example["succ"] = 0
example["rpd"] = 0
example["rate"] = 0

vertical_indoor_carcar_540cm_272cm = copy.copy(example)
vertical_indoor_carcar_540cm_292cm = copy.copy(example)
vertical_indoor_carcar_540cm_312cm = copy.copy(example)

vertical_indoor_inwalloutcar_540cm_272cm = copy.copy(example)
vertical_indoor_inwalloutcar_540cm_292cm = copy.copy(example)
vertical_indoor_inwalloutcar_540cm_312cm = copy.copy(example)

vertical_indoor_incaroutwall_540cm_272cm = copy.copy(example)
vertical_indoor_incaroutwall_540cm_292cm = copy.copy(example)
vertical_indoor_incaroutwall_540cm_312cm = copy.copy(example)

vertical_outdoor_carcar_cement_540cm_272cm = copy.copy(example)
vertical_outdoor_carcar_cement_540cm_292cm = copy.copy(example)
vertical_outdoor_carcar_cement_540cm_312cm = copy.copy(example)

vertical_outdoor_carcar_zhuancao_540cm_272cm = copy.copy(example)
vertical_outdoor_carcar_zhuancao_540cm_292cm = copy.copy(example)
vertical_outdoor_carcar_zhuancao_540cm_312cm = copy.copy(example)

parallel_indoor_carcar_400cm_610cm = copy.copy(example)
parallel_indoor_carcar_400cm_640cm = copy.copy(example)
parallel_indoor_carcar_400cm_670cm = copy.copy(example)

parallel_indoor_inwalloutcar_400cm_610cm = copy.copy(example)
parallel_indoor_inwalloutcar_400cm_640cm = copy.copy(example)
parallel_indoor_inwalloutcar_400cm_670cm = copy.copy(example)

parallel_indoor_incaroutwall_400cm_610cm = copy.copy(example)
parallel_indoor_incaroutwall_400cm_640cm = copy.copy(example)
parallel_indoor_incaroutwall_400cm_670cm = copy.copy(example)

total_vertical_indoor = copy.copy(example)

total_parallel_indoor = copy.copy(example)

generalization_vertical = copy.copy(example)
generalization_parallel = copy.copy(example)


for d in data_list:
    opersite_position = "left"
    if "parkingslot_position" in d["tags"] and \
        d["tags"]["parkingslot_position"] == "left":
        opersite_position = "right"

    if d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.72:
        vertical_indoor_carcar_540cm_272cm["total"] += 1
        if d["succ"]:
            vertical_indoor_carcar_540cm_272cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.92:
        vertical_indoor_carcar_540cm_292cm["total"] += 1
        if d["succ"]:
            vertical_indoor_carcar_540cm_292cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 3.12:
        vertical_indoor_carcar_540cm_312cm["total"] += 1
        if d["succ"]:
            vertical_indoor_carcar_540cm_312cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.72:
        vertical_indoor_inwalloutcar_540cm_272cm["total"] += 1
        if d["succ"]:
            vertical_indoor_inwalloutcar_540cm_272cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.92:
        vertical_indoor_inwalloutcar_540cm_292cm["total"] += 1
        if d["succ"]:
            vertical_indoor_inwalloutcar_540cm_292cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 3.12:
        vertical_indoor_inwalloutcar_540cm_312cm["total"] += 1
        if d["succ"]:
            vertical_indoor_inwalloutcar_540cm_312cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.72:
        vertical_indoor_incaroutwall_540cm_272cm["total"] += 1
        if d["succ"]:
            vertical_indoor_incaroutwall_540cm_272cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.92:
        vertical_indoor_incaroutwall_540cm_292cm["total"] += 1
        if d["succ"]:
            vertical_indoor_incaroutwall_540cm_292cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 3.12:
        vertical_indoor_incaroutwall_540cm_312cm["total"] += 1
        if d["succ"]:
            vertical_indoor_incaroutwall_540cm_312cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        (d["tags"]["road_surface_material"] == "cement" or \
        d["tags"]["road_surface_material"] == "asphalt") and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.72:
        vertical_outdoor_carcar_cement_540cm_272cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_cement_540cm_272cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        (d["tags"]["road_surface_material"] == "cement" or \
        d["tags"]["road_surface_material"] == "asphalt") and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.92:
        vertical_outdoor_carcar_cement_540cm_292cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_cement_540cm_292cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        (d["tags"]["road_surface_material"] == "cement" or \
        d["tags"]["road_surface_material"] == "asphalt") and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 3.12:
        vertical_outdoor_carcar_cement_540cm_312cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_cement_540cm_312cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        d["tags"]["road_surface_material"] == "zhuancao" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.72:
        vertical_outdoor_carcar_zhuancao_540cm_272cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_zhuancao_540cm_272cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        d["tags"]["road_surface_material"] == "zhuancao" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 2.92:
        vertical_outdoor_carcar_zhuancao_540cm_292cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_zhuancao_540cm_292cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.VERTICAL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "outdoor" and \
        d["tags"]["road_surface_material"] == "zhuancao" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 5.4 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 3.12:
        vertical_outdoor_carcar_zhuancao_540cm_312cm["total"] += 1
        if d["succ"]:
            vertical_outdoor_carcar_zhuancao_540cm_312cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.1:
        parallel_indoor_carcar_400cm_610cm["total"] += 1
        if d["succ"]:
            parallel_indoor_carcar_400cm_610cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.4:
        parallel_indoor_carcar_400cm_640cm["total"] += 1
        if d["succ"]:
            parallel_indoor_carcar_400cm_640cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        d["tags"]["parking_obstacles"]["left"] == "car" and \
        d["tags"]["parking_obstacles"]["right"] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.7:
        parallel_indoor_carcar_400cm_670cm["total"] += 1
        if d["succ"]:
            parallel_indoor_carcar_400cm_670cm["succ"] += 1

        #if not d["succ"]:
        #    for gt in gt_list:
        #        if d["name"] == gt["name"]:
        #            break
        #    with open("./{}_gt.json".format(d["name"]), "w") as f:
        #        json.dump(gt, f)

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.1:
        parallel_indoor_inwalloutcar_400cm_610cm["total"] += 1
        if d["succ"]:
            parallel_indoor_inwalloutcar_400cm_610cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.4:
        parallel_indoor_inwalloutcar_400cm_640cm["total"] += 1
        if d["succ"]:
            parallel_indoor_inwalloutcar_400cm_640cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        (d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "pillar" or \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "wall") and \
        d["tags"]["parking_obstacles"][opersite_position] == "car" and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.7:
        parallel_indoor_inwalloutcar_400cm_670cm["total"] += 1
        if d["succ"]:
            parallel_indoor_inwalloutcar_400cm_670cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.1:
        parallel_indoor_incaroutwall_400cm_610cm["total"] += 1
        if d["succ"]:
            parallel_indoor_incaroutwall_400cm_610cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.4:
        parallel_indoor_incaroutwall_400cm_640cm["total"] += 1
        if d["succ"]:
            parallel_indoor_incaroutwall_400cm_640cm["succ"] += 1

    elif d["slot_type"] == "ParkingSlotType.PARALLEL" and \
        "indoor_outdoor" in d["tags"] and \
        d["tags"]["indoor_outdoor"] == "indoor" and \
        "parking_obstacles" in d["tags"] and \
        "parkingslot_position" in d["tags"] and \
        d["tags"]["parking_obstacles"][d["tags"]["parkingslot_position"]] == "car" and \
        (d["tags"]["parking_obstacles"][opersite_position] == "pillar" or
        d["tags"]["parking_obstacles"][opersite_position] == "wall") and \
        "road_space" in d["tags"] and \
        d["tags"]["road_space"] == 4.0 and \
        "parkingslot_space" in d["tags"] and \
        d["tags"]["parkingslot_space"] == 6.7:
        parallel_indoor_incaroutwall_400cm_670cm["total"] += 1
        if d["succ"]:
            parallel_indoor_incaroutwall_400cm_670cm["succ"] += 1

    else:
        #generalization
        if d["slot_type"] == "ParkingSlotType.VERTICAL":
            generalization_vertical["total"] += 1
            if d["succ"]:
                generalization_vertical["succ"] += 1
        else:
            generalization_parallel["total"] += 1
            if d["succ"]:
                generalization_parallel["succ"] += 1

        
def r2(score):
    return round(score, 2)

def calcRate(d):
    if d["total"] == 0 or d["total"] < d["succ"]:
        return
    if d["total"] - d["succ"] != 0:
        d["rpd"] = int(d["total"] / (d["total"] - d["succ"]))
    else:
        d["rpd"] = "100+"
    d["rate"] = r2(d["succ"] / d["total"])

calcRate(vertical_indoor_carcar_540cm_272cm)
calcRate(vertical_indoor_carcar_540cm_292cm)
calcRate(vertical_indoor_carcar_540cm_312cm)

calcRate(vertical_indoor_inwalloutcar_540cm_272cm)
calcRate(vertical_indoor_inwalloutcar_540cm_292cm)
calcRate(vertical_indoor_inwalloutcar_540cm_312cm)

calcRate(vertical_indoor_incaroutwall_540cm_272cm)
calcRate(vertical_indoor_incaroutwall_540cm_292cm)
calcRate(vertical_indoor_incaroutwall_540cm_312cm)

total_vertical_indoor["total"] = \
    vertical_indoor_carcar_540cm_272cm["total"] + \
    vertical_indoor_carcar_540cm_292cm["total"] + \
    vertical_indoor_carcar_540cm_312cm["total"] + \
    vertical_indoor_inwalloutcar_540cm_272cm["total"] + \
    vertical_indoor_inwalloutcar_540cm_292cm["total"] + \
    vertical_indoor_inwalloutcar_540cm_312cm["total"] + \
    vertical_indoor_incaroutwall_540cm_272cm["total"] + \
    vertical_indoor_incaroutwall_540cm_292cm["total"] + \
    vertical_indoor_incaroutwall_540cm_312cm["total"]

total_vertical_indoor["succ"] = \
    vertical_indoor_carcar_540cm_272cm["succ"] + \
    vertical_indoor_carcar_540cm_292cm["succ"] + \
    vertical_indoor_carcar_540cm_312cm["succ"] + \
    vertical_indoor_inwalloutcar_540cm_272cm["succ"] + \
    vertical_indoor_inwalloutcar_540cm_292cm["succ"] + \
    vertical_indoor_inwalloutcar_540cm_312cm["succ"] + \
    vertical_indoor_incaroutwall_540cm_272cm["succ"] + \
    vertical_indoor_incaroutwall_540cm_292cm["succ"] + \
    vertical_indoor_incaroutwall_540cm_312cm["succ"]

calcRate(total_vertical_indoor)

calcRate(vertical_outdoor_carcar_cement_540cm_272cm)
calcRate(vertical_outdoor_carcar_cement_540cm_292cm)
calcRate(vertical_outdoor_carcar_cement_540cm_312cm)

calcRate(vertical_outdoor_carcar_zhuancao_540cm_272cm)
calcRate(vertical_outdoor_carcar_zhuancao_540cm_292cm)
calcRate(vertical_outdoor_carcar_zhuancao_540cm_312cm)

calcRate(parallel_indoor_carcar_400cm_610cm)
calcRate(parallel_indoor_carcar_400cm_640cm)
calcRate(parallel_indoor_carcar_400cm_670cm)

calcRate(parallel_indoor_inwalloutcar_400cm_610cm)
calcRate(parallel_indoor_inwalloutcar_400cm_640cm)
calcRate(parallel_indoor_inwalloutcar_400cm_670cm)

calcRate(parallel_indoor_incaroutwall_400cm_610cm)
calcRate(parallel_indoor_incaroutwall_400cm_640cm)
calcRate(parallel_indoor_incaroutwall_400cm_670cm)

total_parallel_indoor["total"] = \
    parallel_indoor_carcar_400cm_610cm["total"] + \
    parallel_indoor_carcar_400cm_640cm["total"] + \
    parallel_indoor_carcar_400cm_670cm["total"] + \
    parallel_indoor_inwalloutcar_400cm_610cm["total"] + \
    parallel_indoor_inwalloutcar_400cm_640cm["total"] + \
    parallel_indoor_inwalloutcar_400cm_670cm["total"] + \
    parallel_indoor_incaroutwall_400cm_610cm["total"] + \
    parallel_indoor_incaroutwall_400cm_640cm["total"] + \
    parallel_indoor_incaroutwall_400cm_670cm["total"]

total_parallel_indoor["succ"] = \
    parallel_indoor_carcar_400cm_610cm["succ"] + \
    parallel_indoor_carcar_400cm_640cm["succ"] + \
    parallel_indoor_carcar_400cm_670cm["succ"] + \
    parallel_indoor_inwalloutcar_400cm_610cm["succ"] + \
    parallel_indoor_inwalloutcar_400cm_640cm["succ"] + \
    parallel_indoor_inwalloutcar_400cm_670cm["succ"] + \
    parallel_indoor_incaroutwall_400cm_610cm["succ"] + \
    parallel_indoor_incaroutwall_400cm_640cm["succ"] + \
    parallel_indoor_incaroutwall_400cm_670cm["succ"]

calcRate(total_parallel_indoor)

calcRate(generalization_vertical)
calcRate(generalization_parallel)


print("vertical_indoor_carcar_540cm_272cm: {}\n\
vertical_indoor_carcar_540cm_292cm: {}\n\
vertical_indoor_carcar_540cm_312cm: {}\n\
vertical_indoor_inwalloutcar_540cm_272cm: {}\n\
vertical_indoor_inwalloutcar_540cm_292cm: {}\n\
vertical_indoor_inwalloutcar_540cm_312cm: {}\n\
vertical_indoor_incaroutwall_540cm_272cm: {}\n\
vertical_indoor_incaroutwall_540cm_292cm: {}\n\
vertical_indoor_incaroutwall_540cm_312cm: {}\n\
parallel_indoor_carcar_400cm_610cm: {}\n\
parallel_indoor_carcar_400cm_640cm: {}\n\
parallel_indoor_carcar_400cm_670cm: {}\n\
parallel_indoor_inwalloutcar_400cm_610cm: {}\n\
parallel_indoor_inwalloutcar_400cm_640cm: {}\n\
parallel_indoor_inwalloutcar_400cm_670cm: {}\n\
parallel_indoor_incaroutwall_400cm_610cm: {}\n\
parallel_indoor_incaroutwall_400cm_640cm: {}\n\
parallel_indoor_incaroutwall_400cm_670cm: {}\n\
total_vertical_indoor: {}\n\
total_parallel_indoor: {}\n\
generalization_vertical: {}\n\
generalization_parallel: {}\n".format(
    vertical_indoor_carcar_540cm_272cm,
    vertical_indoor_carcar_540cm_292cm,
    vertical_indoor_carcar_540cm_312cm,
    vertical_indoor_inwalloutcar_540cm_272cm,
    vertical_indoor_inwalloutcar_540cm_292cm,
    vertical_indoor_inwalloutcar_540cm_312cm,
    vertical_indoor_incaroutwall_540cm_272cm,
    vertical_indoor_incaroutwall_540cm_292cm,
    vertical_indoor_incaroutwall_540cm_312cm,
    parallel_indoor_carcar_400cm_610cm,
    parallel_indoor_carcar_400cm_640cm,
    parallel_indoor_carcar_400cm_670cm,
    parallel_indoor_inwalloutcar_400cm_610cm,
    parallel_indoor_inwalloutcar_400cm_640cm,
    parallel_indoor_inwalloutcar_400cm_670cm,
    parallel_indoor_incaroutwall_400cm_610cm,
    parallel_indoor_incaroutwall_400cm_640cm,
    parallel_indoor_incaroutwall_400cm_670cm,
    total_vertical_indoor,
    total_parallel_indoor,
    generalization_vertical,
    generalization_parallel,
    )
)

print("OK")

