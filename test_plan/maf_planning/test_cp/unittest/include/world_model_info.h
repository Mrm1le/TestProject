#pragma once
#include "ros_cpp_convert.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

#define private public
#define protected public

#include "common/parking_world_model.h"
#include "maf_interface/include/maf_interface/maf_perception_interface.h"

struct ParkingSlotPointTypeEnum;
namespace msquare{
namespace parking{

using PerceptionFusionObjectResult = perception_interface_msgs::PerceptionFusionObjectResult;
using PerceptionFusionObjectResultConstPtr = perception_interface_msgs::PerceptionFusionObjectResultConstPtr;


inline std::vector<std::string> getWorldModelTopics();
void getWorldModel(std::shared_ptr<WorldModel> &world_model, std::string bag_file_name, int frame_id);
void feedWorldModel(std::shared_ptr<WorldModel> &model,  rosbag::MessageInstance const frame);
void bagReadTest(std::string bag_file_name);
void updateEgoPose(std::shared_ptr<WorldModel> &model,  mla_localization_msgs::MLALocalizationConstPtr& ego_pose);
void updateFusionObject(std::shared_ptr<WorldModel> &model,  PerceptionFusionObjectResultConstPtr & ros_fusion_object);
void updateGroundLine(std::shared_ptr<WorldModel> &model,  perception_interface_msgs::FusionGroundLineResultConstPtr & ros_ground_line);
void updateFusionApa(std::shared_ptr<WorldModel> &model,  worldmodel_msgs::FusionAPAConstPtr fusion_apa);

inline void getWorldModel(std::shared_ptr<WorldModel> &world_model, std::string bag_file_name, int frame_id){
    rosbag::Bag bag;
    bag.open(bag_file_name, rosbag::bagmode::Read);
    std::vector<std::string> topics=getWorldModelTopics();
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frame_count = 0;
    std::cout<<"bag size:"<< view.size() <<std::endl;
    frame_id = (frame_id >= view.size()) ? 0 : frame_id;

    if(view.size() == 0){
        std::cout<<"no useful topic frame."<<std::endl;
        return;
    }

    for(rosbag::MessageInstance const m : view)
    {   
        if(frame_count == frame_id){
            feedWorldModel(world_model, m);
        } 
        ++frame_count;
    }

}

inline std::vector<std::string> getWorldModelTopics(){
    std::vector<std::string> topics={
        "/mla/egopose",     //
        "/perception/fusion/object_parking_environment",
        "/perception/fusion/ground_line",    // ground line
        "/worldmodel/parking_slot_info"
    };

    return topics;
}

inline void feedWorldModel(std::shared_ptr<WorldModel> &model,  rosbag::MessageInstance const frame){
    mla_localization_msgs::MLALocalizationConstPtr ego_local = frame.instantiate<mla_localization_msgs::MLALocalization>();
    updateEgoPose(model, ego_local);

    PerceptionFusionObjectResultConstPtr fusion_object = frame.instantiate<PerceptionFusionObjectResult>();
    // updateFusionObject(model, fusion_object);

    perception_interface_msgs::FusionGroundLineResultConstPtr ground_line = frame.instantiate<perception_interface_msgs::FusionGroundLineResult>();
    // updateGroundLine(model, ground_line);

    worldmodel_msgs::FusionAPAConstPtr ros_fusion_apa = frame.instantiate<worldmodel_msgs::FusionAPA>();
    // updateFusionApa(model, ros_fusion_apa);

}

inline void updateFusionApa(std::shared_ptr<WorldModel> &model,  worldmodel_msgs::FusionAPAConstPtr ros_fusion_apa){
    std::shared_ptr<maf_worldmodel::FusionAPA> received_fusion_apa = convertSlots(*ros_fusion_apa);
    maf_worldmodel::FusionAPA fusion_apa{};
    if (received_fusion_apa->available & FusionAPA::PARKING_SLOTS) {
      fusion_apa.available |= FusionAPA::PARKING_SLOTS;
      fusion_apa.parking_slots = received_fusion_apa->parking_slots;
    }

    if (received_fusion_apa->available & FusionAPA::EGO_PARKING_SLOT_TRACK_ID) {
      fusion_apa.available |= FusionAPA::EGO_PARKING_SLOT_TRACK_ID;
      fusion_apa.ego_parking_slot_track_id =
          received_fusion_apa->ego_parking_slot_track_id;
    }

    if (received_fusion_apa->available & FusionAPA::EGO_PARKING_SLOT_MAP_ID) {
      fusion_apa.available |= FusionAPA::EGO_PARKING_SLOT_MAP_ID;
      fusion_apa.ego_parking_slot_map_id =
          received_fusion_apa->ego_parking_slot_map_id;
    }

    if (received_fusion_apa->available &
        FusionAPA::SUGGESTED_PARKING_SLOT_TRACK_ID) {
      fusion_apa.available |= FusionAPA::SUGGESTED_PARKING_SLOT_TRACK_ID;
      fusion_apa.suggested_parking_slot_track_id =
          received_fusion_apa->suggested_parking_slot_track_id;
    }

    if (received_fusion_apa->available &
        FusionAPA::SUGGESTED_PARKING_SLOT_MAP_ID) {
      fusion_apa.available |= FusionAPA::SUGGESTED_PARKING_SLOT_MAP_ID;
      fusion_apa.suggested_parking_slot_map_id =
          received_fusion_apa->suggested_parking_slot_map_id;
    }

    if (received_fusion_apa->available & FusionAPA::RESERVED_INFO) {
      fusion_apa.available |= FusionAPA::RESERVED_INFO;
      fusion_apa.reserved_info = received_fusion_apa->reserved_info;
    }


    std::vector<msquare::ParkingLotDetectionInfo> ret;
    ret.clear();
    ret.resize(fusion_apa.parking_slots.size());
    if ((fusion_apa.available & FusionAPA::PARKING_SLOTS) &&
      (fusion_apa.available & FusionAPA::EGO_PARKING_SLOT_TRACK_ID) &&
      (fusion_apa.available & FusionAPA::EGO_PARKING_SLOT_MAP_ID)) {
        auto &input = fusion_apa.parking_slots;
        
        for (size_t i = 0; i < input.size(); ++i) {
        msquare::ParkingLotDetectionInfo parking_lot_info;
        parking_lot_info.corners.resize(
            input[i].parking_slot.local_points_fusion.size());
        for (size_t j = 0; j < input[i].parking_slot.local_points_fusion.size();
            ++j) {
            msquare::ParkingLotDetectionInfo::CornerPoint corner;
            corner.position.x = input[i].parking_slot.local_points_fusion[j].x;
            corner.position.y = input[i].parking_slot.local_points_fusion[j].y;
            corner.position.z = input[i].parking_slot.local_points_fusion[j].z;
            corner.confidence =
                input[i].parking_slot.local_points_fusion_confidence[j];
            corner.is_visible =
                (input[i].parking_slot.local_points_fusion_type[j].value ==
                1);
            parking_lot_info.corners[j] = corner;
        }
        parking_lot_info.is_good = true;
        parking_lot_info.is_empty = (input[i].apa_info.status == APAInfo::VACANT);
        parking_lot_info.is_on_map_list = (input[i].apa_info.map_id > 0);
        parking_lot_info.id = parking_lot_info.is_on_map_list
                                    ? input[i].apa_info.map_id
                                    : input[i].parking_slot.track_id;
        parking_lot_info.is_car_in =
            parking_lot_info.is_on_map_list
                ? (fusion_apa.ego_parking_slot_map_id ==
                    parking_lot_info.id)
                : (fusion_apa.ego_parking_slot_track_id ==
                    parking_lot_info.id);
        ret[i] = parking_lot_info;
        }
    }

    model->feed_parking_lots_detection_fusion_results(ret);

}

inline void updateGroundLine(std::shared_ptr<WorldModel> &model,  perception_interface_msgs::FusionGroundLineResultConstPtr & ros_ground_line){
    std::shared_ptr<maf_perception_interface::FusionGroundLineResult> maf_ground_line = convertFusionGroundlines(*ros_ground_line);
    std::vector<maf_perception_interface::FusionGroundLineData> ground_line_data = maf_ground_line->ground_line.ground_line_data;

    std::vector<GroundLine> ground_line_fusion_array;
    ground_line_fusion_array.resize(ground_line_data.size());
    for (size_t i = 0; i < ground_line_data.size(); ++i) {
        ground_line_fusion_array[i].id = ground_line_data[i].track_id;
        ground_line_fusion_array[i].type = static_cast<GroundLineType>(ground_line_data[i].type.value);
        for(auto local_p : ground_line_data[i].local_points_fusion){
            Point3D value;
            value.x = local_p.x;
            value.y = local_p.y;
            value.z = local_p.z;
            ground_line_fusion_array[i].pts.push_back(value);
        }
    }


    std::vector<FusionFreespacePoint> freespace_array;
    freespace_array.clear();
    for (size_t i = 0; i < ground_line_data.size(); ++i) {
      for (size_t j = 0; j < ground_line_data[i].local_points_fusion.size(); j++) {
        FusionFreespacePoint fs_point;
        fs_point.position.x = ground_line_data[i].local_points_fusion[j].x;
        fs_point.position.y = ground_line_data[i].local_points_fusion[j].y;
        fs_point.position.z = ground_line_data[i].local_points_fusion[j].z;
        fs_point.confidence = ground_line_data[i].local_points_fusion_confidence[j];
        fs_point.type = FusionFreespacePointType::PFUSION_FREESPACE_UNKNOWN;
        freespace_array.emplace_back(fs_point);
      }
    }

    model->feed_parking_ground_line_fusion(ground_line_fusion_array);
    model->feed_parking_fusion_freespace(freespace_array);
}

inline void updateFusionObject(std::shared_ptr<WorldModel> &model,  PerceptionFusionObjectResultConstPtr & ros_fusion_object){
    std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult> maf_fusion_object = convertFusionObjects(*ros_fusion_object);
    std::vector<FusionObject> fusion_array;

    std::vector<maf_perception_interface::PerceptionFusionObjectData> fusion_object_data = maf_fusion_object->perception_fusion_objects_data;
    fusion_array.resize(fusion_object_data.size());
    for (size_t i = 0; i < fusion_object_data.size(); ++i) {
        fusion_array[i].track_id = fusion_object_data[i].track_id;
        fusion_array[i].type = static_cast<FusionObjectType>(from_msd_fusion_type(fusion_object_data[i].type_info));
        fusion_array[i].is_static = fusion_object_data[i].is_static;
        fusion_array[i].shape.height = fusion_object_data[i].shape.height;
        fusion_array[i].shape.length = fusion_object_data[i].shape.length;
        fusion_array[i].shape.width = fusion_object_data[i].shape.width;
        fusion_array[i].position.x = fusion_object_data[i].position.x;
        fusion_array[i].position.y = fusion_object_data[i].position.y;
        fusion_array[i].position.z = fusion_object_data[i].position.z;
        fusion_array[i].velocity.x = fusion_object_data[i].velocity.x;
        fusion_array[i].velocity.y = fusion_object_data[i].velocity.y;
        fusion_array[i].velocity.z = fusion_object_data[i].velocity.z;
        fusion_array[i].relative_position.x = fusion_object_data[i].relative_position.x;
        fusion_array[i].relative_position.y = fusion_object_data[i].relative_position.y;
        fusion_array[i].relative_velocity.x = fusion_object_data[i].velocity_relative_to_ground.x;
        fusion_array[i].relative_velocity.y = fusion_object_data[i].velocity_relative_to_ground.y;
        fusion_array[i].heading_yaw = fusion_object_data[i].heading_yaw;
        fusion_array[i].relative_heading_yaw = fusion_object_data[i].relative_heading_yaw;
    }
    model->feed_parking_fusion_car(fusion_array);  
}

inline void updateEgoPose(std::shared_ptr<WorldModel> &model,  mla_localization_msgs::MLALocalizationConstPtr& mla_localization){
    convertEgoPose(*mla_localization);

    Pose3D enu;
    Pose2D pose;

    enu.position.x = mla_localization->position.position_local.x;
    enu.position.y = mla_localization->position.position_local.y;
    enu.position.z = mla_localization->position.position_local.z;

    enu.orientation.x =
        mla_localization->orientation.quaternion_local.x;
    enu.orientation.y =
        mla_localization->orientation.quaternion_local.y;
    enu.orientation.z =
        mla_localization->orientation.quaternion_local.z;
    enu.orientation.w =
        mla_localization->orientation.quaternion_local.w;
    
    double yaw = std::atan2(
        2 * (enu.orientation.w * enu.orientation.z + enu.orientation.x * enu.orientation.y),
        1 - 2 * (enu.orientation.y * enu.orientation.y + enu.orientation.z * enu.orientation.z)
    );
    
    pose.x = enu.position.x;
    pose.y = enu.position.y;
    pose.theta = yaw;
   
    model->feed_localization_status(true);
    model->feed_ego_pose(pose);
    model->feed_ego_enu(enu);

    // update frenet coord
}

inline void bagReadTest(std::string bag_file_name){
    rosbag::Bag bag;
    bag.open(bag_file_name, rosbag::bagmode::Read);
    std::vector<std::string> topics=getWorldModelTopics();
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frame_count = 0;
    std::cout<<"bag size:"<< view.size() <<std::endl;

    for(rosbag::MessageInstance const m : view)
    {   
        auto ego_local = m.instantiate<mla_localization_msgs::MLALocalization>();
        std::cout<<frame_count;
        std::cout<<"\t x:" << ego_local->position.position_local.x;
        std::cout<<"\t y:" << ego_local->position.position_local.y;
        std::cout<<"\t z:" << ego_local->position.position_local.z;
        std::cout<<std::endl; 
        frame_count ++;   
    }

}




}   // end namespace parking
}   // end namespace msquare


