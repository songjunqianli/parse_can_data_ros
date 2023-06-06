#include <ros/ros.h>
#include <std_msgs/String.h>
#include "pkg_pubCanData/radarObject.h"
#include "pkg_pubCanData/radarObjects.h"
#include <ctime>

#include "can_data.hpp"

using pkg_pubCanData::radarObject;
using pkg_pubCanData::radarObjects;

/**
 * @brief 解析毫米波雷达数据
 * 
 * @param canfd_map 一帧的canfd数据
 * @param BO_messages 毫米波雷达的报文定义信息
 */
void pubRadarData(unordered_map<uint, canfd_frame> canfd_map, CanData* radarCan, ros::Publisher pub_radar) {
    
    // 存放所有报文的数据  通过报文ID检索  每一个报文对应的是其包含的信号的信息
    unordered_map<uint, unordered_map<uint, float>> boValue_map;
    for (auto &canfd : canfd_map) {
        auto bo_id = canfd.first;
        auto canfd_radar_data = canfd.second;
        auto signals = radarCan->BO_messages_[bo_id]->getSignals();
        unordered_map<uint, float> sigValue_map;

        //根据DBC中信号的定义来解析can data中信号的数据
        for (auto &sig : signals) {
            signal_ST sigDbcInfo;
            sigDbcInfo.code_format_ = sig.second->getEndianness();
            sigDbcInfo.factor = sig.second->getFactor();
            sigDbcInfo.length = sig.second->getLength();
            sigDbcInfo.offset = sig.second->getOffset();
            sigDbcInfo.signal_name = sig.second->getName();
            sigDbcInfo.start_bit = sig.second->getStartBit();
            sigDbcInfo.unit = sig.second->getUnit();

            float sig_value = radarCan->calSignalValue(canfd_radar_data, sigDbcInfo);
            sigValue_map.emplace(sigDbcInfo.start_bit, sig_value);
        }

        boValue_map.emplace(bo_id, sigValue_map);

    }

    if(boValue_map.size() == RADAR_CANBO_NUM) {
        radarObjects radarObjects_msg;
        uint object_num = RADAR_CANBO_NUM/2;
        for(uint i = 0; i < object_num; i++) {
            radarObject radarObject_msg;
            // part2的数据
            radarObject_msg.YVelRel = boValue_map[i+RADAR_CANBO_START][YVelRel];
            radarObject_msg.YPos = boValue_map[i+RADAR_CANBO_START][YPos];
            radarObject_msg.XVelRel = boValue_map[i+RADAR_CANBO_START][XVelRel];
            radarObject_msg.XPos = boValue_map[i+RADAR_CANBO_START][XPos];
            radarObject_msg.ObjType = boValue_map[i+RADAR_CANBO_START][ObjType];
            radarObject_msg.MeasFlag = boValue_map[i+RADAR_CANBO_START][MeasFlag];
            // part1的数据
            radarObject_msg.MotionPattern = boValue_map[i+RADAR_CANBO_P1_START][MotionPattern];
            radarObject_msg.YPos_Stdev = boValue_map[i+RADAR_CANBO_P1_START][YPos_Stdev];
            radarObject_msg.XVelRel_Stdev = boValue_map[i+RADAR_CANBO_P1_START][XVelRel];
            radarObject_msg.XPos_Stdev = boValue_map[i+RADAR_CANBO_P1_START][XPos_Stdev];
            radarObject_msg.XAccRel = boValue_map[i+RADAR_CANBO_P1_START][XAccRel];
            radarObject_msg.ValidFlag = boValue_map[i+RADAR_CANBO_P1_START][ValidFlag];
            radarObject_msg.UpdateFlag = boValue_map[i+RADAR_CANBO_P1_START][UpdateFlag];
            radarObject_msg.ObstacleProb = boValue_map[i+RADAR_CANBO_P1_START][ObstacleProb];
            radarObject_msg.ID = boValue_map[i+RADAR_CANBO_P1_START][ID];
            radarObject_msg.ExstProb = boValue_map[i+RADAR_CANBO_P1_START][ExstProb];

            ROS_INFO_STREAM( "obj " << i << ": " 
                            << "x_pos_vel: " <<  radarObject_msg.XPos << " " << radarObject_msg.XVelRel << ", " 
                            << "y_pos_vel: " <<  radarObject_msg.YPos << " " << radarObject_msg.YVelRel);
            radarObjects_msg.radarObjects.push_back(radarObject_msg);
        }

        radarObjects_msg.header.stamp = ros::Time::now();
        pub_radar.publish(radarObjects_msg);
        ROS_INFO("========Pub Radar info ok.========");
    }
    

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "pub_radar");
    ros::NodeHandle nh;
    ros::Publisher pub_radarObjects = nh.advertise<pkg_pubCanData::radarObjects>("radarObjects", 3);

    // string can_name = "can0";
    // string dbc_path = "/home/songj/work/数据采集/data_collection_v2/ws_canData/src/pkg_pubChassisData/dbc/MPC_20220902.dbc";

    string can_name;
    string dbc_path;
    nh.getParam("can_name", can_name);
    nh.getParam("dbc_path", dbc_path);

    CanData* radar_data = new CanData(can_name, dbc_path);
    radar_data->loadDBC();
    radar_data->initCan();
    
    vector<canfd_frame> canfd_vector;
    unordered_map<uint, canfd_frame> canfd_map;

    while (ros::ok) {

        // 循环读取一帧的数据，一共80个canfd data
        read(radar_data->socket_canfd_, &radar_data->canfd_data_, sizeof(radar_data->canfd_data_));
        if (radar_data->canfd_data_.can_id == RADAR_CANBO_START) {
            // canfd_vector.push_back(radar_data->canfd_data_);
            canfd_map.emplace(radar_data->canfd_data_.can_id, radar_data->canfd_data_);

            while (radar_data->canfd_data_.can_id < RADAR_CANBO_END) {
                read(radar_data->socket_canfd_, &radar_data->canfd_data_, sizeof(radar_data->canfd_data_));
                // canfd_vector.push_back(radar_data->canfd_data_);
                canfd_map.emplace(radar_data->canfd_data_.can_id, radar_data->canfd_data_);
            }
        }

        // 将容器中的数据进行处理，需要判断容器中数据的大小是否满足80个canfd数据
        if(canfd_map.size() == RADAR_CANBO_NUM) {
            pubRadarData(canfd_map, radar_data, pub_radarObjects);
        }


        // canfd_vector.clear();
        canfd_map.clear();

    }

    return 0;
}

