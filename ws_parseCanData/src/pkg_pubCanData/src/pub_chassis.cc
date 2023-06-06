#include <ros/ros.h>
#include <std_msgs/String.h>
#include "pkg_pubCanData/wheelSpeed.h"
#include "pkg_pubCanData/vehicleSpeed.h"
#include "pkg_pubCanData/yawRate.h"
#include <ctime>

#include "can_data.hpp"

using pkg_pubCanData::wheelSpeed;
using pkg_pubCanData::vehicleSpeed;
using pkg_pubCanData::yawRate;

int main (int argc, char **argv) {
    ros::init(argc, argv, "pub_chassis");
    ros::NodeHandle nh;
    ros::Publisher pub_wheelSpeed = nh.advertise<pkg_pubCanData::wheelSpeed>("wheelSpeed", 3);
    ros::Publisher pub_vehicleSpeed = nh.advertise<pkg_pubCanData::vehicleSpeed>("vehicleSpeed", 3);
    ros::Publisher pub_yawRate = nh.advertise<pkg_pubCanData::yawRate>("yawRate", 3);

    // string can_name = "can0";
    // string dbc_path = "/home/songj/work/数据采集/data_collection_v2/ws_canData/src/pkg_pubChassisData/dbc/MPC_20220902.dbc";

    string can_name;
    string dbc_path;
    nh.getParam("can_name", can_name);
    nh.getParam("dbc_path", dbc_path);

    CanData* chassis_data = new CanData(can_name, dbc_path);
    chassis_data->loadDBC();
    chassis_data->initCan();
    

    while (ros::ok) {

        read(chassis_data->socket_canfd_, &chassis_data->canfd_data_, sizeof(chassis_data->canfd_data_));

        if (chassis_data->canfd_data_.can_id == BOEachWheelSpeed) {
            auto BOInfo_wheel = chassis_data->BO_messages_[BOEachWheelSpeed];
            auto SGInfos_wheels = BOInfo_wheel->getSignals();
            signal_ST sig_fl_wheel, sig_fr_wheel, sig_rl_wheel, sig_rr_wheel;
            chassis_data->getSignalDBCInfo(SGInfos_wheels, SIG_FL_WHEEL_SPEED, sig_fl_wheel);
            chassis_data->getSignalDBCInfo(SGInfos_wheels, SIG_FR_WHEEL_SPEED, sig_fr_wheel);
            chassis_data->getSignalDBCInfo(SGInfos_wheels, SIG_RL_WHEEL_SPEED, sig_rl_wheel);
            chassis_data->getSignalDBCInfo(SGInfos_wheels, SIG_RR_WHEEL_SPEED, sig_rr_wheel);

            wheelSpeed wheeel_speed;
            wheeel_speed.header.stamp = ros::Time::now();
            wheeel_speed.FLSpeed = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_fl_wheel);
            wheeel_speed.FRSpeed = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_fr_wheel);
            wheeel_speed.RLSpeed = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_rl_wheel);
            wheeel_speed.RRSpeed = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_rr_wheel);

            pub_wheelSpeed.publish(wheeel_speed);
            ROS_INFO("publish wheel speed ok. FL: %f FR: %f RL: %f RR: %f", wheeel_speed.FLSpeed, wheeel_speed.FRSpeed, wheeel_speed.RLSpeed, wheeel_speed.RRSpeed);

        } else if (chassis_data->canfd_data_.can_id == BOCurrentVehicleSpeed) {
            auto BOInfo_vehicle_speed = chassis_data->BO_messages_[BOCurrentVehicleSpeed];
            auto SGInfos_vehicle_speed = BOInfo_vehicle_speed->getSignals();
            signal_ST sig_vehicle_speed;
            chassis_data->getSignalDBCInfo(SGInfos_vehicle_speed, SIG_VEHICLE_SPEED, sig_vehicle_speed);

            vehicleSpeed vehicle_speed;
            vehicle_speed.header.stamp = ros::Time::now();
            vehicle_speed.vehicleSpeed = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_vehicle_speed);

            pub_vehicleSpeed.publish(vehicle_speed);
            ROS_INFO("publish vehicle speed ok. vehicle speed: %f", vehicle_speed.vehicleSpeed);

        } else if (chassis_data->canfd_data_.can_id == BOYawRate) {
            auto BOInfo_yaw_rate = chassis_data->BO_messages_[BOYawRate];
            auto SGInfos_yaw_rate = BOInfo_yaw_rate->getSignals();
            signal_ST sig_yaw_rate;
            chassis_data->getSignalDBCInfo(SGInfos_yaw_rate, SIG_YAW_RATE, sig_yaw_rate);

            yawRate yaw_rate;
            yaw_rate.header.stamp = ros::Time::now();
            yaw_rate.yawRate = chassis_data->calSignalValue(chassis_data->canfd_data_, sig_yaw_rate);
            ROS_INFO("publish yaw rate ok. vehicle speed: %f", yaw_rate.yawRate);

        }

    }

    return 0;
}

