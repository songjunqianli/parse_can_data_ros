#pragma once

#include <iostream>
#include <cstring>
#include <string>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
// #include "gflags/gflags.h"
// #include "glog/logging.h"
// #include "json/json.h"
#include "database.hpp"

#define SIG_FL_WHEEL_SPEED "Wheel_Speed_FL_1F0_S" 
#define SIG_FR_WHEEL_SPEED "Wheel_Speed_FR_1F0_S" 
#define SIG_RL_WHEEL_SPEED "Wheel_Speed_RL_1F0_S" 
#define SIG_RR_WHEEL_SPEED "Wheel_Speed_RR_1F0_S" 
#define SIG_VEHICLE_SPEED "IPB_Vehicle_Speed_S"
#define SIG_YAW_RATE "Yaw_Rate_Signal_S"

#define RADAR_CANBO_START 32
#define RADAR_CANBO_P2_END 71
#define RADAR_CANBO_P1_START 80
#define RADAR_CANBO_END 119
#define RADAR_CANBO_NUM 80

#define CAN_CHASSIS "can0"
#define CAN_RADAR "can1"

using namespace std;
using AS::CAN::DbcLoader::AttributeType;
using AS::CAN::DbcLoader::Database;
using AS::CAN::DbcLoader::DbcObjType;
using AS::CAN::DbcLoader::EnumAttribute;
using AS::CAN::DbcLoader::FloatAttribute;
using AS::CAN::DbcLoader::IntAttribute;
using AS::CAN::DbcLoader::StringAttribute;
using AS::CAN::DbcLoader::Message;
using AS::CAN::DbcLoader::Order;

enum CanFrame_ID {
    BOCurrentVehicleSpeed = 289,
    BOEachWheelSpeed = 496,
    BOYawRate = 546   // Yaw Rate Signal ---------但是不确定是否就是自车的yaw信息---------
};

enum BOPart1_SigStartBit {
    MotionPattern = 31,
    YPos_Stdev = 22,
    XVelRel_Stdev = 32,
    XPos_Stdev = 14,
    XAccRel = 39,
    ValidFlag = 23,
    UpdateFlag = 15,
    ObstacleProb = 28,
    ID = 7,
    ExstProb = 41
};

enum BOPart2_SigStartBit {
    YVelRel = 47,
    YPos = 12,
    XVelRel = 7,
    XPos = 31,
    ObjType = 33,
    MeasFlag = 52
};

struct signal_ST {
    string signal_name;
    Order code_format_;
    uint start_bit = 0;
    uint length = 0;
    float factor = 0.0;
    float offset = 0.0f;
    string unit;    // 信号的单位
};


class CanData
{
private:
    string can_device_;
    string dbc_file_path_;

    can_filter chassis_filter_[3];
    uint8_t raw_socket_;    // 
    sockaddr_can addr_;
    Database *dbc_ = NULL;

    // const Message *BO_vehicle_speed_;
    // const Message *BO_wheel_speed_;
    // const Message *BO_yaw_rate_;
    // vector<signal_ST> signals_;
    // unordered_map< CanFrame_ID, vector<signal_ST> > dbc_chassis_;
    int crossByte(const int &start,const int &size);
    int crossBytesLocation(const int &start,const int &size, const int &crossbyte);
    int bit_mask(const int &nbits);


public:
    int socket_canfd_;
    canfd_frame canfd_data_;    // 接收can数据
    double fl_speed_, fr_speed_, rl_speed_, rr_speed_;
    double vehicle_speed_, yaw_rate_;

    std::unordered_map<unsigned int, const Message *> BO_messages_;
    CanData(const string &canx, const string &dbc_path);
    ~CanData();

    // 接收数据
    void initCan();

    // load DBC file
    void loadDBC();

    // 解析数据
    void getSignalDBCInfo(unordered_map<std::string, const AS::CAN::DbcLoader::Signal *> SignalsInfo, 
                          const string &signal_name, 
                          signal_ST &signal_info);
    double calSignalValue(const canfd_frame &canfd_data, const signal_ST &signal_info);

    void parseChassisSignal();
    double test_parseSpecifiedSignal(const uint &bo_id , const string &signal_name, const canfd_frame &can_data);
    // void parseVehicleSpeed();
    // void parseWheelSpeed();
    // void parseYawRate();
    // void parseCanFrame();
    // void testParse_IPB_0x220(const canfd_frame& canfd_data);

};