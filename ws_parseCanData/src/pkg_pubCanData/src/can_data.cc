#include "can_data.hpp"
#include <ros/ros.h>

CanData::CanData(const string &canx, const string &dbc_path) {
    can_device_ = canx;
    dbc_file_path_ = dbc_path;
}

CanData::~CanData() {
}

int CanData::crossByte(const int &start,const int &size){
    int tmp = (start % 8)+1 -size;

    if( tmp < 0 ){
        if(tmp < -16){
            return 4;
        }
        else if(tmp < -8){
            return 3;
        }
        else
            return 2;
    }
    return 0;
}

int CanData::crossBytesLocation(const int &start,const int &size, const int &crossbyte){
    int offset = 0;
    if(crossbyte == 2)
        offset = 8;
    else if(crossbyte == 3)
        offset = 16;
    else if(crossbyte == 4)
        offset = 24;
    int ret = 1 + offset -(size - (start % 8));
    return ret ;
}

int CanData::bit_mask(const int &nbits){
    int i;
    int ret = 1;
    for(i = 0; i < nbits -1;i++)
    {
        ret = ret * 2+1;
    }
    return ret;
}

// 接收数据
void CanData::initCan() {
    // 创建socket can
    socket_canfd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_canfd_ < 0) {
        ROS_ERROR("Socket can creat error!");
        return;
    }

    /********************* 绑定 canx 设备与 socket *********************/
    ifreq ifr;  // if.h
    strcpy(ifr.ifr_name, can_device_.c_str());
    ioctl(socket_canfd_, SIOCGIFINDEX, &ifr); // 指定编号为 canx 的设备，获取设备索引

    sockaddr_can addr;
    addr.can_family = AF_CAN;  // 指定协议族
    addr.can_ifindex = ifr.ifr_ifindex;  // 设备索引
    // 将套接字与 can0 绑定
    int bind_res = bind(socket_canfd_, (sockaddr *)&addr, sizeof(addr));
    if(bind_res < 0) {
        ROS_ERROR("Bind error!");
        return;
    }


int canfd_on = 1;
setsockopt(socket_canfd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

    /********************* 过滤规则设置 *********************/
    // CAN_SFF_MASK 0x000007FFU  (SFF: standard frame format)
    // 此处设置三组过滤规则，只接收 ID 为 xxx、xxx、xxx 的三种数据帧

    //  can0连接到底盘的can口
    // if(can_device_ == CAN_CHASSIS) {
    //     // 轮速信息
    //     chassis_filter_[0].can_id = BOEachWheelSpeed;
    //     chassis_filter_[0].can_mask = CAN_SFF_MASK; // 标准帧 (SFF: standard frame format)

    //     // Current vehicle speed
    //     chassis_filter_[1].can_id = BOCurrentVehicleSpeed;
    //     chassis_filter_[1].can_mask = CAN_SFF_MASK; // 标准帧 (SFF: standard frame format)

    //     // Yaw Rate Signal ---------不确定是否就是自车的yaw信息---------
    //     chassis_filter_[2].can_id = BOYawRate;
    //     chassis_filter_[2].can_mask = CAN_SFF_MASK; // 标准帧 (SFF: standard frame format)

    //     if (setsockopt(socket_canfd_, SOL_CAN_RAW, CAN_RAW_FILTER, &chassis_filter_, sizeof(chassis_filter_)) > -1) {
    //         LOG(INFO) << "Set sockopt of can0 ok!";
    //     }
    // }else if (can_device_ == CAN_RADAR) {
    //     can_filter filter[1];
    //     filter[0].can_id = 0;
    //     filter[0].can_mask = 0;
    //     if(setsockopt(socket_canfd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) > -1) {
    //         LOG(INFO) << "Set sockopt of can1 ok!";
    //     }
    // }


}

//load DBC file
void CanData::loadDBC() {
    dbc_ = new Database(dbc_file_path_);
    //获取所有的报文信息
    BO_messages_ = dbc_->getMessages();

}

void CanData::getSignalDBCInfo(unordered_map<std::string, const AS::CAN::DbcLoader::Signal *> SignalsInfo, 
                               const string &signal_name,
                               signal_ST &signal_info) {

    signal_info.code_format_ = SignalsInfo[signal_name]->getEndianness();
    signal_info.factor = SignalsInfo[signal_name]->getFactor();
    signal_info.length = static_cast<unsigned int>(SignalsInfo[signal_name]->getLength());
    signal_info.start_bit = static_cast<unsigned int>(SignalsInfo[signal_name]->getStartBit());
    signal_info.offset = SignalsInfo[signal_name]->getOffset();
    signal_info.signal_name = SignalsInfo[signal_name]->getName();
    signal_info.unit = SignalsInfo[signal_name]->getUnit();

}

double CanData::calSignalValue(const canfd_frame &canfd_data, const signal_ST &signal_info) {
    double ret;
    int64_t result = 0;

    if (signal_info.code_format_ == Order::LE) {
        uint8_t *data_int = (uint8_t *)&(canfd_data.data);
        unsigned short bit = (unsigned short)signal_info.start_bit;
        for (int bitpos = 0; bitpos < signal_info.length; bitpos++) {
            if (data_int[bit / 8] & (1 << (bit % 8))) {
                // if (signal_info.code_format_ == Order::LE) {
                    result |= (1ULL << bitpos);
                // } 
                // else {
                //     result |= (1ULL << (signal_info.length - bitpos - 1));
                // }
            }
            bit++;
        }            
    } else {
        uint8_t *data_int = (uint8_t *)&(canfd_data.data);

        int crossBytes = crossByte(signal_info.start_bit, signal_info.length);
        if(crossBytes){
            int offset = crossBytesLocation(signal_info.start_bit, signal_info.length, crossBytes);
            if (crossBytes == 2) {
                result = ((((data_int[signal_info.start_bit/8])<<8) | (data_int[signal_info.start_bit/8+1])) >> (offset)) 
                         & bit_mask(signal_info.length);
            } else if(crossBytes == 3) {
                result =((((data_int[signal_info.start_bit/8])<<16) | (data_int[signal_info.start_bit/8+1]<<8)|data_int[signal_info.start_bit/8+2]) >> (offset)) 
                         & bit_mask(signal_info.length);
            }
        } else {
            result = ( (data_int[signal_info.start_bit/8] ) >> (signal_info.start_bit%8 - signal_info.length +1)) & bit_mask(signal_info.length);
        }

    }

    ret = (double)result * signal_info.factor + signal_info.offset;
    return ret;
}

void CanData::parseChassisSignal() {

    while(read(socket_canfd_, &canfd_data_, sizeof(canfd_data_)) > 0) {

        // LOG(INFO) << "          BO ID : " << canfd_data_.can_id << ", " << "BO DLC : " << static_cast<unsigned int>(canfd_data_.len);

        // for(int i=0; i < static_cast<unsigned int>(canfd_data_.len); i++) {
        //     LOG(INFO) << "            DATA" << "[" << i << "] : " << canfd_data_.data[i];
        // }
    
        if (canfd_data_.can_id == BOEachWheelSpeed) {

            ROS_INFO("========BO ID: %d", canfd_data_.can_id);
            // 读取DBC中信号的信息
            auto BOInfo_wheel = BO_messages_[BOEachWheelSpeed];
            auto SGInfos_wheels = BOInfo_wheel->getSignals();
            signal_ST sig_fl_wheel, sig_fr_wheel, sig_rl_wheel, sig_rr_wheel;
            
            getSignalDBCInfo(SGInfos_wheels, SIG_FL_WHEEL_SPEED, sig_fl_wheel);
            fl_speed_ = calSignalValue(canfd_data_, sig_fl_wheel);
            getSignalDBCInfo(SGInfos_wheels, SIG_FR_WHEEL_SPEED, sig_fr_wheel);
            fr_speed_ = calSignalValue(canfd_data_, sig_fr_wheel);
            getSignalDBCInfo(SGInfos_wheels, SIG_RL_WHEEL_SPEED, sig_rl_wheel);
            rl_speed_ = calSignalValue(canfd_data_, sig_rl_wheel);
            getSignalDBCInfo(SGInfos_wheels, SIG_RR_WHEEL_SPEED, sig_rr_wheel);
            rr_speed_ = calSignalValue(canfd_data_, sig_rr_wheel);

            // LOG(INFO) << "      wheel speed: " << fl_speed_ <<", "
            //                                    << fr_speed_ <<", "
            //                                    << rl_speed_ <<", "
            //                                    << rr_speed_;

        } else if (canfd_data_.can_id == BOCurrentVehicleSpeed) {

            ROS_INFO("========BO ID: %d", canfd_data_.can_id);
            auto BOInfo_vehicle_speed = BO_messages_[BOCurrentVehicleSpeed];
            auto SGInfos_vehicle_speed = BOInfo_vehicle_speed->getSignals();
            signal_ST sig_vehicle_speed;
            
            getSignalDBCInfo(SGInfos_vehicle_speed, SIG_VEHICLE_SPEED, sig_vehicle_speed);
            vehicle_speed_ = calSignalValue(canfd_data_, sig_vehicle_speed);

        } else if (canfd_data_.can_id == BOYawRate) {

            ROS_INFO("========BO ID: %d", canfd_data_.can_id);
            auto BOInfo_yaw_rate = BO_messages_[BOYawRate];
            auto SGInfos_yaw_rate = BOInfo_yaw_rate->getSignals();
            signal_ST sig_yaw_rate;

            getSignalDBCInfo(SGInfos_yaw_rate, SIG_YAW_RATE, sig_yaw_rate);
            yaw_rate_ = calSignalValue(canfd_data_, sig_yaw_rate);

        }
    
    }


}

double CanData::test_parseSpecifiedSignal(const uint &bo_id , const string &signal_name,  const canfd_frame &can_data) {

    if (can_data.can_id == bo_id) {
        // 读取DBC中信号的信息
        auto BOInfo = BO_messages_[bo_id];
        auto SGInfos = BOInfo->getSignals();
        signal_ST signal_info;
        signal_info.code_format_ = SGInfos[signal_name]->getEndianness();
        signal_info.factor = SGInfos[signal_name]->getFactor();
        signal_info.length = static_cast<unsigned int>(SGInfos[signal_name]->getLength());
        signal_info.start_bit = static_cast<unsigned int>(SGInfos[signal_name]->getStartBit());
        signal_info.offset = SGInfos[signal_name]->getOffset();
        signal_info.signal_name = SGInfos[signal_name]->getName();
        signal_info.unit = SGInfos[signal_name]->getUnit();
        bool isSigned = SGInfos[signal_name]->isSigned();

        int64_t result = 0;

        if (signal_info.code_format_ == Order::LE) {
            uint8_t *data_int = (uint8_t *)&(can_data.data);
            unsigned short bit = (unsigned short)signal_info.start_bit;
            for (int bitpos = 0; bitpos < signal_info.length; bitpos++) {
                if (data_int[bit / 8] & (1 << (bit % 8))) {
                    // if (signal_info.code_format_ == Order::LE) {
                        result |= (1ULL << bitpos);
                    // } 
                    // else {
                    //     result |= (1ULL << (signal_info.length - bitpos - 1));
                    // }
                }
                bit++;
            }            
        } else {
            uint8_t *data_int = (uint8_t *)&(can_data.data);
            int crossBytes = crossByte(signal_info.start_bit, signal_info.length);
            if(crossBytes){
                int offset = crossBytesLocation(signal_info.start_bit, signal_info.length, crossBytes);
                if (crossBytes == 2) {
                    result = ((((data_int[signal_info.start_bit/8])<<8) | (data_int[signal_info.start_bit/8+1])) >> (offset)) 
                             & bit_mask(signal_info.length);
                } else if(crossBytes == 3) {
                    result =((((data_int[signal_info.start_bit/8])<<16) | (data_int[signal_info.start_bit/8+1]<<8)|data_int[signal_info.start_bit/8+2]) >> (offset)) 
                             & bit_mask(signal_info.length);
                }
            } else {
                result = ( (data_int[signal_info.start_bit/8] ) >> (signal_info.start_bit%8 - signal_info.length +1)) & bit_mask(signal_info.length);
            }

        }

        // LOG(INFO) << "Result: " << (double)result * signal_info.factor + signal_info.offset;
        return (double)result * signal_info.factor + signal_info.offset;
    } else {
        return 0;
    }

}


