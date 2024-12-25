#pragma once

#include <iomanip>
#include <chrono>
#include <shared_mutex>
#include "acc_lib/allan_acc.h"
#include "acc_lib/fitallan_acc.h"
#include "gyr_lib/allan_gyr.h"
#include "gyr_lib/fitallan_gyr.h"


class IMU_Calibr
{
private:
    /* data */
    bool _start = true;
    bool _end = false;
    int _max_time_min = 10;

    std::chrono::system_clock::time_point _start_t;
    std::string _data_save_path;

public:
    std::shared_ptr<imu::AllanGyr> gyr_x;
    std::shared_ptr<imu::AllanGyr> gyr_y;
    std::shared_ptr<imu::AllanGyr> gyr_z;
    std::shared_ptr<imu::AllanAcc> acc_x;
    std::shared_ptr<imu::AllanAcc> acc_y;
    std::shared_ptr<imu::AllanAcc> acc_z;

    IMU_Calibr(int max_cluster, int max_time_min=10, std::string data_save_path="./"); // default is 10 mins
    ~IMU_Calibr();
    
    void push(
        double gyr_x,
        double gyr_y,
        double gyr_z,
        double acc_x,
        double acc_y,
        double acc_z,
        std::chrono::system_clock::time_point time
    );

    bool isEnd()
    {
        return _end;
    }

    void show();


};

