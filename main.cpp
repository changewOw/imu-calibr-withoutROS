
#include"imu_calibr.h"

#include<thread>
#include <fstream>
#include <libobsensor/ObSensor.hpp>
#include <mutex>
#include <atomic>

class FPSCounter {
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

public:
    FPSCounter() : frameCount(0), lastFpsUpdate(Clock::now()), startTime(lastFpsUpdate) {}

    void newFrame() {
        TimePoint currentTime = Clock::now();
        ++frameCount;

        Duration elapsedTime = currentTime - lastFpsUpdate;

        if (elapsedTime >= Duration(std::chrono::seconds(1))) {
            double seconds = std::chrono::duration<double>(elapsedTime).count();
            fps = frameCount / seconds;

            frameCount = 0;
            lastFpsUpdate = currentTime;

            std::cout << "FPS: " << fps << "\n";
        }
    }

private:
    int frameCount;
    double fps;
    TimePoint lastFpsUpdate;
    TimePoint startTime;
};

std::mutex gyr_mtx, acc_mtx;

std::atomic_bool gyr_flag(false);
std::atomic_bool acc_flag(false);

std::shared_ptr<ob::GyroFrame> gGyr_data;
std::shared_ptr<ob::AccelFrame> gAcc_data;


int main()
{
    auto imu_calibr_wrappper = IMU_Calibr(100, 120);



    return 0;
    

    // FPSCounter fps_gyr = FPSCounter();
    // FPSCounter fps_acc = FPSCounter();

    // Print the SDK version number, the SDK version number is divided into major version number, minor version number and revision number
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;

    // Create a Context.
    ob::Context ctx;

    // Query the list of connected devices
    auto devList = ctx.queryDeviceList();

    if(devList->deviceCount() == 0) {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    // Create a device, 0 represents the index of the first device
    auto                        dev         = devList->getDevice(0);
    std::shared_ptr<ob::Sensor> gyroSensor  = nullptr;
    std::shared_ptr<ob::Sensor> accelSensor = nullptr;

    gyroSensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);
    accelSensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);


    if(gyroSensor and accelSensor)
    {
        {
            // Get configuration list
            auto profiles = gyroSensor->getStreamProfileList();
            // Select the first profile to open stream
            auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            // auto profile = profiles->getGyroStreamProfile(OBGyroFullScaleRange::OB_GYRO_FS_1000dps, OBGyroSampleRate::OB_SAMPLE_RATE_500_HZ);
            std::cout<< "Gyr sample rate is "<< profile->as<ob::GyroStreamProfile>()->sampleRate()<<std::endl;

            gyroSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(gyr_mtx);
                gGyr_data = frame->as<ob::GyroFrame>();
                gyr_flag = true;
            });
        }
        {
            auto profiles = accelSensor->getStreamProfileList();
            // Select the first profile to open stream
            auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            accelSensor->start(profile, [](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(acc_mtx);
                gAcc_data = frame->as<ob::AccelFrame>();
                acc_flag = true;
            });
        }
    }

    while(not imu_calibr_wrappper.isEnd())
    {
        if(gyr_flag and acc_flag)
        {
            std::unique_lock<std::mutex> lk1(gyr_mtx);
            std::unique_lock<std::mutex> lk2(acc_mtx);
            gyr_flag = false;
            acc_flag = false;
            const auto& gyr_data = gGyr_data->value();
            const auto& acc_data = gAcc_data->value();
            imu_calibr_wrappper.push(
                gyr_data.x,
                gyr_data.y,
                gyr_data.z,
                acc_data.x,
                acc_data.y,
                acc_data.z,
                std::chrono::system_clock::now()
            );
        }
    }

    imu_calibr_wrappper.show();

    if(gyroSensor) {
        gyroSensor->stop();
    }
    if(accelSensor) {
        accelSensor->stop();
    }

    printf("Finished!!!!\n");

    return 0;
}


/*

type: IMU
name: imu
"Gyr (unit: rad/s)":
  avg-axis:
    gyr_n: 0.00068683492698299296
    gyr_w: 3.099346056950939e-05
  x-axis:
    gyr_n: 0.00065246263943236798
    gyr_w: 2.7122600138178711e-05
  y-axis:
    gyr_n: 0.00077050334957597729
    gyr_w: 3.7774540705968716e-05
  z-axis:
    gyr_n: 0.00063753879194063372
    gyr_w: 2.8083240864380756e-05
"Acc (unit: m/s^2)":
  avg-axis:
    acc_n: 0.0058930384813208119
    acc_w: 0.00021006216254982658
  x-axis:
    acc_n: 0.0051685026614580309
    acc_w: 0.00010061205283148411
  y-axis:
    acc_n: 0.0062591164147610106
    acc_w: 0.00017734186368931256
  z-axis:
    acc_n: 0.0062514963677433935
    acc_w: 0.0003522325711286831



2hrs
type: IMU
name: imu
"Gyr (unit: rad/s)":
  avg-axis:
    gyr_n: 0.00069736788822923263
    gyr_w: 1.8356555632638541e-05
  x-axis:
    gyr_n: 0.00065336305695158647
    gyr_w: 1.9517043769104921e-05
  y-axis:
    gyr_n: 0.00068470957821832178
    gyr_w: 1.9802943488989844e-05
  z-axis:
    gyr_n: 0.00075403102951778931
    gyr_w: 1.574967963982085e-05
"Acc (unit: m/s^2)":
  avg-axis:
    acc_n: 0.006055343242956643
    acc_w: 0.00020022678501542998
  x-axis:
    acc_n: 0.0059668425890045075
    acc_w: 0.00010033769249961657
  y-axis:
    acc_n: 0.0055357476556324716
    acc_w: 0.00014608351950515819
  z-axis:
    acc_n: 0.0066634394842329525
    acc_w: 0.00035425914304151515

*/