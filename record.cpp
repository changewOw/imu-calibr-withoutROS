
#include <thread>
#include <fstream>
#include <libobsensor/ObSensor.hpp>
#include <mutex>
#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <assert.h>
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



typedef struct thrData_t
{
    double x,y,z;
    uint64_t timestamp;
};


std::atomic_bool gStop(false);
std::shared_ptr<cv::Mat> gColorMat;
std::mutex color_mtx, gyr_mtx, acc_mtx;
std::vector<thrData_t> gyr_arr, acc_arr;




// Save the color image in png format
void saveColor(std::shared_ptr<ob::ColorFrame> colorFrame, uint64_t index) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string colorName = "./dataset/cam0/" + std::to_string(index) + ".png";
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    // cv::imshow("Display window", colorRawMat);
    gColorMat = std::make_shared<cv::Mat>(colorRawMat);
}


void showRGB()
{
    while(not gStop)
    {
        {
            std::unique_lock<std::mutex> lk(color_mtx);
            if(gColorMat){
               cv::imshow("Display window", *gColorMat.get());
            }
        }

        
        cv::waitKey(1);
    }

}



int main()
{
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

    FPSCounter fps_color = FPSCounter();
    FPSCounter fps_gyr = FPSCounter();
    FPSCounter fps_acc = FPSCounter();

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
    std::shared_ptr<ob::Sensor> colorSensor = nullptr;

    gyroSensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);
    accelSensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);
    colorSensor = dev->getSensorList()->getSensor(OB_SENSOR_COLOR);

    // Create a format conversion Filter
    ob::FormatConvertFilter formatConvertFilter;

    if(gyroSensor and accelSensor and colorSensor)
    {
        {
            // Get configuration list
            auto profiles = colorSensor->getStreamProfileList();
            // Select the first profile to open stream
            // auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            auto profile = profiles->getVideoStreamProfile(1280, 800, OB_FORMAT_MJPG, 30);

            colorSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame){
                std::unique_lock<std::mutex> lk(color_mtx);
                auto time_stamp = frame->timeStampUs();
                auto color_frame = frame->as<ob::ColorFrame>();
                uint64_t time_stamp_ns = time_stamp * 1000;
                // std::cout<<time_stamp_ns<<std::endl;
                fps_color.newFrame();

                assert(color_frame->format() == OB_FORMAT_MJPG);
                formatConvertFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
                color_frame = formatConvertFilter.process(color_frame)->as<ob::ColorFrame>();

                formatConvertFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
                color_frame = formatConvertFilter.process(color_frame)->as<ob::ColorFrame>();
                saveColor(color_frame, time_stamp_ns);
            });
        }
        {
            // Get configuration list
            auto profiles = gyroSensor->getStreamProfileList();
            // Select the first profile to open stream
            auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            // auto profile = profiles->getGyroStreamProfile(OBGyroFullScaleRange::OB_GYRO_FS_1000dps, OBGyroSampleRate::OB_SAMPLE_RATE_500_HZ);

            gyroSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(gyr_mtx);
                auto timeStamp = frame->timeStampUs();
                auto gyroFrame = frame->as<ob::GyroFrame>();

                uint64_t timeStamp_ns = timeStamp * 1000;
                const auto& value = gyroFrame->value();
                thrData_t data;
                data.x = value.x;
                data.y = value.y;
                data.z = value.z;
                data.timestamp = timeStamp_ns;
                gyr_arr.push_back(data);

                fps_gyr.newFrame();
            });
        }
        {
            // Get configuration list
            auto profiles = accelSensor->getStreamProfileList();
            // Select the first profile to open stream
            auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            // auto profile = profiles->getAccelStreamProfile(OBAccelFullScaleRange::OB_ACCEL_FS_2g, OBAccelSampleRate::OB_SAMPLE_RATE_500_HZ);

            accelSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(acc_mtx);
                auto timeStamp = frame->timeStampUs();
                auto accFrame = frame->as<ob::AccelFrame>();

                uint64_t timeStamp_ns = timeStamp * 1000;
                const auto& value = accFrame->value();
                thrData_t data;
                data.x = value.x;
                data.y = value.y;
                data.z = value.z;
                data.timestamp = timeStamp_ns;
                acc_arr.push_back(data);

                fps_acc.newFrame();
            });
        }
    }else{
        return 0;
    }
    

    // std::thread th_show(showRGB);

    while(not gStop) {
        // Get the value of the pressed key, if it is the ESC key, exit the program
        int key = cv::waitKey(1) & 255;
        if(key == 27)
            gStop = true;
    }


    // std::cout<<gyr_arr.size()<<std::endl;
    // std::cout<<acc_arr.size()<<std::endl;


    {
        std::ofstream file("./dataset/imu0.csv");
        file << "timestamp"<<","<<"omega_x"<<","<<"omega_y"<<","<<"omega_z"<<","<<"alpha_x"<<","<<"alpha_y"<<","<<"alpha_z"<<"\n";

        int maxlen = std::min(gyr_arr.size(), acc_arr.size());
        for(int i=0;i<maxlen;i++)
        {
            assert(gyr_arr[i].timestamp == acc_arr[i].timestamp);
            file << gyr_arr[i].timestamp;
            file << ",";
            file << gyr_arr[i].x;
            file << ",";
            file << gyr_arr[i].y;
            file << ",";
            file << gyr_arr[i].z;
            file << ",";
            file << acc_arr[i].x;
            file << ",";
            file << acc_arr[i].y;
            file << ",";
            file << acc_arr[i].z;
            file << "\n";
        }
    }

    
    // th_show.join();

    // turn off the flow
    if(gyroSensor) {
        gyroSensor->stop();
    }
    if(accelSensor) {
        accelSensor->stop();
    }
    if(colorSensor)
    {
        colorSensor->stop();
    }

    return 0;
}



/*
611.57
611.426
634.867
399.981
1280
800
Distorion k1-k6 p1 p2
-0.841918
0.0869382
0.142698
-0.821658
0.0578776
0.154161
-0.000362444
0.000367282
######## second
 305.785
305.713
317.434
199.99
640
400
Distorion
-0.841918
0.0869382
0.142698
-0.821658
0.0578776
0.154161
-0.000362444
0.000367282



rosrun kalibr kalibr_calibrate_cameras --target aprilgrid.yaml --models pinhole-equi --topics /cam0/image_raw --bag v2.bag --bag-from-to 2 1000 --show-extraction
*/