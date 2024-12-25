
#include"imu_calibr.h"
#include <fstream>
#include <yaml-cpp/yaml.h>


IMU_Calibr::IMU_Calibr(
    int max_cluster,
    int max_time_min,
    std::string data_save_path
)
{
    _data_save_path = data_save_path;
    _max_time_min = max_time_min;
    gyr_x = std::make_shared<imu::AllanGyr>("gyr x", max_cluster);
    gyr_y = std::make_shared<imu::AllanGyr>("gyr y", max_cluster);
    gyr_z = std::make_shared<imu::AllanGyr>("gyr z", max_cluster);
    acc_x = std::make_shared<imu::AllanAcc>("acc x", max_cluster);
    acc_y = std::make_shared<imu::AllanAcc>("acc y", max_cluster);
    acc_z = std::make_shared<imu::AllanAcc>("acc z", max_cluster);
}

IMU_Calibr::~IMU_Calibr()
{

}

void IMU_Calibr::push(
    double gyr_x_data,
    double gyr_y_data,
    double gyr_z_data,
    double acc_x_data,
    double acc_y_data,
    double acc_z_data,
    std::chrono::system_clock::time_point time
)
{
    double time_d; // sec

    if(_start)
    {
        _start_t = time;
        _start = false;
        time_d = 0.0;
    }else{
        double elapsed = std::chrono::duration_cast<std::chrono::seconds>(time - _start_t).count();

        if(double(elapsed / 60) > _max_time_min)
            _end = true;
        
        time_d = elapsed;
    }

    gyr_x->pushRadPerSec( gyr_x_data, time_d );
    gyr_y->pushRadPerSec( gyr_y_data, time_d );
    gyr_z->pushRadPerSec( gyr_z_data, time_d );
    acc_x->pushMPerSec2( acc_x_data, time_d );
    acc_y->pushMPerSec2( acc_y_data, time_d );
    acc_z->pushMPerSec2( acc_z_data, time_d );

}




void
writeData3( 
            const std::string data_save_path,
            const std::string sensor_name,
            const std::vector< double >& gyro_ts_x,
            const std::vector< double >& gyro_d_x,
            const std::vector< double >& gyro_d_y,
            const std::vector< double >& gyro_d_z )
{
    std::ofstream out_t;
    std::ofstream out_x;
    std::ofstream out_y;
    std::ofstream out_z;
    out_t.open( data_save_path + "data_" + sensor_name + "_t.txt", std::ios::trunc );
    out_x.open( data_save_path + "data_" + sensor_name + "_x.txt", std::ios::trunc );
    out_y.open( data_save_path + "data_" + sensor_name + "_y.txt", std::ios::trunc );
    out_z.open( data_save_path + "data_" + sensor_name + "_z.txt", std::ios::trunc );
    out_t << std::setprecision( 10 );
    out_x << std::setprecision( 10 );
    out_y << std::setprecision( 10 );
    out_z << std::setprecision( 10 );

    for ( int index = 0; index < gyro_ts_x.size( ); ++index )
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d_x[index] << '\n';
        out_y << gyro_d_y[index] << '\n';
        out_z << gyro_d_z[index] << '\n';
    }

    out_t.close( );
    out_x.close( );
    out_y.close( );
    out_z.close( );
}




void
writeYAML( const std::string data_path,
           const std::string sensor_name,
           const imu::FitAllanGyr& gyr_x,
           const imu::FitAllanGyr& gyr_y,
           const imu::FitAllanGyr& gyr_z,
           const imu::FitAllanAcc& acc_x,
           const imu::FitAllanAcc& acc_y,
           const imu::FitAllanAcc& acc_z )
{
    std::ofstream fout(data_path + sensor_name + "_imu_param.yaml");
    YAML::Node config;

    config["type"] = "IMU";
    config["name"] = sensor_name;


    YAML::Node config_gyr;
    {
        YAML::Node temp;
        temp["gyr_n"] = ( gyr_x.getWhiteNoise( ) + gyr_y.getWhiteNoise( ) + gyr_z.getWhiteNoise( ) ) / 3;
        temp["gyr_w"] = ( gyr_x.getBiasInstability( ) + gyr_y.getBiasInstability( ) + gyr_z.getBiasInstability( ) ) / 3;
        config_gyr["avg-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["gyr_n"] = gyr_x.getWhiteNoise( );
        temp["gyr_w"] = gyr_x.getBiasInstability( );
        config_gyr["x-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["gyr_n"] = gyr_y.getWhiteNoise( );
        temp["gyr_w"] = gyr_y.getBiasInstability( );
        config_gyr["y-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["gyr_n"] = gyr_z.getWhiteNoise( );
        temp["gyr_w"] = gyr_z.getBiasInstability( );
        config_gyr["z-axis"] = temp;
    }
    config["Gyr (unit: rad/s)"] = config_gyr;


    YAML::Node config_acc;
    {
        YAML::Node temp;
        temp["acc_n"] = ( acc_x.getWhiteNoise( ) + acc_y.getWhiteNoise( ) + acc_z.getWhiteNoise( ) ) / 3;
        temp["acc_w"] = ( acc_x.getBiasInstability( ) + acc_y.getBiasInstability( ) + acc_z.getBiasInstability( ) ) / 3;
        config_acc["avg-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["acc_n"] = acc_x.getWhiteNoise( );
        temp["acc_w"] = acc_x.getBiasInstability( );
        config_acc["x-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["acc_n"] = acc_y.getWhiteNoise( );
        temp["acc_w"] = acc_y.getBiasInstability( );
        config_acc["y-axis"] = temp;
    }
    {
        YAML::Node temp;
        temp["acc_n"] = acc_z.getWhiteNoise( );
        temp["acc_w"] = acc_z.getBiasInstability( );
        config_acc["z-axis"] = temp;
    }
    config["Acc (unit: m/s^2)"] = config_acc;

    fout << config;
    std::cout << "YAML file written successfully!" << std::endl;
    fout.close();
}


void IMU_Calibr::show()
{
    gyr_x->calc( );
    std::vector< double > gyro_v_x  = gyr_x->getVariance( );
    std::vector< double > gyro_d_x  = gyr_x->getDeviation( );
    std::vector< double > gyro_ts_x = gyr_x->getTimes( );

    gyr_y->calc( );
    std::vector< double > gyro_v_y  = gyr_y->getVariance( );
    std::vector< double > gyro_d_y  = gyr_y->getDeviation( );
    std::vector< double > gyro_ts_y = gyr_y->getTimes( );

    gyr_z->calc( );
    std::vector< double > gyro_v_z  = gyr_z->getVariance( );
    std::vector< double > gyro_d_z  = gyr_z->getDeviation( );
    std::vector< double > gyro_ts_z = gyr_z->getTimes( );

    std::cout << "Gyro X " << std::endl;
    imu::FitAllanGyr fit_gyr_x( gyro_v_x, gyro_ts_x, gyr_x->getFreq( ) );
    std::cout << "  bias " << gyr_x->getAvgValue( ) / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro y " << std::endl;
    imu::FitAllanGyr fit_gyr_y( gyro_v_y, gyro_ts_y, gyr_y->getFreq( ) );
    std::cout << "  bias " << gyr_y->getAvgValue( ) / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro z " << std::endl;
    imu::FitAllanGyr fit_gyr_z( gyro_v_z, gyro_ts_z, gyr_z->getFreq( ) );
    std::cout << "  bias " << gyr_z->getAvgValue( ) / 3600 << " degree/s" << std::endl;
    std::cout << "-------------------" << std::endl;

    std::vector< double > gyro_sim_d_x = fit_gyr_x.calcSimDeviation( gyro_ts_x );
    std::vector< double > gyro_sim_d_y = fit_gyr_y.calcSimDeviation( gyro_ts_y );
    std::vector< double > gyro_sim_d_z = fit_gyr_z.calcSimDeviation( gyro_ts_z );

    writeData3(_data_save_path,  "sim_gyr", gyro_ts_x, gyro_sim_d_x, gyro_sim_d_y, gyro_sim_d_z );
    writeData3(_data_save_path,  "gyr", gyro_ts_x, gyro_d_x, gyro_d_y, gyro_d_z );


    std::cout << "==============================================" << std::endl;
    std::cout << "==============================================" << std::endl;

    acc_x->calc( );
    std::vector< double > acc_v_x  = acc_x->getVariance( );
    std::vector< double > acc_d_x  = acc_x->getDeviation( );
    std::vector< double > acc_ts_x = acc_x->getTimes( );

    acc_y->calc( );
    std::vector< double > acc_v_y  = acc_y->getVariance( );
    std::vector< double > acc_d_y  = acc_y->getDeviation( );
    std::vector< double > acc_ts_y = acc_y->getTimes( );

    acc_z->calc( );
    std::vector< double > acc_v_z  = acc_z->getVariance( );
    std::vector< double > acc_d_z  = acc_z->getDeviation( );
    std::vector< double > acc_ts_z = acc_z->getTimes( );

    std::cout << "acc X " << std::endl;
    imu::FitAllanAcc fit_acc_x( acc_v_x, acc_ts_x, acc_x->getFreq( ) );
    std::cout << "-------------------" << std::endl;

    std::cout << "acc y " << std::endl;
    imu::FitAllanAcc fit_acc_y( acc_v_y, acc_ts_y, acc_y->getFreq( ) );
    std::cout << "-------------------" << std::endl;

    std::cout << "acc z " << std::endl;
    imu::FitAllanAcc fit_acc_z( acc_v_z, acc_ts_z, acc_z->getFreq( ) );
    std::cout << "-------------------" << std::endl;

    std::vector< double > acc_sim_d_x = fit_acc_x.calcSimDeviation( acc_ts_x );
    std::vector< double > acc_sim_d_y = fit_acc_y.calcSimDeviation( acc_ts_x );
    std::vector< double > acc_sim_d_z = fit_acc_z.calcSimDeviation( acc_ts_x );

    writeData3( "sim_acc", "imu", acc_ts_x, acc_sim_d_x, acc_sim_d_y, acc_sim_d_z );
    writeData3( "acc", "imu", acc_ts_x, acc_d_x, acc_d_y, acc_d_z );

    writeYAML( _data_save_path, "imu", fit_gyr_x, fit_gyr_y, fit_gyr_z, fit_acc_x, fit_acc_y, fit_acc_z );

}