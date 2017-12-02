//
// Created by rasp on 5/31/17.
//

#include "Utility/system_log.h"

System_Log::System_Log()
{
    //initialize_defaults();
}
System_Log::System_Log(string record_path)
{
    initialize_defaults(record_path);
}
System_Log::~System_Log()
{
    printf("File closed \n");
    txtlog.close();
    csvlog.close();
    gpslog.close();
    gpsorilog.close();
    gpsaccsample.close();
}

void System_Log::initialize_defaults(string record_path)
{
    boost::filesystem::path dir(record_path);
    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created from system_log file !" << std::endl;
    }

    printf("Initialize Log file (csv/txt) \n");
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    strftime(txtfilename,80,"%Y-%m-%d_%H:%M:%S.txt",now);
    strftime(csvfilename,80,"%Y-%m-%d_%H:%M:%S.csv",now);


    txtlog.open(record_path +"/"+ string(txtfilename));
    csvlog.open(record_path +"/"+ string(csvfilename));
    gpslog.open(record_path + "/gpsdata.csv");
    gpsorilog.open(record_path + "/gpsdata_original_coonversion.csv");
    gpsaccsample.open(record_path + "/gps_acc_sample_data.csv");
    visionEstimatePositionLog.open(record_path + "/visionEstimatePositionLog.csv");
    visionEstimate2IMULog.open(record_path + "/visionEstimate2IMULog.csv");

    gpslog << string("ned_time")  + "," + "x" + "," + "y" + "," + "z" + "," +
              "gps_time" + "," + "lat" + "," + "lon" + "," + "alt" + "," +
              "gpsx" + "," + "gpsy" + "," + "gpsz" + "," +
              "DRMS error" + "," + "99% SphericalAccuracyStandard error" + "," +
              "\n";
    gpsorilog << string("ned_time")  + "," + "x" + "," + "y" + "," + "z" + "," +
              "gps_time" + "," + "lat" + "," + "lon" + "," + "alt" + "," +
              "gpsx" + "," + "gpsy" + "," + "gpsz" + "," +
              "DRMS error" + "," + "99% SphericalAccuracyStandard error" + "," +
              "\n";
    gpsaccsample << string("ned_time") + "," + "x" + "," + "y" + "," + "z" + "," +
                    "vx" + "," + "vy" + "," + "vz" + "," +
                    "gps_time" + "," + "lat" + "," + "lon" + "," + "alt" + "," +
                    "gps_vx" + "," + "gps_vy" + "," + "gps_vz" + "," +
                    "highres_imu_time" + "," + "xacc" + "," + "yacc" + "," + "zacc" + "," +
                    "xgyro" + "," + "ygyro" + "," + "zgyro" + "," +
                    "\n";
    visionEstimatePositionLog << string("x") + "," + "y" + "," + "z" + "," + "scaled_x" + "," + "scaled_y" + "," + "scaled_z" + "\n";
}

// write log to text file
void System_Log::write2txt(string text, mavlink_global_position_int_t global_position)
{
//    fprintf(txtlog,"%s (int) : %lf, %lf, %lf \n", text.c_str(), (float)global_position.lat, (float)global_position.lon, (float)global_position.alt);
}
void System_Log::write2txt(string text, mavlink_local_position_ned_t local_position)
{
//    fprintf(txtlog,"%s (ned) : %lf, %lf, %lf \n", text.c_str(), local_position.x, local_position.y, local_position.z);
}
void System_Log::write2txt(string text, mavlink_gps_raw_int_t gps_raw)
{
//    fprintf(txtlog,"%s (GPS raw) : %d, %d \n", text.c_str(), gps_raw.satellites_visible, gps_raw.eph );
}
void System_Log::write2txt()
{  }
void System_Log::write2txt(string text)
{
//    fprintf(txtlog, "%s \n", text.c_str());
}
void System_Log::write2txt(string text, Mat matrix)
{
    txtlog << text << matrix << endl;
}

// write log to csv file
void System_Log::write2csv(string text, mavlink_global_position_int_t global_position)
{
    csvlog << text + string("(global_position_int)") + "," + to_string(global_position.time_boot_ms)+ "," + to_string(global_position.lat) + "," + to_string(global_position.lon) + "," + to_string(global_position.alt) +  "," + string("(VELOCITY)") + ","
              + to_string(global_position.vx) + "," + to_string(global_position.vy) + "," + to_string(global_position.vz)
              + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_local_position_ned_t local_position)
{
    csvlog << text + string("(local_position_ned)") + "," + to_string(local_position.time_boot_ms)+ "," + to_string(local_position.x) + "," + to_string(local_position.y) + "," + to_string(local_position.z) + "," + to_string(local_position.vx) + "," + to_string(local_position.vy) + "," + to_string(local_position.vz) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_highres_imu_t highres_imu)
{
    csvlog << text + string("(highres_imu)") + "," + to_string(highres_imu.time_usec) + "," + to_string(highres_imu.xacc) + "," + to_string(highres_imu.yacc) + "," + to_string(highres_imu.zacc) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_raw_imu_t raw_imu)
{
    csvlog << text + string("(raw_imu)") + "," + to_string(raw_imu.xacc) + "," + to_string(raw_imu.yacc) + "," + to_string(raw_imu.zacc) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_gps_raw_int_t gps_raw)
{
    csvlog << text + string("(gps_raw_int)") + "," + to_string(gps_raw.time_usec) + "," + to_string(gps_raw.satellites_visible) + "," + to_string(gps_raw.eph)
              + "," + to_string(gps_raw.lat)+ "," + to_string(gps_raw.lon)+ "," + to_string(gps_raw.alt) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_attitude_t attitude)
{
    csvlog << text + string("(Attitude)") + "," + to_string(attitude.time_boot_ms)+ "," + to_string(attitude.roll) + "," + to_string(attitude.pitch)
              + "," + to_string(attitude.yaw)+ "," + to_string(attitude.rollspeed)+ "," + to_string(attitude.pitchspeed)
              + "," + to_string(attitude.yawspeed) + "," + "\n";
}
void System_Log::write2csv(string text, Mat mat_pos)
{
    csvlog << text << string("(Matrix)") << ',' << format(mat_pos, "CSV") << '\n';
}
void System_Log::write2csv(string text, Mat mat_pos, mavlink_local_position_ned_t local_position, float errX, float errY, float errZ, float errAvg)
{
    csvlog << text + string("(mat_pos)") + "," + to_string(mat_pos.at<float>(0)) + "," + to_string(mat_pos.at<float>(1)) + "," + to_string(mat_pos.at<float>(2)) + ","
              + string("(ned)") + "," + to_string(local_position.x) + "," + to_string(local_position.y) + "," + to_string(local_position.z) + ","
              + string("(%Error)") + "," + to_string(errX) + "," + to_string(errY) + "," + to_string(errZ) + "," + to_string(errAvg) + "," + "\n";
}
void System_Log::write2csv(string text)
{
    csvlog << text + "\n";
}
void System_Log::write2gps(float nedtime, float x, float y, float z, float gpstime, float lat, float lon, float alt, float gpsx, float gpsy, float gpsz, float errDRMS, float err99Sph) {
    gpslog << to_string(nedtime)  + "," + to_string(x) + "," + to_string(y) + "," + to_string(z) + "," +
            to_string(gpstime) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(alt) + "," +
            to_string(gpsx) + "," + to_string(gpsy) + "," + to_string(gpsz) + "," +
            to_string(errDRMS) + "," + to_string(err99Sph) + "," +
              "\n";
}
void System_Log::write2origps(float nedtime, float x, float y, float z, float gpstime, float lat, float lon, float alt, float gpsx, float gpsy, float gpsz, float errDRMS, float err99Sph) {
    gpsorilog << to_string(nedtime)  + "," + to_string(x) + "," + to_string(y) + "," + to_string(z) + "," +
              to_string(gpstime) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(alt) + "," +
              to_string(gpsx) + "," + to_string(gpsy) + "," + to_string(gpsz) + "," +
              to_string(errDRMS) + "," + to_string(err99Sph) + "," +
              "\n";
}

void System_Log::write2gpsaccsample(float nedtime, float x, float y, float z, float vx, float vy, float vz, float gpstime, float lat, float lon, float alt, float gpsvx, float gpsvy, float gpsvz, float highres_imu_time, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float attitude_time, float roll, float pitch, float yaw){
    gpsaccsample << to_string(nedtime)  + "," + to_string(x) + "," + to_string(y) + "," + to_string(z) + "," +
                    to_string(vx) + "," + to_string(vy) + "," + to_string(vz) + "," +
                    to_string(gpstime) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(alt) + "," +
                    to_string(gpsvx) + "," + to_string(gpsvy) + "," + to_string(gpsvz) + "," +
                    to_string(highres_imu_time)  + "," + to_string(xacc) + "," + to_string(yacc) + "," + to_string(zacc) + "," +
                    to_string(xgyro) + "," + to_string(ygyro) + "," + to_string(zgyro) + "," +
                    to_string(attitude_time)  + "," + to_string(roll) + "," + to_string(pitch) + "," + to_string(yaw) + "," +
                    "\n";
}

void System_Log::write2visionEstimatePositionLog(Mat mat_pos)
{
//    Mat M = mat_pos.clone();
//    visionEstimatePositionLog << M.at<double>(0,0) << '\n';
    visionEstimatePositionLog << mat_pos.at<double>(0,3) << ',' << mat_pos.at<double>(1,3) << ',' << mat_pos.at<double>(2,3) << ',' <<  '\n';
}

void System_Log::write2visionEstimate2IMULog(double x, double y, double z, double xs, double ys, double zs)
{
    visionEstimate2IMULog << x << ',' << y << ',' << z << ',' << xs << ',' << ys << ',' << zs <<  '\n';
}


