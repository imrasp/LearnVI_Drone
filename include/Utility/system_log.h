//
// Created by rasp on 8/3/17.
//

#ifndef LEARNVI_DRONE_SYSTEM_LOG_H
#define LEARNVI_DRONE_SYSTEM_LOG_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <vector>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <mavlink/v1.0/common/mavlink.h>
#include <boost/filesystem.hpp>

using std::ofstream;
using namespace std;
using namespace cv;

class System_Log
{
public:
    // Con/De structors
    System_Log();
    System_Log(string record_path);
    ~System_Log();

    void initialize_defaults(string record_path);

    // write log to text file
    void write2txt(string text, mavlink_global_position_int_t global_position);
    void write2txt(string text, mavlink_local_position_ned_t local_position);
    void write2txt(string text, mavlink_gps_raw_int_t gps_raw);
    void write2txt();
    void write2txt(string text);
    void write2txt(string text, Mat matrix);

    // write log to csv file
    void write2csv(string text, mavlink_global_position_int_t global_position);
    void write2csv(string text, mavlink_local_position_ned_t local_position);
    void write2csv(string text, mavlink_highres_imu_t highres_imu);
    void write2csv(string text, mavlink_gps_raw_int_t gps_raw);
    void write2csv(string text, mavlink_raw_imu_t raw_imu);
    void write2csv(string text, mavlink_attitude_t attitude);
    void write2csv(string text, Mat mat_pos);
    void write2csv(string text, Mat mat_pos, mavlink_local_position_ned_t local_position, float errX, float errY, float errZ, float errAvg);
    void write2csv();
    void write2csv(string text);
    void write2gps(float nedtime, float x, float y, float z, float gpstime, float lat, float lon, float alt, float gpsx, float gpsy, float gpsz, float errDRMS, float err99Sph);
    void write2origps(float nedtime, float x, float y, float z, float gpstime, float lat, float lon, float alt, float gpsx, float gpsy, float gpsz, float errDRMS, float err99Sph);
    void write2gpsaccsample(float nedtime, float x, float y, float z, float vx, float vy, float vz, float gpstime, float lat, float lon, float alt, float gpsvx, float gpsvy, float gpsvz, float highres_imu_time, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float attitude_time, float roll, float pitch, float yaw);
    void write2visionEstimatePositionLog(Mat mat_pos);
    void write2visionEstimate2IMULog(double x, double y, double z, double xs, double ys, double zs);



private:

    char datetime[40];
    char txtfilename[80];
    char txtpath[160], csvpath[160];
    char csvfilename[80];
    ofstream csvlog, gpslog, txtlog, gpsorilog, gpsaccsample, visionEstimatePositionLog,visionEstimate2IMULog;
};


#endif //LEARNVI_DRONE_SYSTEM_LOG_H
