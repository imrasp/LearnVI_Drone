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

#include <opencv2/opencv.hpp>
#include <mavlink/v1.0/common/mavlink.h>

using std::ofstream;
using namespace std;
using namespace cv;

class System_Log
{
public:
    // Con/De structors
    System_Log();
    System_Log(int ch);
    ~System_Log();

    void initialize_defaults();

    // write log to text file
    void write2txt(string text, mavlink_global_position_int_t global_position);
    void write2txt(string text, mavlink_local_position_ned_t local_position);
    void write2txt(string text, mavlink_gps_raw_int_t gps_raw);
    void write2txt();
    void write2txt(string text);

    // write log to csv file
    void write2csv(string text, mavlink_global_position_int_t global_position);
    void write2csv(string text, mavlink_local_position_ned_t local_position);
    void write2csv(string text, mavlink_highres_imu_t highres_imu);
    void write2csv(string text, mavlink_gps_raw_int_t gps_raw);
    void write2csv(string text, mavlink_raw_imu_t raw_imu);
    void write2csv(string text, Mat mat_pos);
    void write2csv(string text, Mat mat_pos, mavlink_local_position_ned_t local_position, float errX, float errY, float errZ, float errAvg);
    void write2csv();
    void write2csv(string text);



private:

    char datetime[40];
    char txtfilename[80];
    char csvfilename[80];
    ofstream csvlog;
    FILE* txtlog;
};


#endif //LEARNVI_DRONE_SYSTEM_LOG_H
