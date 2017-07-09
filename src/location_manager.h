//
// Created by rasp on 5/31/17.
//

#ifndef DRONE_PROJECT_LOCATION_MANAGER_H
#define DRONE_PROJECT_LOCATION_MANAGER_H

#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <common/mavlink.h>

#include "system_log.h"

using namespace cv;
using namespace std;

#define EARTH_RADIUS 6378137

class Location_Manager
{
public:

    Location_Manager();
    ~Location_Manager();
    Location_Manager(System_Log *system_log_);

    void initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos);
    void initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos);
    Mat geodetic2NED(mavlink_global_position_int_t gps_pos);
    Mat geodetic2NED(mavlink_gps_raw_int_t gps_pos);




private:

    mavlink_local_position_ned_t init_local_position;
    mavlink_global_position_int_t init_global_position;

    System_Log *system_log;

    float degrees2radians(float degrees);
    float radians2degrees(float radians);

};

#endif //DRONE_PROJECT_LOCATION_MANAGER_H
