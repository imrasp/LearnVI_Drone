/*
 * location_manager.cpp
 *
 *  Created on: May 10, 2017
 *      Author: rasp
 */

#include "location_manager.h"

Location_Manager::Location_Manager()
{}
Location_Manager::~Location_Manager()
{}
Location_Manager::Location_Manager(System_Log *system_log_)
        : system_log(system_log_)
{}

void Location_Manager::initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos)
{
    init_local_position = local_pos;
    init_global_position = global_pos;
}

void Location_Manager::initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos)
{
    init_local_position = local_pos;
    init_global_position.lat = global_pos.lat;
    init_global_position.lon = global_pos.lon;
    init_global_position.alt = global_pos.alt;
}

Mat Location_Manager::geodetic2NED(mavlink_gps_raw_int_t gps_pos)
{
    mavlink_global_position_int_t convert_pos;
    convert_pos.lat = gps_pos.lat;
    convert_pos.lon = gps_pos.lon;
    convert_pos.alt = gps_pos.alt;

    return geodetic2NED(convert_pos);
}
Mat Location_Manager::geodetic2NED(mavlink_global_position_int_t gps_pos)
{
    float latrel = gps_pos.lat - init_global_position.lat;
    float lonrel = gps_pos.lon - init_global_position.lon;
    float altrel = gps_pos.alt - init_global_position.alt;

    float x = 2 * EARTH_RADIUS * cos((init_global_position.lat + gps_pos.lat)
                                     / 2 / 1e+7 / 180*M_PI ) * sin(lonrel / 2 / 1e+7 / 180 * M_PI);
    float y = 2 * EARTH_RADIUS * sin(latrel / 2 / 1e+7 / 180* M_PI);
    float z = altrel / 1e+3;

    Mat result = Mat_<float>(3,1);
    result.at<float>(0) = x + init_local_position.x;
    result.at<float>(1) = y + init_local_position.y;
    result.at<float>(2) = z + init_local_position.z;
    return result;
}

float degrees2radians(float degrees)
{
    return ( degrees * M_PI ) / 180;
}

float radians2degrees(float radians)
{
    return ( radians * 180 ) / M_PI;
}


