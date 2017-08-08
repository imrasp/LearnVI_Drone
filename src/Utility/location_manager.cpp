#include "Utility/location_manager.h"

Location_Manager::~Location_Manager() {}

Location_Manager::Location_Manager(System_Log *system_log_)
        : system_log(system_log_)//, mono_live_viorb(nullptr), mono_offline_viorb(nullptr)
{
    bStartSLAM = false;
}

Location_Manager::Location_Manager(System_Log *system_log_, Mono_Live_VIORB *mono_live_viorb_, Mono_Record_VIORB *mono_record_viorb_)
        : system_log(system_log_), mono_live_viorb(mono_live_viorb_), mono_record_viorb(mono_record_viorb_) {
    if(mono_live_viorb) bLiveMode = true;
    if(mono_record_viorb) bRecordMode = true;

    initializePosedata();
    bStartSLAM = false;
}

void Location_Manager::activateSLAM(){
    bStartSLAM = true;
}

void Location_Manager::initializePosedata(){
    current_pose.xacc = 0;
    current_pose.yacc = 0;
    current_pose.zacc = 0;

    current_pose.xacc = 0;
    current_pose.yacc = 0;
    current_pose.zacc = 0;

    current_pose.roll = 0;
    current_pose.pitch = 0;
    current_pose.yaw = 0;

    current_pose.lat = 0;
    current_pose.lon = 0;
    current_pose.alt = 0;

    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.z = 0;

    current_pose.satellites_visible = 0;
    current_pose.hdop = 0;
}

void Location_Manager::poseToSLAM(mavlink_highres_imu_t highres_imu) {
    current_pose.xacc = highres_imu.xacc;
    current_pose.yacc = highres_imu.yacc;
    current_pose.zacc = highres_imu.zacc;

    current_pose.xacc = highres_imu.xgyro;
    current_pose.yacc = highres_imu.ygyro;
    current_pose.zacc = highres_imu.zgyro;

    if(bStartSLAM)
    {
        if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData(current_pose);
    }

}

void Location_Manager::poseToSLAM(mavlink_attitude_t attitude) {
    current_pose.roll = attitude.roll;
    current_pose.pitch = attitude.pitch;
    current_pose.yaw = attitude.yaw;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData(current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_global_position_int_t global_pos) {
    current_pose.lat = global_pos.lat;
    current_pose.lon = global_pos.lon;
    current_pose.alt = global_pos.alt;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData(current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_local_position_ned_t local_pos) {
    current_pose.x = local_pos.x;
    current_pose.y = local_pos.y;
    current_pose.z = local_pos.z;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData(current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_gps_raw_int_t gps_raw) {
    current_pose.satellites_visible = gps_raw.satellites_visible;
    current_pose.hdop = gps_raw.eph;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData(current_pose);
    }
}


void Location_Manager::initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos) {
    init_local_position = local_pos;
    init_global_position = global_pos;
}

void Location_Manager::initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos) {
    init_local_position = local_pos;
    init_global_position.lat = global_pos.lat;
    init_global_position.lon = global_pos.lon;
    init_global_position.alt = global_pos.alt;
}

Mat Location_Manager::geodetic2NED(mavlink_gps_raw_int_t gps_pos) {
    mavlink_global_position_int_t convert_pos;
    convert_pos.lat = gps_pos.lat;
    convert_pos.lon = gps_pos.lon;
    convert_pos.alt = gps_pos.alt;

    return geodetic2NED(convert_pos);
}

Mat Location_Manager::geodetic2NED(mavlink_global_position_int_t gps_pos) {
    float latrel = gps_pos.lat - init_global_position.lat;
    float lonrel = gps_pos.lon - init_global_position.lon;
    float altrel = gps_pos.alt - init_global_position.alt;

    float x = 2 * EARTH_RADIUS * cos((init_global_position.lat + gps_pos.lat)
                                     / 2 / 1e+7 / 180 * M_PI) * sin(lonrel / 2 / 1e+7 / 180 * M_PI);
    float y = 2 * EARTH_RADIUS * sin(latrel / 2 / 1e+7 / 180 * M_PI);
    float z = altrel / 1e+3;

    Mat result = Mat_<float>(3, 1);
    result.at<float>(0) = x + init_local_position.x;
    result.at<float>(1) = y + init_local_position.y;
    result.at<float>(2) = z + init_local_position.z;
    return result;
}

float degrees2radians(float degrees) {
    return (degrees * M_PI) / 180;
}

float radians2degrees(float radians) {
    return (radians * 180) / M_PI;
}

