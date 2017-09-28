#ifndef LEARNVI_DRONE_LOCATION_MANAGER_H
#define LEARNVI_DRONE_LOCATION_MANAGER_H


#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <mavlink/v1.0/common/mavlink.h>

#include "Utility/system_log.h"
#include "SLAMInterface/mono_offline_viorb.h"
#include "SLAMInterface/mono_live_viorb.h"
#include "SLAMInterface/mono_record_viorb.h"

using namespace cv;
using namespace std;

#define EARTH_RADIUS 6378137

class Location_Manager {
public:
    ~Location_Manager();
    Location_Manager(System_Log *system_log_);
    Location_Manager(System_Log *system_log_, Mono_Live_VIORB *mono_live_viorb_, Mono_Record_VIORB *mono_record_viorb_);

    void initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos);
    void initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos);
    Mat geodetic2NED(mavlink_global_position_int_t gps_pos);
    Mat geodetic2NED(mavlink_gps_raw_int_t gps_pos);
    bool isInitialized();

    void poseToSLAM(mavlink_highres_imu_t highres_imu);
    void poseToSLAM(mavlink_global_position_int_t global_pos);
    void poseToSLAM(mavlink_attitude_t attitude);
    void poseToSLAM(mavlink_local_position_ned_t local_pos);
    void poseToSLAM(mavlink_gps_raw_int_t gps_raw);
    void activateSLAM();

    void getEstimatedVisionPose(Mat pose);

    posedata current_pose;
    posedata lastest_pose;
    Mat estimate_vision_pose, current_estimate_vision_pose;
    bool bisInitialized;
private:

    void initializePosedata();

    double init_nedx, init_nedy, init_nedz;
    mavlink_local_position_ned_t init_local_position;
    mavlink_global_position_int_t init_global_position;
    System_Log *system_log;
    Mono_Live_VIORB *mono_live_viorb;
    Mono_Record_VIORB *mono_record_viorb;
    bool bLiveMode, bRecordMode, bStartSLAM;
    float degrees2radians(float degrees);
    float radians2degrees(float radians);

};

#endif //LEARNVI_DRONE_LOCATION_MANAGER_H
