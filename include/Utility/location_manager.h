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

class Autopilot_Interface;
class Mavlink_Control;

using namespace cv;
using namespace std;

#define EARTH_RADIUS 6378137

class Location_Manager {
public:
    ~Location_Manager();
    Location_Manager(System_Log *system_log_);
    Location_Manager(System_Log *system_log_, Mono_Live_VIORB *mono_live_viorb_);
    void setMavlinkControl(Mavlink_Control *mavlink_control_);

    void initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos);
    void initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos);
    Mat geodetic2NED(mavlink_global_position_int_t gps_pos);
    Mat geodetic2NED(mavlink_gps_raw_int_t gps_pos);
    bool isInitialized();

    void setPose(mavlink_highres_imu_t highres_imu);
    void setPose(mavlink_global_position_int_t global_pos);
    void setPose(mavlink_attitude_t attitude);
    void setPose(mavlink_local_position_ned_t local_pos);
    void setPose(mavlink_gps_raw_int_t gps_raw);
    void activateSLAM();

    void setInitialEstimateVisionPose(posedata pose);
    void setEstimatedVisionPose(Mat pose,posedata apose);
    void setUpdateVisionPoseToMavlink(bool update);
    bool getUpdateVisionPoseToMavlink();
    void setUpdateGPSPoseToMavlink(bool update);
    bool getUpdateGPSPoseToMavlink();

    posedata current_pose, lastest_pose, pEstimatedVisionPose;
    Mat estimate_vision_pose, current_estimate_vision_pose;
    bool bisInitialized, bNotFirstEstimatedPose;

    void setSLAMTrackingStage(int stage);
    int getSALMTrackingStage();
//    eTrackingState
//    SYSTEM_NOT_READY=-1,
//    NO_IMAGES_YET=0,
//    NOT_INITIALIZED=1,
//    OK=2,
//    LOST=3
    int SLAMTrackingStage;

private:

    void initializePosedata();

    double init_nedx, init_nedy, init_nedz;
    mavlink_local_position_ned_t init_local_position;
    mavlink_global_position_int_t init_global_position;
    System_Log *system_log;
    Mavlink_Control *mavlink_control;
    Mono_Live_VIORB *mono_live_viorb;
    bool bStartSLAM, bUpdateGPSPoseToMavlink, bUpdateVisionPoseToMavlink;
    float degrees2radians(float degrees);
    float radians2degrees(float radians);
    void getRotationTranslation(Mat mtransformation, float *roll, float *pitch, float *yaw);

    int counter;
double dScaleX, dScaleY,dScaleZ;
    //Transformation from camera to ned
    Eigen::Matrix4d Tnb, Tbc;

};

#endif //LEARNVI_DRONE_LOCATION_MANAGER_H
