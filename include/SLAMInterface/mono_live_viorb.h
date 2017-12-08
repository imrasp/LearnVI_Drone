#ifndef LEARNVI_DRONE_MONO_LIVE_VIORB_H
#define LEARNVI_DRONE_MONO_LIVE_VIORB_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>

#include "Utility/system_log.h"
//#include "Utility/location_manager.h"
#include "VIORB/System.h"
#include "VIORB/IMU/imudata.h"
#include "VIORB/GPS/gpsdata.h"
#include "VIORB/IMU/configparam.h"
#include "Utility/systemConfigParam.h"


class Location_Manager;

typedef struct posedata {
    int timestampunix, timestampms, timestampns;
    float gpstime, nedtime, highres_imu_time, attitude_time;
    float x, y, z;
    double gpsx, gpsy, gpsz;
    float xacc, yacc, zacc;
    float xgyro, ygyro, zgyro;
    float roll, pitch, yaw;
    float vx,vy,vz;
    double gpsvx, gpsvy, gpsvz;
    float gpsxacc, gpsyacc, gpszacc;
    int timebootms;

    uint8_t satellites_visible;
    uint16_t hdop;
    int32_t lat, lon, alt;
}posedata;

class Mono_Live_VIORB {
public:
    Mono_Live_VIORB(System_Log *system_log_, SystemConfigParam *configParam_);
    ~Mono_Live_VIORB();

    void start();
    void stop();
    void grabFrameData();
    void cameraLoop();
    void recordData();
    void recordFrame();
    void getIMUdata(posedata current_pose_);
    void getGPSdata(posedata current_pose_);
    int getTrackingStage();
    void setLocationManager(Location_Manager *location_manager_);

    cv::Mat vision_estimated_pose, lastest_vision_estimated_pose, accumulate_vision_estimated_pose;
    cv::Mat matFrame, matFrameForward, matFrameForwardLast, matFrameDownward,matFrameCurrentForward;
    double xc,yc,zc;
    double rollc,pitchc,yawc;
    int timestampc, firstTimestamp,timestampcamera;
    double ax, ay, az;
    posedata current_pose, slam_last_pose, gps_pose, slam_gps_pose;

private:
    SystemConfigParam *configParam;
    System_Log *system_log;
    VideoCapture stream1, stream2;
    Location_Manager *location_manager;
    ofstream limu, lgps, lframe;

    bool time_to_exit;

    ORB_SLAM2::System *SLAM;
    ORB_SLAM2::ConfigParam *config;
    double imageMsgDelaySec;

    double frameDiff;
    int iFrame, imu_counter;

    double frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious);

    ORB_SLAM2::IMUData::vector_t vimuData;

    void calAvgProcessingTime(double time);
    bool startCalprocessingTime;
    double startTime;
    double avgTime;
    double maxPTime, minPTime;
    int frameNo, rFrame;
    int latestTrackingStage;
    int iRecordedFrame;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    bool bAccMultiply98;
    //1 meter/square second  =  0.101972 acceleration of gravity
    const double ms2Tog = 0.101972;

    string sep = ",";
};

#endif //LEARNVI_DRONE_MONO_LIVE_VIORB_H
