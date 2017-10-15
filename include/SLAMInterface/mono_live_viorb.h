#ifndef LEARNVI_DRONE_MONO_LIVE_VIORB_H
#define LEARNVI_DRONE_MONO_LIVE_VIORB_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>

#include "Utility/system_log.h"
//#include "Utility/location_manager.h"
#include "VIORB/System.h"
#include "VIORB/IMU/imudata.h"
#include "VIORB/IMU/configparam.h"

class Location_Manager;

typedef struct posedata {
    float timestampms, timestampns;
    float gpstime, nedtime, highres_imu_time, attitude_time;
    float x, y, z;
    double gpsx, gpsy, gpsz;
    float xacc, yacc, zacc;
    float xgyro, ygyro, zgyro;
    float roll, pitch, yaw;
    float vx,vy,vz;
    double gpsvx, gpsvy, gpsvz;
    float gpsxacc, gpsyacc, gpszacc;
    float timebootms;

    uint8_t satellites_visible;
    uint16_t hdop;
    int32_t lat, lon, alt;
}posedata;

class Mono_Live_VIORB {
public:
    Mono_Live_VIORB(System_Log *system_log_, bool bUseView);
    ~Mono_Live_VIORB();

    void start(char *&vocabulary, char *&setting);
    void stop();
    void grabFrameData();
    void findCamera();
    void cameraLoop();
    void getIMUdata(posedata current_pose_);
    int getTrackingStage();
    void setLocationManager(Location_Manager *location_manager_);

    cv::Mat vision_estimated_pose, lastest_vision_estimated_pose, accumulate_vision_estimated_pose;
    cv::Mat matFrame, matFrameForward, matFrameForwardLast;
    double xc,yc,zc;
    double rollc,pitchc,yawc;
    double timestampc, firstTimestamp;
    double ax, ay, az;
    posedata current_pose, initial_slam_pose;

private:

    System_Log *system_log;
    VideoCapture *stream;
    Location_Manager *location_manager;

    bool bUseView;

    bool time_to_exit;

    vector<float> vTimesTrack;
    double ttrack;

    ORB_SLAM2::System *SLAM;
    ORB_SLAM2::ConfigParam *config;
    double imageMsgDelaySec;

    double frameDiff;
    bool getFirstFrame, isFirstFrame, first_estimate_vision_pose;
    int camFrame;

    double frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious);

    std::vector<ORB_SLAM2::IMUData> vimuData;

    void calAvgProcessingTime(double time);
    bool startCalprocessingTime;
    double startTime;
    double avgTime;
    double maxPTime, minPTime;
    int frameNo;
    int trackingStage;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    bool bAccMultiply98;
    //1 meter/square second  =  0.101972 acceleration of gravity
    const double ms2Tog = 0.101972;
};

#endif //LEARNVI_DRONE_MONO_LIVE_VIORB_H
