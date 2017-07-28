//
// Created by rasp on 7/16/17.
//

#ifndef LEARNVI_DRONE_MONO_OFFLINE_VIORB_H
#define LEARNVI_DRONE_MONO_OFFLINE_VIORB_H


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <string>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include<opencv2/core/core.hpp>

#include<System.h>

#include "IMU/imudata.h"
#include "IMU/configparam.h"
#include "cameraCapture.h"
#include "mavlink_control.h"

class Mono_Offline_VIORB {
public:
    Mono_Offline_VIORB(boost::mutex *pMu);
    ~Mono_Offline_VIORB();

    void start(char *&vocabulary, char *&setting);
    void stop();
    void grabFrameData();
    void findCamera();
    void cameraLoop();

    int getTrackingStage();

    cv::Mat matFrame, matFrameForward, matFrameForwardLast;
    double xc,yc,zc;
    double rollc,pitchc,yawc;
    double timestampc,timestamp, firstTimestamp;
    double ax, ay, az;

private:
    boost::mutex *mu;
    Camera_Capture *cameraCapture;
    Mavlink_Control *mavlinkControl;
    VideoCapture *stream;

    bool time_to_exit;

    vector<float> vTimesTrack;
    double ttrack;
    positiondata posdata;
    positiondata posdatac;
    positiondata posdatalastest;

    ORB_SLAM2::System *SLAM;
    ORB_SLAM2::ConfigParam *config;
    double imageMsgDelaySec;

    double frameDiff;
    bool getFirstFrame;
    bool isFirstFrame;
    int camFrame;

    double frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious);

    void getIMUdata();
    std::vector<ORB_SLAM2::IMUData> vimuData;
    void resetvIMUdata();

    void calAvgProcessingTime(double time);
    bool startCalprocessingTime;
    double startTime;
    double avgTime;
    double maxPTime, minPTime;
    int frameno, frameNo;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    bool bAccMultiply98;
    //1 meter/square second  =  0.101972 acceleration of gravity
    const double ms2Tog = 0.101972;

    string foldername, filename;
    ifstream imulog, tframelog;
};

#endif //LEARNVI_DRONE_MONO_OFFLINE_VIORB_H
