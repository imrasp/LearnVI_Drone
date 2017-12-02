//#ifndef LEARNVI_DRONE_MONO_OFFLINE_VIORB_H
//#define LEARNVI_DRONE_MONO_OFFLINE_VIORB_H
//
//#include<iostream>
//#include<algorithm>
//#include<fstream>
//#include<chrono>
//#include <string>
//#include <boost/thread.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <boost/filesystem.hpp>
//#include<opencv2/core/core.hpp>
//
//#include "VIORB/System.h"
//#include "VIORB/IMU/imudata.h"
//#include "VIORB/IMU/configparam.h"
//#include "VIORB/GPS/gpsdata.h"
//#include "SLAMInterface/mono_live_viorb.h"
//#include "Utility/system_log.h"
//#include "Utility/systemConfigParam.h"
//
//class Mono_Offline_VIORB {
//public:
//    Mono_Offline_VIORB(System_Log *system_log_, SystemConfigParam *configParam_ );
//    ~Mono_Offline_VIORB();
//
//    void start();
//    void stop();
//    void grabFrameData();
//    void findCamera();
//    void cameraLoop();
//
//    int getTrackingStage();
//
//    cv::Mat matFrame, matFrameForward, matFrameForwardLast;
//    double xc,yc,zc;
//    double rollc,pitchc,yawc;
//    double timestampc,timestamp, firstTimestamp;
//    double ax, ay, az;
//
//private:
//    System_Log *system_log;
//    SystemConfigParam *configParam;
//    VideoCapture *stream;
//
//    bool time_to_exit;
//
//    vector<float> vTimesTrack;
//    double ttrack;
//
//    ORB_SLAM2::System *SLAM;
//    ORB_SLAM2::ConfigParam *config;
//    double imageMsgDelaySec;
//
//    double frameDiff;
//    bool getFirstFrame;
//    bool isFirstFrame;
//    int camFrame;
//
//    double frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious);
//
//    void getIMUdata();
//    ORB_SLAM2::IMUData::vector_t vimuData;
//    void resetvIMUdata();
//
//    void calAvgProcessingTime(double time);
//    bool startCalprocessingTime;
//    double startTime;
//    double avgTime;
//    double maxPTime, minPTime;
//    int frameno, frameNo;
//
//    // 3dm imu output per g. 1g=9.80665 according to datasheet
//    const double g3dm = 9.80665;
//    bool bAccMultiply98;
//    //1 meter/square second  =  0.101972 acceleration of gravity
//    const double ms2Tog = 0.101972;
//
//    string foldername, filename;
//    ifstream imulog, tframelog;
//};
//#endif //LEARNVI_DRONE_MONO_OFFLINE_VIORB_H
