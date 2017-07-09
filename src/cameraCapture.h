//
// Created by rasp on 5/31/17.
//

#ifndef DRONE_PROJECT_CAMERACAPTURE_H
#define DRONE_PROJECT_CAMERACAPTURE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
//#include <thread>
//#include <mutex>

#include <common/mavlink.h>

#include "mavlink_control.h"

using namespace cv;
//using namespace std;
using namespace boost;
using namespace boost::posix_time;

//class Position {
//public:
//    float x;
//    float y;
//    float z;
//    float roll;
//    float pitch;
//    float yaw;
//    float timestamp_pos;
//};

class Camera_Capture {
public:
    Camera_Capture(boost::mutex *pMu, Mavlink_Control *mavlinkControl_);
    virtual ~Camera_Capture();

    void start();
    void stop();
    int findCamera();
    void loopCamera();
    void getPoseData();
    void getFrameData(Mat &cameraFrame_, positiondata posdata_);

    boost::mutex *mu;
    boost::thread *threadCamera;
    Mavlink_Control *mavlinkControl;
    bool time_to_exit;

    Mat currentFrame;
    positiondata posdata;
    double tframe;
    int frameno;

};

#endif //DRONE_PROJECT_CAMERACAPTURE_H
