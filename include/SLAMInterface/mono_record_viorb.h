#ifndef LEARNVI_DRONE_MONO_RECORD_VIORB_H
#define LEARNVI_DRONE_MONO_RECORD_VIORB_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include "SLAMInterface/mono_live_viorb.h"
#include "Utility/system_log.h"

using namespace cv;
using namespace boost;
using namespace boost::posix_time;

class Mono_Record_VIORB {
public:
    Mono_Record_VIORB();
    ~Mono_Record_VIORB();
    Mono_Record_VIORB(System_Log *system_log_, bool gui, Mono_Live_VIORB *mono_live_viorb_, int timespace_);

    void start(char* filename);
    void stop();
    int findCamera();
    void loopCamera();
    void cameraRecorder();
    void getPoseData(string msg_name, posedata current_pose);


private:
    System_Log *system_log;
    bool bUseView;
    Mono_Live_VIORB *mono_live_viorb;

    boost::thread *threadCamera, *threadRecorder;
    int timespace;

    string foldername, imgname;
    ofstream imulog, tframelog;
    ofstream csvposelog;
    cv::Mat currentFrame;
    bool time_to_exit;
    int frameno;
    double tframe;
};
#endif //LEARNVI_DRONE_MONO_RECORD_VIORB_H
