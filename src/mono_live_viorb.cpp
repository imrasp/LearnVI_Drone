//
// Created by rasp on 6/17/17.
//

#include "mono_live_viorb.h"

class time_point;

Mono_Live_VIORB::Mono_Live_VIORB(boost::mutex *pMu, Camera_Capture *cameraCapture_ , Mavlink_Control *mavlinkControl_)
{
    mu = pMu;
    cameraCapture = cameraCapture_;
    mavlinkControl = mavlinkControl_;
    time_to_exit = false;
    isFirstFrame = false;
    getFirstFrame = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;

}

Mono_Live_VIORB::~Mono_Live_VIORB()
{}

void Mono_Live_VIORB::start(char *&vocabulary, char *&setting) {
    cout << "Starting SLAM..." << endl;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(vocabulary, setting, ORB_SLAM2::System::MONOCULAR, true);
    config = new ORB_SLAM2::ConfigParam(setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
   // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();

    findCamera();

    boost::thread threadCamera = boost::thread(&Mono_Live_VIORB::cameraLoop, this);

    cout << "Start IMU thread..." << endl;
    boost::thread threadIMUdata = boost::thread(&Mono_Live_VIORB::getIMUdata, this);

    while (!getFirstFrame) isFirstFrame = true;
    cout << "Start Camera thread..." << endl;
    boost::thread threadSLAM = boost::thread(&Mono_Live_VIORB::grabFrameData, this);

}

void Mono_Live_VIORB::stop()
{
    time_to_exit = true;
    cout << "SLAM shutdown..." << endl;
    // Stop all threads
    SLAM->Shutdown();

    cout << "Save camera trajectory..." << endl;
    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cout << "Average Processing time per frame is " << avgTime << " milliseconds = " << avgTime / 1000 << " seconds" <<endl
            << "Max processing time : " << maxPTime << endl
            << "Min processing time : " << minPTime << endl;
}

void Mono_Live_VIORB::grabFrameData()
{
    cout << "grab Frame data for SLAM..." <<endl;

    int ni = 1;
    while(!time_to_exit)
    {
        //cout << "#frame " << ni << " time send data to slam : " << std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) << endl;
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        //mu->lock();
        // get image and position
        //cv::Mat im = cameraCapture->currentFrame;
        //timestampc = cameraCapture->tframe;
        //posdata = cameraCapture->posdata;
        matFrameForwardLast = matFrameForward.clone();

        //cout << "before get frame : " <<std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) <<endl;
        matFrameForward = matFrame;
        //cout << "after get frame : " <<std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) <<endl;
        //posdatac = posdata;
        //cout << "after get position data : " <<std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) <<endl;
        timestampc = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

        frameDiff = 0;
        if (isFirstFrame)
            isFirstFrame = false;
        else
            frameDiff = frameDifference(matFrameForward, matFrameForwardLast);
        //mu->unlock();

//        if (frameDiff == 0 )
//        {
//            //cout << "SKIPPING DUPLICATE FRAME" << endl;
//            continue;
//        }
//        else
//        {
            // Pass the forward image to the SLAM system
//            std::vector<ORB_SLAM2::IMUData> vimuData;
//
//            rollc = posdatac.roll;
//            pitchc = posdatac.pitch;
//            yawc = posdatac.yaw;

//            xc = posdatac.x;
//            yc = posdatac.y;
//            zc = posdatac.z;
            // x, y, z, roll, pitch, yaw, timestamp
            //ORB_SLAM2::IMUData imudata(xc, yc, zc, rollc, pitchc, yawc, timestampc);

            // multiply * ms2Tog to convert m/s2 to g and devided to 1000 converted it to mg
//            ax = posdatac.xacc * ms2Tog ;
//            ay = posdatac.yacc * ms2Tog ;
//            az = posdatac.zacc * ms2Tog ;
//            if(bAccMultiply98)
//            {
//                ax *= g3dm;
//                ay *= g3dm;
//                az *= g3dm;
//            }
            // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
//            ORB_SLAM2::IMUData imudata(rollc, pitchc, yawc, ax, ay, az, timestampc );

//            vimuData.push_back(imudata);

            if (vimuData.size() == 0) {
                //cout<<"Hit blank IMU slot ###############################" << endl;
                cout << "Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }
        cout << "imu data size = " << vimuData.size() <<endl;

            // Pass the image to the SLAM system
            //SLAM->SetFrameNumber(ni++);
            SLAM->TrackMonoVI(matFrameForward, vimuData, timestampc);
            vimuData.clear();
            //while(!SLAM->bLocalMapAcceptKF()) {
            //}

//        }

        //cout << "#frame " << ni-1 << " time before start new slam loop : " << std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1) << endl;
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
    }

}

double Mono_Live_VIORB::frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious) {
    double diff = 0.0;
    //cout << "matFrameCurrent size : " << matFrameCurrent.size() <<  " matFramePrevious : " << matFramePrevious.size() << endl;
    assert(matFrameCurrent.rows > 0 && matFrameCurrent.cols > 0);
    assert(
            matFrameCurrent.rows == matFramePrevious.rows
            && matFrameCurrent.cols == matFramePrevious.cols);
    assert(
            matFrameCurrent.type() == CV_8UC3 && matFramePrevious.type() == CV_8UC3);
    for (int i = 0; i < matFrameCurrent.rows; i++) {
        for (int j = 0; j < matFrameCurrent.cols; j++) {
            cv::Vec3b cur;
            cv::Vec3b prev;
            cur = matFrameCurrent.at<cv::Vec3b>(i, j);
            prev = matFramePrevious.at<cv::Vec3b>(i, j);
            for (int k = 0; k < 3; k++)
                diff += fabs(cur[k] - prev[k]);
        }
    }
    return diff;
}
void Mono_Live_VIORB::findCamera(){
    cout << "Starting camera connection..." << endl;
    // VideoCapture = 0 is the id of video device.0 if you have only one camera.

    int maxTested = 2;
    int i;
    for (i = maxTested; i >= 0; i--){
        VideoCapture stream(i);
        bool res = (stream.isOpened());
        cout << res <<endl;
        if (res)
        {
            cout <<"Open camera " << i <<endl;
            break;
        }
        else
        {
            stream.release();
            cout <<"Camera " << i << " is released" <<endl;
        }
    }

    if(i == -1)
    {
        cout << "cannot open camera";
        //return 0;
    }

    stream  = new VideoCapture(i);
}


void Mono_Live_VIORB::cameraLoop(){

    camFrame = 1;

    while(!time_to_exit) {
        stream->read(matFrame);
        //imshow("cam", matFrame);
        //waitKey(30);
        // time since epoch in milliseconds
        //timestampc = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        //posdata = mavlinkControl->getCurrentPose();
        //cout << "timestamp : " << timestampc << endl;
        //usleep(100000);

        if (camFrame == 1)
            getFirstFrame = true;
	//cout << "cameraLoop::matFrameForward : " << matFrameForward.size() << endl;
        camFrame++;
	//usleep(100000);
    }
}

void Mono_Live_VIORB::calAvgProcessingTime(double time) {
    if(startCalprocessingTime)
    {
        startTime = time;
        startCalprocessingTime = false;
        frameNo++;
    } else
    {
        double processingTime = time - startTime;
        avgTime = ((avgTime * (frameNo - 1)) + processingTime ) / frameNo;

        if(maxPTime < processingTime) maxPTime = processingTime;
        else if(minPTime < processingTime) minPTime = processingTime;

        startCalprocessingTime = true;
    }

}

void Mono_Live_VIORB::getIMUdata() {
    posdatalastest.roll = 0;
    cout << "start getting imu data ... " << endl;
    while (!time_to_exit) {
        posdatac = mavlinkControl->getCurrentPose();

        double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

        rollc = posdatac.roll;

        if(rollc == posdatalastest.roll) continue;
        pitchc = posdatac.pitch;
        yawc = posdatac.yaw;

//            xc = posdatac.x;
//            yc = posdatac.y;
//            zc = posdatac.z;
        // x, y, z, roll, pitch, yaw, timestamp
        //ORB_SLAM2::IMUData imudata(xc, yc, zc, rollc, pitchc, yawc, timestampc);

        // multiply * ms2Tog to convert m/s2 to g and devided to 1000 converted it to mg
        ax = posdatac.xacc * ms2Tog;
        ay = posdatac.yacc * ms2Tog;
        az = posdatac.zacc * ms2Tog;
        if (bAccMultiply98) {
            ax *= g3dm;
            ay *= g3dm;
            az *= g3dm;
        }
        // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
        ORB_SLAM2::IMUData imudata(rollc, pitchc, yawc, ax, ay, az, timestamp );

        vimuData.push_back(imudata);
        posdatac = posdatalastest;

        usleep(200);
    }
}

//int Mono_Live_VIORB::getTrackingStage(){
//    return SLAM->getTrackingStage();
//}

