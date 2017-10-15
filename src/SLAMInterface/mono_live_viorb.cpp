
#undef NDEBUG

#include <assert.h>   // reinclude the header to update the definition of assert()
#include "SLAMInterface/mono_live_viorb.h"
#include "Utility/location_manager.h"

Mono_Live_VIORB::Mono_Live_VIORB(System_Log *system_log_, bool bUseView_) : system_log(system_log_),
                                                                            bUseView(bUseView_) {
    time_to_exit = false;
    isFirstFrame = false;
    getFirstFrame = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;
    firstTimestamp = 0;
    first_estimate_vision_pose = false;
}

Mono_Live_VIORB::~Mono_Live_VIORB() {}

void Mono_Live_VIORB::setLocationManager(Location_Manager *location_manager_) {
    location_manager = location_manager_;
}

void Mono_Live_VIORB::start(char *&vocabulary, char *&setting) {
    cout << "Starting SLAM..." << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(vocabulary, setting, ORB_SLAM2::System::MONOCULAR, bUseView);
    config = new ORB_SLAM2::ConfigParam(setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();

    findCamera();
    cout << "Start Camera thread..." << endl;
    boost::thread threadCamera = boost::thread(&Mono_Live_VIORB::cameraLoop, this);

    cout << "Start SLAM thread..." << endl;
    boost::thread threadSLAM = boost::thread(&Mono_Live_VIORB::grabFrameData, this);

}

void Mono_Live_VIORB::stop() {
    time_to_exit = true;
    cout << "SLAM shutdown..." << endl;
    // Stop all threads
    SLAM->Shutdown();

    cout << "Save camera trajectory..." << endl;
    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cout << "Average Processing time per frame is " << avgTime << " milliseconds = " << avgTime / 1000 << " seconds"
         << endl
         << "Max processing time : " << maxPTime << endl
         << "Min processing time : " << minPTime << endl;
}

void Mono_Live_VIORB::grabFrameData() {
    cout << "grab Frame data for SLAM..." << endl;

    int ni = 1;

    while (!time_to_exit) {

        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        matFrameForwardLast = matFrameForward.clone();
        matFrameForward = matFrame.clone();

        timestampc = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
        if (firstTimestamp == 0) firstTimestamp = timestampc;
        timestampc = (timestampc - firstTimestamp) / 1000;

        frameDiff = 0;

        if (!getFirstFrame)
            continue;
        else
            frameDiff = frameDifference(matFrameForward, matFrameForwardLast);

        if (frameDiff == 0) {
            //cout << "SKIPPING DUPLICATE FRAME" << endl;
            continue;
        } else {


            if (vimuData.size() < 10) {
                //cout << "Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }
            if(!first_estimate_vision_pose) initial_slam_pose = current_pose;

            // Pass the image to the SLAM system
            //SLAM->TrackMonoVI(matFrameForward, vimuData, timestampc);
            cout << camFrame << " :: " << ni << " :: vimuData size : " << vimuData.size() << endl;
            vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, timestampc);

            //update tracking stage to location manager
            cout << "Tracking status : " << getTrackingStage() << endl;
            system_log->write2txt("Tracking status : " + to_string(getTrackingStage()));
            if(trackingStage != getTrackingStage()) {
                location_manager->setSLAMTrackingStage(getTrackingStage());
                trackingStage = getTrackingStage();
            }

            //set initial pose of vision estimation pose
            if(vision_estimated_pose.size().height > 1) {
                if(!first_estimate_vision_pose){
                    location_manager->setInitialEstimateVisionPose(initial_slam_pose);
                    first_estimate_vision_pose = true;
                }
                else{
                    // cout << " vision_estimated_pose = " << vision_estimated_pose << endl;
                    location_manager->setEstimatedVisionPose(vision_estimated_pose);
                    system_log->write2txt("Estimated_Position (SLAM Frame " + to_string(ni) + "(" + to_string(camFrame) + ")) ", vision_estimated_pose);
                    //system_log->write2csv("Estimated_Position (SLAM Frame " + to_string(ni) + "(" + to_string(camFrame) + ")) ", vision_estimated_pose);
                    cout << "call write2visionEstimatePositionLog" << endl;
                    system_log->write2visionEstimatePositionLog(vision_estimated_pose);
                }

            }


//            if(!lastest_vision_estimated_pose.empty()){
//                accumulate_vision_estimated_pose = vision_estimated_pose * lastest_vision_estimated_pose;
//                lastest_vision_estimated_pose = accumulate_vision_estimated_pose.clone();
//            }
//            else lastest_vision_estimated_pose = vision_estimated_pose;
//
//            system_log->write2txt("Accumulate Estimated_Position (SLAM) ",lastest_vision_estimated_pose);

            vimuData.clear();
            //while(!SLAM->bLocalMapAcceptKF()) {
            //}
        }
        calAvgProcessingTime(
                std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)); // stop
        ni++;
    }

}

double Mono_Live_VIORB::frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious) {
    double diff = 0.0;
    // cout << "matFrameCurrent size : " << matFrameCurrent.size() <<  " matFramePrevious : " << matFramePrevious.size() << endl;
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

void Mono_Live_VIORB::findCamera() {
    cout << "Starting camera connection..." << endl;
    // VideoCapture = 0 is the id of video device.0 if you have only one camera.

    int maxTested = 2;
    int i;
    for (i = maxTested; i >= 0; i--) {
        VideoCapture stream(i);
        bool res = (stream.isOpened());
        cout << res << endl;
        if (res) {
            cout << "Open camera " << i << endl;
            break;
        } else {
            stream.release();
            cout << "Camera " << i << " is released" << endl;
        }
    }

    if (i == -1) {
        cout << "cannot open camera";
        //return 0;
    }
    stream = new VideoCapture(i);
}


void Mono_Live_VIORB::cameraLoop() {

    cout << "starting camera (Mono_Live_VIORB)" << endl;
    camFrame = 1;

    while (!time_to_exit) {
        stream->read(matFrame);
        //cout << "matFrame size : " << matFrame.size();
//        imshow( "Display window", matFrame );
//        if (waitKey(30) >= 0)
//            break;
        if (camFrame == 2) {
            getFirstFrame = true;
            isFirstFrame = true;
        }

        camFrame++;

    }
}

void Mono_Live_VIORB::calAvgProcessingTime(double time) {
    if (startCalprocessingTime) {
        startTime = time;
        startCalprocessingTime = false;
        frameNo++;
    } else {
        double processingTime = time - startTime;
        avgTime = ((avgTime * (frameNo - 1)) + processingTime) / frameNo;

        if (maxPTime < processingTime) maxPTime = processingTime;
        else if (minPTime < processingTime) minPTime = processingTime;

        startCalprocessingTime = true;
    }

}

void Mono_Live_VIORB::getIMUdata(posedata current_pose_) {
    current_pose = current_pose_;
    double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
    if (firstTimestamp == 0) firstTimestamp = timestamp;
    timestamp = (timestamp - firstTimestamp) / 1000;

    rollc = current_pose.xgyro;
    pitchc = current_pose.ygyro;
    yawc = current_pose.zgyro;

    //cout << "roll pitch yaw : " << rollc << ", " << pitchc << ", " << yawc << endl;
    ax = current_pose.xacc;
    ay = current_pose.yacc;
    az = current_pose.zacc;

    if (bAccMultiply98) {
        ax *= g3dm;
        ay *= g3dm;
        az *= g3dm;
    }
    // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
    ORB_SLAM2::IMUData imudata(rollc, pitchc, yawc, ax, ay, az, timestamp);
    vimuData.push_back(imudata);
}

int Mono_Live_VIORB::getTrackingStage() {
    return SLAM->getTrackingStage();
}
