
#undef NDEBUG

#include <assert.h>   // reinclude the header to update the definition of assert()
#include "SLAMInterface/mono_live_viorb.h"
#include "Utility/location_manager.h"

Mono_Live_VIORB::Mono_Live_VIORB(System_Log *system_log_, SystemConfigParam *configParam_) : system_log(system_log_),
                                                                                             configParam(configParam_) {
    time_to_exit = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;
    firstTimestamp = 0;
}

Mono_Live_VIORB::~Mono_Live_VIORB() {}

void Mono_Live_VIORB::setLocationManager(Location_Manager *location_manager_) {
    location_manager = location_manager_;
}

void Mono_Live_VIORB::start() {
    cout << "Start Camera thread..." << endl;
    boost::thread threadCamera = boost::thread(&Mono_Live_VIORB::cameraLoop, this);

    cout << "Starting SLAM..." << endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(configParam->vocabulary, configParam->setting, ORB_SLAM2::System::MONOCULAR,
                                 string(configParam->gui) != "DISABLE");
    config = new ORB_SLAM2::ConfigParam(configParam->setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();


    cout << "Start SLAM thread..." << endl;
    boost::thread threadSLAM = boost::thread(&Mono_Live_VIORB::grabFrameData, this);

    if (configParam->bRecordSLAM) {
        cout << "Stard Record thread..." << endl;
        boost::thread threadRecord = boost::thread(&Mono_Live_VIORB::recordData, this);
    }
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

    int iSLAMFrame = 1;
    while (!time_to_exit) {

        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        matFrameForwardLast = matFrameForward.clone();
        matFrameForward = matFrame.clone();

        if (iSLAMFrame == 1) firstTimestamp = timestampc;
        timestampc = (timestampc - firstTimestamp) / 1000;

        frameDiff = 0;
        if (matFrameForward.rows <= 0 || matFrameForward.cols <= 0)
            continue;
        else
            frameDiff = frameDifference(matFrameForward, matFrameForwardLast);

        if (frameDiff == 0) {
            cout << "SKIPPING DUPLICATE FRAME" << endl;
            continue;
        } else {
            if (vimuData.size() < 10) {
                cout << "Skipping this frame (waiting for more IMUs)!" << endl;
                //cout << "Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }

            slam_last_pose = current_pose;

            if((gps_pose.timestampunix != nullptr && gps_pose.timestampunix - (std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1))) < 500){
                if (firstTimestamp == 0) firstTimestamp = gps_pose.timestampunix;
                double timestamp = (gps_pose.timestampunix - firstTimestamp) / 1000;
                ORB_SLAM2::GPSData gpsdata(gps_pose.lat, gps_pose.lon, gps_pose.alt, gps_pose.gpsx, gps_pose.gpsy, gps_pose.gpsz, timestamp);
                // Pass the image to the SLAM system
                vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, timestampc);
            }
            else{
                ORB_SLAM2::GPSData gpsdata(0, 0, 0, 0, 0, 0, 0);
                // Pass the image to the SLAM system
                vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, timestampc);
            }
            // Pass the image to the SLAM system
//            vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, timestampc);

            cout << iFrame << " :: " << iSLAMFrame << " :: vimuData size : " << vimuData.size() << endl;
            cout << "Tracking status : " << getTrackingStage() << endl;

            //update tracking stage to location manager
            if (getTrackingStage() == 2) {
                // 1st track
                if (latestTrackingStage == 1) {
                    location_manager->setInitialEstimateVisionPose(slam_last_pose);
                } else if (latestTrackingStage == 2) {
                    location_manager->setEstimatedVisionPose(vision_estimated_pose, slam_last_pose);
                }
                system_log->write2visionEstimatePositionLog(vision_estimated_pose);
            }

            latestTrackingStage = getTrackingStage();

            vimuData.clear();
            //while(!SLAM->bLocalMapAcceptKF()) {
            //}
        }
        calAvgProcessingTime(
                std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)); // stop
        iSLAMFrame++;
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

void Mono_Live_VIORB::cameraLoop() {

    stream1 = VideoCapture(configParam->camera1);
    cout << "open a Forward camera \n";
    if(configParam->camera2 > 0){
        stream2 = VideoCapture(configParam->camera2);
        cout << "open a Downward camera \n";
    }

    iFrame = 1;
    while (!time_to_exit) {
        timestampc = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        cout << "get frame " << iFrame << endl;
        stream1.read(matFrameForward);
        if(configParam->camera2 > 0){
            stream2.read(matFrameDownward);
        }
        iFrame++;
    }
}

void Mono_Live_VIORB::recordData() {
    //initialize Record folder
    boost::filesystem::path dir(configParam->record_path);
    boost::filesystem::path dir2(configParam->record_path + "/Camera1");
    boost::filesystem::path dir3(configParam->record_path + "/Camera2");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir)) {
            std::cout << "....Successfully Created Main Directory!" << std::endl;
        }
    }
    if (!(boost::filesystem::exists(dir2))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir2))
            std::cout << "....Successfully Created Forward Directory!" << std::endl;
    }
    if (!(boost::filesystem::exists(dir3))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir3))
            std::cout << "....Successfully Created Downward Directory!" << std::endl;
    }

    lframe.open(configParam->record_path + "/frame.csv");
    limugps.open(configParam->record_path + "/imugps.csv");

    limugps
            << string("FrameNo") + ","
               + "timestamp(ns)" + ","
               + "timeboot(ms)" + ","
               + "xgyro" + ","
               + "xyyro" + ","
               + "zgyro" + ","
               + "xacc" + ","
               + "yacc" + ","
               + "zacc" + ","
//               + "x" + ","
//               + "y" + ","
//               + "z" + ","
//               + "roll" + ","
//               + "pitch" + ","
//               + "yaw" + ","
//               //+ "satellites_visible" + ","
//               + "hdop" + ","
//               + "lat" + ","
//               + "lon" + ","
//               + "alt" + ","
//               + "vx" + ","
//               + "vy" + ","
//               + "vz" + ","
//               + "gpsxacc" + ","
//               + "gpsyacc" + ","
//               + "gpszacc" +
                       "\n";

    iRecordedFrame = 1;
    while (!time_to_exit) {
        imwrite(configParam->record_path + "/Camera1/" + to_string(iRecordedFrame) + ".jpg", matFrameForward);
        imwrite(configParam->record_path + "/Camera2/" + to_string(iRecordedFrame) + ".jpg", matFrameDownward);

        lframe << string("Frame,") + to_string(iRecordedFrame) + "," + to_string(timestampc) + "," + "\n";
        usleep(configParam->timespace); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec

        iRecordedFrame++;
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
void Mono_Live_VIORB::getGPSdata(posedata current_pose_){

    gps_pose = current_pose_;
}
void Mono_Live_VIORB::getIMUdata(posedata current_pose_) {
    current_pose = current_pose_;
    double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
    if (firstTimestamp == 0) firstTimestamp = timestamp;
    timestamp = (timestamp - firstTimestamp) / 1000;

    rollc = current_pose.xgyro;
    pitchc = current_pose.ygyro;
    yawc = current_pose.zgyro;

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

    string sep = ",";
    if (configParam->bRecordSLAM && iRecordedFrame == 1) {
        double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);

        limugps << iRecordedFrame << sep << timestamp << sep << current_pose.timebootms << sep << current_pose.xgyro
                << sep << current_pose.ygyro << sep << current_pose.zgyro << sep << current_pose.xacc << sep
                << current_pose.yacc << sep << current_pose.zacc << "\n";
//                  + to_string(timestamp) + ","
//                  + to_string(current_pose.timebootms) + ","
//                  + to_string(current_pose.xgyro) + ","
//                  + to_string(current_pose.ygyro) + ","
//                  + to_string(current_pose.zgyro) + ","
//                  + to_string(current_pose.xacc) + ","
//                  + to_string(current_pose.yacc) + ","
//                  + to_string(current_pose.zacc) + ","
//                  + to_string(current_pose.x) + ","
//                  + to_string(current_pose.y) + ","
//                  + to_string(current_pose.z) + ","
//                  + to_string(current_pose.roll) + ","
//                  + to_string(current_pose.pitch) + ","
//                  + to_string(current_pose.yaw) + ","
//                  //+ to_string(current_pose.satellites_visible) + ","
//                  + to_string(current_pose.hdop) + ","
//                  + to_string(current_pose.lat) + ","
//                  + to_string(current_pose.lon) + ","
//                  + to_string(current_pose.alt) + ","
//                  + to_string(current_pose.vx) + ","
//                  + to_string(current_pose.vy) + ","
//                  + to_string(current_pose.vz) + ","
//                  + to_string(current_pose.gpsxacc) + ","
//                  + to_string(current_pose.gpsyacc) + ","
//                  + to_string(current_pose.gpszacc)
//                  + "\n";
    }
}

int Mono_Live_VIORB::getTrackingStage() {
    return SLAM->getTrackingStage();
}