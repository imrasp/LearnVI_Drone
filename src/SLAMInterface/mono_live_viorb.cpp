
#undef NDEBUG

#include <assert.h>   // reinclude the header to update the definition of assert()
#include "SLAMInterface/mono_live_viorb.h"
#include "Utility/location_manager.h"

Mono_Live_VIORB::Mono_Live_VIORB(System_Log *system_log_, SystemConfigParam *configParam_) : system_log(system_log_), configParam(configParam_) {
    time_to_exit = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;
    firstTimestamp = 0;
    gps_pose.timestampunix_ns = 0;
}

Mono_Live_VIORB::~Mono_Live_VIORB() {}

void Mono_Live_VIORB::setLocationManager(Location_Manager *location_manager_) {
    location_manager = location_manager_;
}

void Mono_Live_VIORB::initializeCamera(){

    stream1 = VideoCapture(configParam->camera1);
    cout << "open a Forward camera \n";
    if(configParam->camera2 > 0){
        stream2 = VideoCapture(configParam->camera2);
        cout << "open a Downward camera \n";
    }
//    query_maximum_resolution(&stream1, max_width, max_height);
//    max_width = 1280; max_height = 720;
//    max_width = 640; max_height = 480;
    max_width = 848; max_height = 480;

    //initialize Record folder
    boost::filesystem::path dir(configParam->record_path);
    boost::filesystem::path dir4(configParam->record_path + "/dataset-dir");
    boost::filesystem::path dir5(configParam->record_path + "/dataset-dir/cam0");
    boost::filesystem::path dir6(configParam->record_path + "/dataset-dir/cam1");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir)) {
            std::cout << "....Successfully Created Main Directory!" << std::endl;
        }
    }
    if (!(boost::filesystem::exists(dir4))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir4))
            std::cout << "....Successfully Created dataset-dir Directory!" << std::endl;
    }
    if (!(boost::filesystem::exists(dir5))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir5))
            std::cout << "....Successfully Created /dataset-dir/cam0 Directory!" << std::endl;
    }
    if (!(boost::filesystem::exists(dir6))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir6))
            std::cout << "....Successfully Created /dataset-dir/cam1 Directory!" << std::endl;
    }

    lframe.open(configParam->record_path + "/frame.csv");
    lgps.open(configParam->record_path + "/gps.csv");
    ldatasetimu.open(configParam->record_path + "/dataset-dir/imu0.csv");

    lframe << "timestamp" << "\n";
    ldatasetimu << "timestamp" << ","  << "omega_x" << "," << "omega_y" << "," << "omega_z" << "," << "alpha_x" << "," << "alpha_y" << "," << "alpha_z" << "\n";
    lgps << "timestamp" << "," << "lat" << "," << "lon" << "," << "alt" << "\n";
}

void Mono_Live_VIORB::start() {

    initializeCamera();

    if(configParam->bCamera) {
        cout << "Start Camera thread..." << endl;
        threadCamera = boost::thread(&Mono_Live_VIORB::cameraLoop, this);
    }

    if (configParam->bRecord) {
        cout << "Stard Record thread..." << endl;
        threadRecord = boost::thread(&Mono_Live_VIORB::recordData, this);
    }

    if (configParam->bLive) {
        cout << "Starting SLAM..." << endl;
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        SLAM = new ORB_SLAM2::System(configParam->vocabulary, configParam->setting, ORB_SLAM2::System::MONOCULAR, string(configParam->gui) != "DISABLE");
        config = new ORB_SLAM2::ConfigParam(configParam->setting);

        imageMsgDelaySec = config->GetImageDelayToIMU();
        // ORBVIO::MsgSynchronizer msgsync(imageMsgDe laySec);
        bAccMultiply98 = config->GetAccMultiply9p8();


        cout << "Start SLAM thread..." << endl;
        boost::thread threadSLAM = boost::thread(&Mono_Live_VIORB::grabFrameData, this);
    }
}

void Mono_Live_VIORB::stop() {
    cout << "SLAM shutdown..." << endl;

    if (configParam->bLive) {
        // Stop all threads
        SLAM->Shutdown();

        cout << "Save camera trajectory..." << endl;
        // Save camera trajectory
        SLAM->SaveKeyFrameTrajectoryTUM(configParam->record_path + "/KeyFrameTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryNavState(configParam->record_path + "/KeyFrameNavStateTrajectory.txt");

        cout << "Average Processing time per frame is " << avgTime << " milliseconds = " << avgTime / 1000 << " seconds"
             << endl
             << "Max processing time : " << maxPTime << endl
             << "Min processing time : " << minPTime << endl;
    }

    time_to_exit = true;

    threadRecord.join();
    threadCamera.join();

    lframe.close();
    lgps.close();
    ldatasetimu.close();
}

void Mono_Live_VIORB::grabFrameData() {
    cout << "grab Frame data for SLAM..." << endl;

    int iSLAMFrame = 1;
    while (!time_to_exit) {

        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        matFrameCurrentForward.copyTo(matFrameForwardLast);
        matFrameForward.copyTo(matFrameCurrentForward);

        frameDiff = 0;
        if (matFrameForwardLast.rows <= 0 || matFrameForwardLast.cols <= 0)
            continue;
        else
            frameDiff = frameDifference(matFrameForward, matFrameForwardLast);

        if (frameDiff == 0) {
//            cout << "SKIPPING DUPLICATE FRAME" << endl;
            continue;
        } else {
            if (vimuData.size() < 10) {
                //cout << "Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }

            slam_last_pose = current_pose;

            ORB_SLAM2::GPSData gpsdata(0, 0, 0, 0, 0, 0, 0);
            // Pass the image to the SLAM system
            vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, timestampc/1000);

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
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)); // stop
        iSLAMFrame++;
    }

}

//double Mono_Live_VIORB::frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious) {
//    double diff = 0.0;
////     cout << "matFrameCurrent size : " << matFrameCurrent.size() <<  " matFramePrevious : " << matFramePrevious.size() << endl;
//    assert(matFrameCurrent.rows > 0 && matFrameCurrent.cols > 0);
//    assert(
//            matFrameCurrent.rows == matFramePrevious.rows
//            && matFrameCurrent.cols == matFramePrevious.cols);
//    assert(
//            matFrameCurrent.type() == CV_8UC3 && matFramePrevious.type() == CV_8UC3);
//    for (int i = 0; i < matFrameCurrent.rows; i++) {
//        for (int j = 0; j < matFrameCurrent.cols; j++) {
//            cv::Vec3b cur;
//            cv::Vec3b prev;
//            cur = matFrameCurrent.at<cv::Vec3b>(i, j);
//            prev = matFramePrevious.at<cv::Vec3b>(i, j);
//            for (int k = 0; k < 3; k++)
//                diff += fabs(cur[k] - prev[k]);
//        }
//    }
//    return diff;
//}

// check is 2 frames is difference or not
double Mono_Live_VIORB::frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious) {
    double diff = 0.0;
    assert(matFrameCurrent.rows > 0 && matFrameCurrent.cols > 0);
    assert(
            matFrameCurrent.rows == matFramePrevious.rows
            && matFrameCurrent.cols == matFramePrevious.cols);
    assert(
            matFrameCurrent.type() == CV_8U && matFramePrevious.type() == CV_8U);
    for (int i = 0; i < matFrameCurrent.rows; i++) {
        for (int j = 0; j < matFrameCurrent.cols; j++) {
            diff += matFrameCurrent.at<uchar>(i,j) - matFramePrevious.at<uchar>(i,j);
        }
    }
    return diff;
}

void Mono_Live_VIORB::query_maximum_resolution(cv::VideoCapture* camera, int &max_width, int &max_height)
{
  // Save current resolution
  const int current_width  = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
  const int current_height = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

  // Get maximum resolution
  camera->set(CV_CAP_PROP_FRAME_WIDTH,  10000);
  camera->set(CV_CAP_PROP_FRAME_HEIGHT, 10000);
  max_width  = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_WIDTH));
  max_height = static_cast<int>(camera->get(CV_CAP_PROP_FRAME_HEIGHT));

  // Restore resolution
  camera->set(CV_CAP_PROP_FRAME_WIDTH,  current_width);
  camera->set(CV_CAP_PROP_FRAME_HEIGHT, current_height);
}

void Mono_Live_VIORB::cameraLoop() {

    iFrame = 0;

    stream1.set(CV_CAP_PROP_FRAME_WIDTH,  max_width);
    stream1.set(CV_CAP_PROP_FRAME_HEIGHT, max_height);
//    cout << "max_width is " << max_width << ", max_height is " << max_height << endl;
//    cout << "get image width is " << stream1.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
//    cout << "get image height is " << stream1.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

    while (!time_to_exit) {

        timestampcamera_ns = boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        timestampcamera = boost::lexical_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

        _mutexFrameCam1Last.lock();
        stream1 >> matFrameForward;
        matFrameForward.convertTo(matFrameForward, CV_8U);
        cv::cvtColor(matFrameForward, matFrameForward, CV_BGR2GRAY);
        _mutexFrameCam1Last.unlock();
//        std::cout << "read matFrameForward size : " << matFrameForward.size() << std::endl;

        if(configParam->camera2 > 0) {
            pthread_mutex_lock(&_pmutexFrameCam2Last);
            stream2 >> matFrameDownward;
            pthread_mutex_unlock(&_pmutexFrameCam2Last);
        }

//        cv::imshow("Camera", matFrameForward);
//        if (cv::waitKey(1) >= 0) break;
        usleep(8000);
        iFrame++;
    }
}

void Mono_Live_VIORB::recordData() {

    int totalRecord = 0;
    cv::Mat recFrameForward, recFrameDownward;


    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    while(!time_to_exit){
//        cout << "matFrameForward.cols is " << matFrameForward.cols << endl;
        if(matFrameForward.cols != max_width) continue;

        _mutexFrameCam1Last.lock();
        matFrameForward.copyTo(recFrameForward);
        _mutexFrameCam1Last.unlock();
        if (configParam->camera2 > 0) {
            pthread_mutex_lock(&_pmutexFrameCam2Last);
            matFrameDownward.copyTo(recFrameDownward);
            pthread_mutex_unlock(&_pmutexFrameCam2Last);
        }

        if(totalRecord > 0) {
                imwrite(configParam->record_path + "/dataset-dir/cam0/" + std::to_string(timestampcamera_ns) + ".png", recFrameForward, compression_params);
                if (configParam->camera2 > 0) {
                    imwrite(configParam->record_path + "/dataset-dir/cam1/" +to_string(timestampcamera_ns) + ".png", recFrameDownward, compression_params);
                }
                lframe << timestampcamera_ns << "\n";
                totalRecord++;
        }
        else{
            totalRecord++;
        }
        usleep(configParam->timespace); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec
    }
    std::cout << "total record is " << totalRecord << std::endl;
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
    if (configParam->bRecord && iFrame > 0) {
        lgps << std::setprecision(10) << gps_pose.timestampunix_ns << sep << current_pose.gpstime << sep
             << current_pose.lat
             << sep << current_pose.lon << sep << current_pose.alt << sep << current_pose.gpsx << sep
             << current_pose.gpsy << sep << current_pose.gpsz << "\n";
    }
}
void Mono_Live_VIORB::getIMUdata(posedata current_pose_) {
    current_pose = current_pose_;

    ax = current_pose.xacc;
    ay = current_pose.yacc;
    az = current_pose.zacc;

    if (bAccMultiply98) {
        ax *= g3dm;
        ay *= g3dm;
        az *= g3dm;
    }
    // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
    ORB_SLAM2::IMUData imudata(current_pose.xgyro, current_pose.ygyro, current_pose.zgyro, ax, ay, az, current_pose.timestampunix_s);
    vimuData.push_back(imudata);

    if (configParam->bRecord){
//        std::cout << "write imu to imu0.csv \n";
        ldatasetimu << std::setprecision(10)<< current_pose.timestampunix_ns
                    << sep << current_pose.xgyro << sep << current_pose.ygyro << sep << current_pose.zgyro
                    << sep << current_pose.xacc << sep << current_pose.yacc << sep << current_pose.zacc << "\n";
    }
}

int Mono_Live_VIORB::getTrackingStage() {
    return SLAM->getTrackingStage();
}