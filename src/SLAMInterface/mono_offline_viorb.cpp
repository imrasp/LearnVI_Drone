#include "SLAMInterface/mono_offline_viorb.h"


Mono_Offline_VIORB::Mono_Offline_VIORB(System_Log *system_log_, SystemConfigParam *configParam_) : system_log(
        system_log_), configParam(configParam_) {
    time_to_exit = false;
    isFirstFrame = false;
    getFirstFrame = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;
    firstTimestamp = 0;
}

Mono_Offline_VIORB::~Mono_Offline_VIORB() {}

void Mono_Offline_VIORB::start() {

    cout << "Starting Offline SLAM..." << endl;

    //Checking for a file
    boost::filesystem::path dir(configParam->record_path);
    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;
    } else cout << dir << " is exist" << endl;

    ifstream frame(configParam->record_path + "/frame.csv");
    if (!frame.is_open()) throw "ERROR: Cannot Open Frame File";
    ifstream imu(configParam->record_path + "/imu.csv");
    if (!imu.is_open()) throw "ERROR: Cannot Open IMU File";
    ifstream gps(configParam->record_path + "/gps.csv");
    if (!gps.is_open()) throw "ERROR: Cannot Open GPS File";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(configParam->vocabulary, configParam->setting, ORB_SLAM2::System::MONOCULAR, true);
    config = new ORB_SLAM2::ConfigParam(configParam->setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();


    int frameno, imuframeno, gpsframeno;
    float timestamp, frametimestamp, firsttimestamp, imutimestamp, gpstimestamp;
    float xgyro, ygyro, zgyro;
    float xacc, yacc, zacc;
    float x, y, z;
    float roll, pitch, yaw;
    float lat, lon, alt;
    float gpsx, gpsy, gpsz;
    float satellites_visible, hdop;
    string getval;

    string line;
    string delimiter = ",";
    size_t pos = 0;
    string token;
    int splitpos, splitframepos;
    ORB_SLAM2::IMUData::vector_t vimuData;

    while (frame.good()) {
        //calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        getline(frame, getval, ',');
        frameno = stoi(getval, NULL); // Frame number
        getline(frame, getval, ',');
        frametimestamp = stof(getval, NULL); // timestamp od frame

        //get frame
        filename = configParam->record_path + "/Camera1/" + to_string(frameno) + ".jpg";
        matFrameForward = imread(filename, CV_LOAD_IMAGE_COLOR);

        //get imu data
        while (imu.good()) {

            getline(imu, getval, ',');
            imuframeno = stof(getval, NULL);
            getline(imu, getval, ',');
            imutimestamp = stof(getval, NULL);
            getline(imu, getval, ',');
            getline(imu, getval, ',');
            xgyro = stof(getval, NULL);
            getline(imu, getval, ',');
            ygyro = stof(getval, NULL);
            getline(imu, getval, ',');
            zgyro = stof(getval, NULL);
            getline(imu, getval, ',');
            xacc = stof(getval, NULL);
            getline(imu, getval, ',');
            yacc = stof(getval, NULL);
            getline(imu, getval, ',');
            zacc = stof(getval, NULL);

            float timestamp;
            if (firstTimestamp == 0) firstTimestamp = imutimestamp;
            timestamp = (imutimestamp - firstTimestamp) / 1000;

            if (bAccMultiply98) {
                xacc *= g3dm;
                yacc *= g3dm;
                zacc *= g3dm;
            }

            if (frameno != imuframeno) {
                std::cout << "Frame No: " << frameno << '\n';
                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
                std::cout << "-------------------" << '\n';

                if (bAccMultiply98) {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }
                if (gps.good()) {
                    while (gps.good()) {

                        getline(gps, getval, ',');
                        gpsframeno = stoi(getval, NULL);
                        getline(gps, getval, ',');
                        gpstimestamp = stof(getval, NULL);

                        if (frameno != gpsframeno) {
                            if (frametimestamp - gpstimestamp < 500) {
                                double timestamp = (gpstimestamp - firstTimestamp) / 1000;
                                ORB_SLAM2::GPSData gpsdata(lat, lon, alt, gpsx, gpsy, gpsz, timestamp);
                                // Pass the image to the SLAM system
                                vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, timestampc);
                            }
                            break;
                        }
                        getline(gps, getval, ',');
                        getline(gps, getval, ',');
                        lat = stof(getval, NULL);
                        getline(gps, getval, ',');
                        lon = stof(getval, NULL);
                        getline(gps, getval, ',');
                        alt = stof(getval, NULL);
                        getline(gps, getval, ',');
                        gpsx = stof(getval, NULL);
                        getline(gps, getval, ',');
                        gpsy = stof(getval, NULL);
                        getline(gps, getval, ',');
                        gpsz = stof(getval, NULL);
                    }
                } else {

                    ORB_SLAM2::GPSData gpsdata(0, 0, 0, 0, 0, 0, 0);
                    // Pass the image to the SLAM system

                    vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, timestampc);
                }

                vimuData.clear();
                // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, timestamp);
                vimuData.push_back(imudata);

                break;
            }
            // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
            ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, timestamp);
            vimuData.push_back(imudata);

        }
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
    }

    frame.close();
    imu.close();
    gps.close();

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

void Mono_Offline_VIORB::stop() {
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

double Mono_Offline_VIORB::frameDifference(cv::Mat &matFrameCurrent, Mat &matFramePrevious) {
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

void Mono_Offline_VIORB::calAvgProcessingTime(double time) {
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
