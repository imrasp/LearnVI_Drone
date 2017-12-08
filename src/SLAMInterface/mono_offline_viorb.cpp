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
    if (!frame.is_open()) {
        cout << "ERROR: Cannot Open Frame File";
        throw 1;
    }
    ifstream imu(configParam->record_path + "/imu.csv");
    if (!imu.is_open()) {
        cout << "ERROR: Cannot Open IMU File";
        throw 1;
    }
    ifstream gps(configParam->record_path + "/gps.csv");
    if (!gps.is_open()) {
        cout << "ERROR: Cannot Open GPS File";
        throw 1;
    }
    std::string str;
    std::getline(imu, str);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(configParam->vocabulary, configParam->setting, ORB_SLAM2::System::MONOCULAR,
                                 string(configParam->gui) != "DISABLE");
    config = new ORB_SLAM2::ConfigParam(configParam->setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDe laySec);
    bAccMultiply98 = config->GetAccMultiply9p8();

    int frameno, imuframeno, gpsframeno;
    float timestamp, frametimestamp, firsttimestamp, imutimestamp, gpstimestamp, t;
    double xgyro, ygyro, zgyro;
    double xacc, yacc, zacc;
    float x, y, z;
    float roll, pitch, yaw;
    float lat, lon, alt;
    float gpsx, gpsy, gpsz;
    float satellites_visible, hdop;
    string getval;


    std::string::size_type sz;

    string line;
    string delimiter = ",";
    size_t pos = 0;
    string token;
    int splitpos, splitframepos;
    ORB_SLAM2::IMUData::vector_t vimuData;
    boost::posix_time::ptime const referencedTime(boost::gregorian::date(2015, 1, 1));

    while (frame.good()) {
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        auto ms = (boost::posix_time::microsec_clock::local_time() - referencedTime).total_milliseconds();
        double oms = boost::lexical_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()); //std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        string sms = to_string(ms);
        double fms = stod(sms);
        float fmsboost = (float) boost::lexical_cast<double>(oms);
        cout.precision(20);
        cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << " | oms = " << oms <<" | oms to string = " << to_string(oms) << " | oms to long = " << static_cast<long>(100 * stof(to_string(oms)))  << " | boost::lexical_cast<double> = " << boost::lexical_cast<double>(oms) << " | double to float = " << fmsboost << endl;

        getline(frame, getval, ',');
        frameno = atoi(getval.c_str()); // Frame number
        getline(frame, getval, '\n');
        frametimestamp = atof(getval.c_str()); // timestamp od frame

//        cout << "Grab frame" << frameno << endl;

        //get frame
        filename = configParam->record_path + "/Camera1/" + to_string(frameno) + ".jpg";
        matFrameForward = imread(filename, CV_LOAD_IMAGE_COLOR);
//        cout << "image loaded" << endl;

        //get imu data
        while (imu.good()) {
//            cout << "get imu value \n";
            getline(imu, getval, ',');
            imuframeno = atoi(getval.c_str());
//            cout << " imuframeno : " << imuframeno << endl;
            getline(imu, getval, ',');
            imutimestamp = atof(getval.c_str());
//            cout << " imutimestamp : " << imutimestamp << endl;
            getline(imu, getval, ',');
//            t = stof(getval, NULL);
            getline(imu, getval, ',');
            xgyro = atof(getval.c_str());
//            cout << " xgyro : " << xgyro << endl;
            getline(imu, getval, ',');
            ygyro = atof(getval.c_str());
//            cout << " ygyro : " << ygyro << endl;
            getline(imu, getval, ',');
            zgyro = atof(getval.c_str());
//            cout << " zgyro : " << zgyro << endl;
            getline(imu, getval, ',');
            xacc = atof(getval.c_str());
//            cout << " xacc : " << xacc << endl;
            getline(imu, getval, ',');
            yacc = atof(getval.c_str());
//            cout << " yacc : " << yacc << endl;
            getline(imu, getval, '\n');
            zacc = atof(getval.c_str());
//            cout << " zacc : " << zacc << endl;

//            cout << "get imu for frame no " << imuframeno << endl;
            float timestamp;
            if (firstTimestamp == 0) firstTimestamp = imutimestamp;
            timestamp = (imutimestamp - firstTimestamp);

            if (bAccMultiply98) {
                xacc *= g3dm;
                yacc *= g3dm;
                zacc *= g3dm;
            }

            if (frameno != imuframeno) {
//                std::cout << "Frame No: " << frameno << '\n';
//                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
//                std::cout << "-------------------" << '\n';

                if (bAccMultiply98) {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }
                ORB_SLAM2::GPSData gpsdata(0, 0, 0, 0, 0, 0, 0);
                // Pass the image to the SLAM system
                vision_estimated_pose = SLAM->TrackMonoVI(matFrameForward, vimuData, gpsdata, (frametimestamp-firstTimestamp)/1000);

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
    SLAM->SaveKeyFrameTrajectoryNavState("KeyFrameNavStateTrajectory.txt");

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
