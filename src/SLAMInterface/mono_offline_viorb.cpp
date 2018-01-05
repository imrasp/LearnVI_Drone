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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(configParam->vocabulary, configParam->setting, ORB_SLAM2::System::MONOCULAR, true);
    ORB_SLAM2::ConfigParam config(configParam->setting);

//    double imageMsgDelaySec = config.GetImageDelayToIMU();
//    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    //Checking files
    boost::filesystem::path dir(configParam->record_path + "/cam0");
    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;
        throw 1;
    }
    std::ifstream imu(configParam->record_path + "/imu0.csv");
    if (!imu.is_open()) {
        std::cout << "ERROR: Cannot Open IMU File" << std::endl;
        throw 1;
    }

    // Read all images from folder order by name(default)
    std::vector<cv::String> fn;
    cv::glob(configParam->record_path + "/cam0/*.png", fn, false);

    cv::Mat image;
    double xgyro, ygyro, zgyro, ax, ay, az, timestamp;
    std::string getval;
    ORB_SLAM2::IMUData::vector_t vimuData;

    // load each frame with timestamp from name along with IMU data
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++) {
        std::string stimestamp = fn[i].substr(fn[i].find_last_of("/")+1, 19);
        double timestamp_camera = std::stod(stimestamp) / 1e9;
        image = cv::imread(fn[i]);
//        cv::imshow("Image", images.back());
//        if (cv::waitKey(1) >= 0) break;

        cout << "grab frame : " << fn[i] << endl;
        //load IMU data of this frame
        while (imu.good()) {
            getline(imu, getval, ',');
            timestamp = atof(getval.c_str())  / 1e9; //cout << " timestamp : " << timestamp << endl;
            getline(imu, getval, ',');
            xgyro = atof(getval.c_str()); // cout << " xgyro : " << xgyro << endl;
            getline(imu, getval, ',');
            ygyro = atof(getval.c_str()); // cout << " ygyro : " << ygyro << endl;
            getline(imu, getval, ',');
            zgyro = atof(getval.c_str()); // cout << " zgyro : " << zgyro << endl;
            getline(imu, getval, ',');
            ax = atof(getval.c_str()); // cout << " xacc : " << xacc << endl;
            getline(imu, getval, ',');
            ay = atof(getval.c_str()); // cout << " yacc : " << yacc << endl;
            getline(imu, getval, '\n');
            az = atof(getval.c_str()); // cout << " zacc : " << zacc << endl;

            if (bAccMultiply98) {
                ax *= g3dm;
                ay *= g3dm;
                az *= g3dm;
            }

            if (timestamp >= timestamp_camera) {
//                std::cout << "-------------------" << '\n';
//                std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << timestamp << '\n';
//                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
//                std::cout << "-------------------" << '\n';

                ORB_SLAM2::GPSData gpsdata(0, 0, 0, 0, 0, 0, 0);
                // Pass the image to the SLAM system
                vision_estimated_pose = SLAM.TrackMonoVI(image, vimuData, gpsdata, timestamp_camera);

                vimuData.clear();
                // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                vimuData.push_back(imudata);

                break;

            } else {
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                vimuData.push_back(imudata);
            }
        }
    }


    std::cout << "Save camera trajectory..." << std::endl;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(configParam->record_path + "/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(configParam->record_path + "/KeyFrameNavStateTrajectory.txt");

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

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
