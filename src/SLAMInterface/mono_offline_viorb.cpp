#include "SLAMInterface/mono_offline_viorb.h"


Mono_Offline_VIORB::Mono_Offline_VIORB(System_Log *system_log_) : system_log(system_log_)
{
    time_to_exit = false;
    isFirstFrame = false;
    getFirstFrame = false;
    startCalprocessingTime = true;
    avgTime = 0;
    frameNo = 0;
    firstTimestamp = 0;
}

Mono_Offline_VIORB::~Mono_Offline_VIORB()
{}

void Mono_Offline_VIORB::start(char *&vocabulary, char *&setting, string foldername_) {
        
cout << "Starting Offline SLAM..." << endl;
    foldername = "../sample_data/"+foldername_; //foldername_;

    //Checking for a file
    boost::filesystem::path dir(foldername);
    if(!(boost::filesystem::exists (dir))) {
        std::cout << "Doesn't Exists" << std::endl;
    }
    else cout << dir << " is exist" << endl;

    ifstream frame(foldername+"/tframe.txt");
    if(!frame.is_open()) std::cout << "ERROR: Cannot Open Frame File" << '\n';
    ifstream imu(foldername+"/posedata.csv");
    if(!imu.is_open()) std::cout << "ERROR: Cannot Open IMU File" << '\n';

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(vocabulary, setting, ORB_SLAM2::System::MONOCULAR, true);
    config = new ORB_SLAM2::ConfigParam(setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();


    int frameno, imuframeno;
    float timestamp, frametimestamp, firsttimestamp;
    float xgyro, ygyro, zgyro;
    float xacc, yacc, zacc;
    float x, y, z;
    float roll, pitch, yaw;
    float lat, lon, alt;
    float satellites_visible, hdop;
    string getval;

    string line;
    string delimiter = ",";
    size_t pos = 0;
    string token;
    int splitpos, splitframepos;
    ORB_SLAM2::IMUData::vector_t vimuData;

    while ( frame.good() ) {
        //calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

        getline (frame,line);
        cout << line << endl;
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            if(token == "Frame") splitframepos = 0;
            else {
                splitframepos++;
                if(splitframepos == 1) frameno = atof(token.c_str());
                else if(splitframepos == 2) {
                    {
                        if (firstTimestamp == 0) firstTimestamp = atof(token.c_str());
                        frametimestamp = (atof(token.c_str()) - firstTimestamp ); // /1000
                    }
                    //cout << "ft is " << ftimestamp << endl;
                }
            }
            //std::cout << token << std::endl;
            line.erase(0, pos + delimiter.length());
        }

        //get frame
        filename = foldername+"/"+to_string(frameno)+".jpg";
        matFrameForward = imread(filename, CV_LOAD_IMAGE_COLOR);

        //get imu data
        while (imu.good()) {
            getline(imu, getval, ','); // "Pose" header
            getline(imu, getval, ','); imuframeno = stof(getval, NULL);
            getline(imu, getval, ','); timestamp = stof(getval, NULL);
            getline(imu, getval, ','); xgyro = stof(getval, NULL);
            getline(imu, getval, ','); ygyro = stof(getval, NULL);
            getline(imu, getval, ','); zgyro = stof(getval, NULL);
            getline(imu, getval, ','); xacc = stof(getval, NULL);
            getline(imu, getval, ','); yacc = stof(getval, NULL);
            getline(imu, getval, ','); zacc = stof(getval, NULL);
            getline(imu, getval, ','); x = stof(getval, NULL);
            getline(imu, getval, ','); y = stof(getval, NULL);
            getline(imu, getval, ','); z = stof(getval, NULL);
            getline(imu, getval, ','); roll = stof(getval, NULL);
            getline(imu, getval, ','); pitch = stof(getval, NULL);
            getline(imu, getval, ','); yaw = stof(getval, NULL);
            getline(imu, getval, ','); lat = stof(getval, NULL);
            getline(imu, getval, ','); lon = stof(getval, NULL);
            getline(imu, getval, ','); alt = stof(getval, NULL);
            getline(imu, getval, ','); satellites_visible = stof(getval, NULL);
            getline(imu, getval, '\n'); hdop = stof(getval, NULL);

            if (frameno != imuframeno) {
                std::cout << "Frame No: " << frameno << '\n';
                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
                std::cout << "-------------------" << '\n';

                if (bAccMultiply98) {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }

                SLAM->TrackMonoVI(matFrameForward, vimuData, frametimestamp);
                vimuData.clear();

                // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, xacc, yacc, zacc, timestamp );
                vimuData.push_back(imudata);
            }


        }
        calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
    }

    frame.close();
    imu.close();

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

void Mono_Offline_VIORB::stop()
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

void Mono_Offline_VIORB::grabFrameData()
{
    cout << "grab Frame data for SLAM..." <<endl;
    cout << "Start processing sequence ..." << endl;

    int frameno;
    string line;
    string delimiter = ",";
    size_t pos = 0;
    string token;
    int splitpos, splitframepos;
    int fno = 1, countimu;
    Mat matFrameForward;
    double roll,pitch,yaw,ax,ay,az,timestamp, ftimestamp, previousroll;
    ORB_SLAM2::IMUData::vector_t vimuData;

    ifstream imufile ("../sample_data/"+foldername+"/imulog.txt");
    ifstream framefile ("../sample_data/"+foldername+"/tframe.txt");

    if (framefile.is_open())
    {
        while ( framefile.good() )
        {
            calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

            getline (framefile,line);
            cout << line << endl;
            while ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                if(token == "Frame") splitframepos = 0;
                else {
                    splitframepos++;
                    if(splitframepos == 1) frameno = atof(token.c_str());
                    else if(splitframepos == 2) {
                        {
                            if (firstTimestamp == 0) firstTimestamp = atof(token.c_str());
                            ftimestamp = (atof(token.c_str()) - firstTimestamp ); // /1000
                        }
                        //cout << "ft is " << ftimestamp << endl;
                    }
                }
                //std::cout << token << std::endl;
                line.erase(0, pos + delimiter.length());
            }

            //get frame
            filename = "../sample_data/"+foldername+"/"+to_string(frameno)+".jpg";
            matFrameForward = imread(filename, CV_LOAD_IMAGE_COLOR);
//            imshow( "Display window", matFrameForward );
//            waitKey(30);

            //get imu data
            if (imufile.is_open())
            {
                countimu = 0;
                while ( imufile.good() && fno == frameno )
                {
                    countimu++;
                    get_new_imu:
                    getline (imufile,line);
                    cout << line << endl;
                    while ((pos = line.find(delimiter)) != std::string::npos) {
                        token = line.substr(0, pos);
                        if(token == "IMU") splitpos = 0;
                        else {
                            splitpos++;
                            if (splitpos == 1) fno = atof(token.c_str());
                            if(fno == frameno)
                            {
                                if (splitpos == 2) {
                                    roll = atof(token.c_str());
                                    if(roll == previousroll) goto get_new_imu;
                                }
                                else if (splitpos == 3) pitch = atof(token.c_str());
                                else if (splitpos == 4) yaw = atof(token.c_str());
                                else if (splitpos == 5) ax = atof(token.c_str());
                                else if (splitpos == 6) ay = atof(token.c_str());
                                else if (splitpos == 7) az = atof(token.c_str());
                                else if (splitpos == 8)
                                {
                                    if (firstTimestamp == 0) firstTimestamp = atof(token.c_str());
                                    timestamp = (atof(token.c_str()) - firstTimestamp );
                                }
                            }
                            else break;
                        }
                        //std::cout << token << std::endl;
                        line.erase(0, pos + delimiter.length());
                    }

                    if (bAccMultiply98) {
                        ax *= g3dm;
                        ay *= g3dm;
                        az *= g3dm;
                    }
                    // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                    ORB_SLAM2::IMUData imudata(roll, pitch, yaw, ax, ay, az, timestamp );
                    vimuData.push_back(imudata);
                }
                //cout <<"fno is " << fno << " and frameno is " << frameno << endl;

            } //end if imu file
            else cout << "Unable to open IMU file";

            SLAM->TrackMonoVI(matFrameForward, vimuData, ftimestamp);
            vimuData.clear();

            //cout << "total imus for this frame  = " << countimu <<endl;
            calAvgProcessingTime(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
        }
        framefile.close();
        imufile.close();
    }//end if frame file
    else cout << "Unable to open Frame file";

    stop();

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
