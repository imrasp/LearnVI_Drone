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
    cout << "Starting SLAM..." << endl;
    foldername = foldername_;
    boost::filesystem::path dir("../sample_data/"+foldername);
    if(!(boost::filesystem::exists (dir))) {
        std::cout << "Doesn't Exists" << std::endl;
    }
    else cout << dir << " is exist" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(vocabulary, setting, ORB_SLAM2::System::MONOCULAR, true);
    config = new ORB_SLAM2::ConfigParam(setting);

    imageMsgDelaySec = config->GetImageDelayToIMU();
    // ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    bAccMultiply98 = config->GetAccMultiply9p8();

    grabFrameData();

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
    std::vector<ORB_SLAM2::IMUData> vimuData;

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