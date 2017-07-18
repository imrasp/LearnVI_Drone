//
// Created by rasp on 5/31/17.
//


#include "cameraCapture.h"


Camera_Capture::Camera_Capture(boost::mutex *pMu, Mavlink_Control *mavlinkControl_)
{
    mu = pMu;
    mavlinkControl = mavlinkControl_;
    time_to_exit = false;
}

Camera_Capture::~Camera_Capture()
{

}

void Camera_Capture::start()
{
    printf("Initialize IMU file (txt) \n");

    cout << "please enter folder name ::" <<endl;
    cin >> foldername;

    boost::filesystem::path dir("./sample_data/"+foldername);
    if(!(boost::filesystem::exists (dir))){
        std::cout<<"Doesn't Exists"<<std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created !" << std::endl;
    }


    imulog.open("./sample_data/"+foldername+"/imulog.txt");
    tframelog.open("./sample_data/"+foldername+"/tframe.txt");

    frameno = 1;
    cout << "Starting IMU thread..." << endl;
    boost::thread threadIMUdata = boost::thread(&Camera_Capture::getIMUdata, this);

    cout << "Starting camera thread..." << endl;
    threadCamera = new boost::thread(&Camera_Capture::loopCamera, this);
}

void Camera_Capture::stop()
{
    time_to_exit = true;
    imulog.close();
}

int Camera_Capture::findCamera()
{
    VideoCapture stream;   //0 is the id of video device.0 if you have only one camera.

    int maxTested = 2;
    int i;
    for (i = maxTested; i >= 0; i--){
        VideoCapture stream(i);
        bool res = (stream.isOpened());
        cout << res <<endl;
        if (res)
        {
            cout <<"Open camera " << i <<endl;
            return i;

        }
        else
        {
            stream.release();
            cout <<"Camera " << i << " is released" <<endl;
        }
    }
    return -1;
}

void Camera_Capture::loopCamera()
{
    cout << "Starting camera connection..." << endl;
    VideoCapture stream;   //0 is the id of video device.0 if you have only one camera.

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

    VideoCapture stream1  = VideoCapture(i);


    string filename;


    while (!time_to_exit)
    {

        //milliseconds_since_epoch
        tframe = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        tframelog << string("Frame,") + to_string(frameno) + "," + to_string(tframe) + "\n";

        stream1.read(currentFrame);
        imshow("cam", currentFrame);

        filename = "./sample_data/"+foldername+"/"+to_string(frameno)+".jpg";
        imwrite(filename,currentFrame);

        usleep(100000); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec

        if (waitKey(30) >= 0)
            break;
        frameno++;
    }
}

void Camera_Capture::getIMUdata() {
    posdatalastest.roll = 0;
    cout << "start getting imu data ... " << endl;
    while (!time_to_exit) {
        posdatac = mavlinkControl->getCurrentPose();

        double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);

        rollc = posdatac.roll;

        if(rollc == posdatalastest.roll) continue;
        pitchc = posdatac.pitch;
        yawc = posdatac.yaw;

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
        // ORB_SLAM2::IMUData imudata(rollc, pitchc, yawc, ax, ay, az, timestamp );
        imulog << string("IMU,")  + to_string(frameno)
                  + "," + to_string(rollc) + "," + to_string(pitchc) + "," + to_string(yawc)
                  + "," + to_string(ax) + "," + to_string(ay) + "," + to_string(az)
                  + "," + to_string(timestamp) + "," + "\n";
        posdatac = posdatalastest;

        usleep(200);
    }
}



