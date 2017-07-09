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
    mu->lock();
    cout << "Starting camera connection" << endl;
    mu->unlock();
    threadCamera = new boost::thread(&Camera_Capture::loopCamera, this);
    //threadCamera.join();
}

void Camera_Capture::stop()
{
    time_to_exit = true;
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

    frameno = 0;

    while (!time_to_exit)
    {
        mu->lock();
        //milliseconds_since_epoch
        tframe = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
        getPoseData();
        stream1.read(currentFrame);
        imshow("cam", currentFrame);
        //cout << "Frame" << ++frameno << " @ " << timestamp1 <<" @@ " << milliseconds_since_epoch << endl;
        mu->unlock();

        if (waitKey(30) >= 0)
            break;
        usleep(100000); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec
    }
}

void Camera_Capture::getPoseData()
{
    posdata = mavlinkControl->getCurrentPose();
   // cout << "current z is " << posdata.z <<endl;
   // printf("current z is %lf \n",posdata.z);
}

void Camera_Capture::getFrameData(Mat &cameraFrame_, positiondata posdata_ )
{
    cameraFrame_ = currentFrame;
    posdata_ = posdata;
}




