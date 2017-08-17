#include "SLAMInterface/mono_record_viorb.h"

Mono_Record_VIORB::Mono_Record_VIORB() {

}

Mono_Record_VIORB::~Mono_Record_VIORB() {

}

Mono_Record_VIORB::Mono_Record_VIORB(System_Log *system_log_, bool gui, Mono_Live_VIORB *mono_live_viorb_, int timespace_)
        : system_log(system_log_), bUseView(gui), mono_live_viorb(mono_live_viorb_), timespace(timespace_) {
    time_to_exit = false;
}

void Mono_Record_VIORB::start(char *filename) {
    printf("Initialize IMU file (txt) \n");

    boost::filesystem::path dir("../sample_data/" + string(filename));
    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created !" << std::endl;
    }

    foldername = string(filename);

    imulog.open("../sample_data/" + string(filename) + "/imulog.txt");
    tframelog.open("../sample_data/" + string(filename) + "/tframe.txt");
    csvposelog.open("../sample_data/" + string(filename) + "/posedata.csv");

    csvposelog
            << string("Pose") + to_string(frameno) + "timestamp" + "," + "," + "xgyro" + "," + "xyyro" + "," + "zgyro" +
               "," + "xacc" + "," + "yacc" + "," + "zacc" + "," + "x" + "," + "y" + "," + "z" + "," + "roll" + "," +
               "pitch" + "," + "yaw" + "," + "zacc" + "," + "lat" + "," + "lon" + "," + "alt" + "," +
               "satellites_visible" + "," + "hdop" + "," + "\n";


    if (bUseView){
        cout << "Starting camera thread in Record mode..." << endl;
        mono_live_viorb = new Mono_Live_VIORB(system_log, false);
        mono_live_viorb->findCamera();
        threadCamera = new boost::thread(&Mono_Live_VIORB::cameraLoop, mono_live_viorb);
    }
    cout << "Starting recorder thread..." << endl;
    threadRecorder = new boost::thread(&Mono_Record_VIORB::cameraRecorder, this);
}

void Mono_Record_VIORB::stop() {
    time_to_exit = true;
    imulog.close();
}

void Mono_Record_VIORB::cameraRecorder() {
    while (!time_to_exit) {
        cout << "++++++++++++++++++++ copy frame ++++++++++++++++++++" <<endl;
        cout << "matFrame " << mono_live_viorb->matFrame.size() << endl;
        mono_live_viorb->matFrame.copyTo(currentFrame);
//        currentFrame = mono_live_viorb->matFrame.clone();
        cout << "current Frame : " << currentFrame.size() <<endl;
        imgname = "../sample_data/"+foldername+"/"+to_string(frameno)+".jpg";
        imwrite(imgname,currentFrame);
        tframelog << string("Frame,") + to_string(frameno) + "," + to_string(tframe) + "," + "\n";
        if (bUseView) {
            imshow("Camera Recorder", currentFrame);
            if (waitKey(30) >= 0)
                break;
        }
        usleep(timespace); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec
        frameno++;
    }
}

void Mono_Record_VIORB::loopCamera() {
    cout << "Starting camera connection..." << endl;
    VideoCapture stream;   //0 is the id of video device.0 if you have only one camera.

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

    VideoCapture stream1 = VideoCapture(i);


    string filename;


    while (!time_to_exit) {
        //milliseconds_since_epoch
        tframe = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
        tframelog << string("Frame,") + to_string(frameno) + "," + to_string(tframe) + "," + "\n";

        stream1.read(currentFrame);
        //imshow("cam", currentFrame);

        filename = "./sample_data/" + foldername + "/" + to_string(frameno) + ".jpg";
        imwrite(filename, currentFrame);

        usleep(1000000); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec

        if (waitKey(30) >= 0)
            break;
        frameno++;
    }
}

void Mono_Record_VIORB::getPoseData(posedata current_pose) {

    double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);

    imulog << string("IMU,") + to_string(frameno)
              + "," + to_string(current_pose.xgyro) + "," + to_string(current_pose.ygyro) + "," + to_string(current_pose.zgyro)
              + "," + to_string(current_pose.xacc) + "," + to_string(current_pose.yacc) + "," + to_string(current_pose.zacc)
              + "," + to_string(timestamp) + "," + "\n";

    csvposelog << string("Pose") + "," + to_string(frameno) + "," + to_string(timestamp) + ","
                  + to_string(current_pose.xgyro) + "," + to_string(current_pose.ygyro) + "," + to_string(current_pose.zgyro) + ","
                  + to_string(current_pose.xacc) + "," + to_string(current_pose.yacc) + "," + to_string(current_pose.zacc) + ","
                  + to_string(current_pose.x) + "," + to_string(current_pose.y) + "," + to_string(current_pose.z) + ","
                  + to_string(current_pose.roll) + "," + to_string(current_pose.pitch) + "," + to_string(current_pose.yaw) + ","
                  + to_string(current_pose.lat) + "," + to_string(current_pose.lon) + "," + to_string(current_pose.alt) + ","
                  + to_string(current_pose.satellites_visible) + "," + to_string(current_pose.hdop) + "," + "\n";
}
