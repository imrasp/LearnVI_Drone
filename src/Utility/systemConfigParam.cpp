//
// Created by rasp on 11/26/17.
//


#include "Utility/systemConfigParam.h"

int SystemConfigParam::camera1;
int SystemConfigParam::camera2;
int SystemConfigParam::fps;
int SystemConfigParam::timespace;
int SystemConfigParam::baudrate;

string SystemConfigParam::uart_name;
string SystemConfigParam::vocabulary;
string SystemConfigParam::setting;
string SystemConfigParam::config;
string SystemConfigParam::mode;
string SystemConfigParam::gui;
string SystemConfigParam::record_path;
string SystemConfigParam::mission_route;

bool SystemConfigParam::bRecordSLAM;
bool SystemConfigParam::bLiveSLAM;
bool SystemConfigParam::bMAVonly;
bool SystemConfigParam::bOffline;

SystemConfigParam::SystemConfigParam()
{
}
SystemConfigParam::SystemConfigParam(int argc, char **argv)
{
    parse_commandline(argc, argv);
    readParams();
}

SystemConfigParam::~SystemConfigParam()
{
}

void SystemConfigParam::initialization(){
    // Default input arguments
#ifdef __APPLE__
    uart_name = (char*)"/dev/tty.usbmodem1";
#else
    uart_name = (char *) "/dev/ttyUSB0";
#endif

    baudrate = 921600; // 57600 or 921600 px4 companion link buadrate
    vocabulary = "../Vocabulary/ORBvoc.txt";
    setting = "../config/mobius.yaml";
    mode = "LIVE";
    gui = "ENABLE";
    record_path = "/home/odroid/workspace/VIDrone/record_data/";
    fps = 10;
    timespace = 1000000;
    mission_route = "./mission_route.txt";
    camera1 = 1;
}

void SystemConfigParam::readParams(){
    cv::FileStorage fSettings(config, cv::FileStorage::READ);

    baudrate = fSettings["system.baudrate"];
    fSettings["system.uart_name"] >> uart_name;
    fSettings["system.vocabulary"] >> vocabulary;
    fSettings["system.setting"] >> setting;
    fSettings["system.mode"] >> mode;
    configMode(mode);
    fSettings["system.gui"] >> gui;
    fSettings["system.record_path"] >> record_path;
    fps = fSettings["system.fps"];
    fps2Timespace(fps);
    fSettings["system.mission_route"] >> mission_route;
    camera1 = fSettings["system.camera1"];
    camera2 = fSettings["system.camera2"];

    cout << "Parameters :" << endl;
    cout << "UART name " << uart_name << " baudrate " << baudrate << endl;
    cout << "Setting file path : " << setting << endl;
    cout << "Mode : " << mode << " GUI " << gui << endl;
    cout << "Save result at " << record_path << endl;
    cout << "FPS : " << fps << " with duration between frame is " <<timespace << endl;
    cout << "1st Camera : /dev/video" << camera1 << endl;
    cout << "2st Camera : /dev/video" << camera2 << endl;
    cout << "Mission route : " << mission_route << endl;
}

void SystemConfigParam::setfps(int fps_){
    fps = fps_;
    timespace = 10000000/fps;
}

void SystemConfigParam::fps2Timespace(int fps){
    timespace = 10000000/fps;
}

void SystemConfigParam::configMode(string mode_){
    if(string(mode_) == "LIVERECORD"){
        bLiveSLAM = true;
        bRecordSLAM = true;
        bMAVonly = false;
        bOffline = false;
    } else if(string(mode_) == "LIVE"){
        bLiveSLAM = true;
        bRecordSLAM = false;
        bMAVonly = false;
        bOffline = false;
    } else if(string(mode_) == "MAVONLY"){
        bLiveSLAM = false;
        bRecordSLAM = false;
        bMAVonly = true;
        bOffline = false;
    } else if(string(mode_) == "OFFLINE"){
        bLiveSLAM = false;
        bRecordSLAM = false;
        bMAVonly = false;
        bOffline = true;
    } else {
        cout << mode_ << "is not implemented (option : LIVERECORD, LIVE, MAVONLY)" << endl;
        throw 1;
    }

}

//   Parse Command Line
void SystemConfigParam::parse_commandline(int argc, char **argv) {
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -c <path_to_config>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // system_config.txt location
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) {
            if (argc > i + 1) {
                config = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    return;
}

int SystemConfigParam::getCamera1() {
    return camera1;
}

int SystemConfigParam::getFps() {
    return fps;
}

int SystemConfigParam::getTimespace() {
    return timespace;
}

int SystemConfigParam::getBaudrate() {
    return baudrate;
}

string SystemConfigParam::getUart_name() {
    return uart_name;
}

string SystemConfigParam::getVocabulary() {
    return vocabulary;
}

string SystemConfigParam::getSetting() {
    return setting;
}

string SystemConfigParam::getConfig() {
    return config;
}

string SystemConfigParam::getMode() {
    return mode;
}

string SystemConfigParam::getGui() {
    return gui;
}

string SystemConfigParam::getRecord_path() {
    return record_path;
}

string SystemConfigParam::getMission_route() {
    return mission_route;
}

bool SystemConfigParam::isBRecordSLAM() {
    return bRecordSLAM;
}

bool SystemConfigParam::isBLiveSLAM() {
    return bLiveSLAM;
}

bool SystemConfigParam::isBMAVonly() {
    return bMAVonly;
}

int SystemConfigParam::getCamera2() {
    return camera2;
}

bool SystemConfigParam::isBOffline() {
    return bOffline;
}
