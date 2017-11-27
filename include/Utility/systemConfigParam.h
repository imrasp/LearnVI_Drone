//
// Created by rasp on 11/26/17.
//

#ifndef LEARNVI_DRONE_SYSTEMCONFIGPARAM_H
#define LEARNVI_DRONE_SYSTEMCONFIGPARAM_H

#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;

class SystemConfigParam {
public:
    static int camera1, camera2;

    static int  fps;
    static int timespace;
    static int baudrate;

    static string uart_name;
    static string vocabulary;
    static string setting;
    static string config;
    static string mode;
    static string gui;
    static string record_path;
    static string mission_route;

    static bool bRecordSLAM;
    static bool bLiveSLAM;
    static bool bMAVonly;

//    SystemConfigParam();
    SystemConfigParam(int argc, char **argv);

//    ~SystemConfigParam();

    void readParams();

    int getCamera1();
    int getFps();
    int getTimespace();
    int getBaudrate();
    string getUart_name();
    string getVocabulary();
    string getSetting();
    string getConfig();
    string getMode();
    string getGui();
    string getRecord_path();
    string getMission_route();
    bool isBRecordSLAM();
    bool isBLiveSLAM();
    bool isBMAVonly();

private:
    void fps2Timespace(int fps);
    void initialization();
    void setfps(int fps_);
    void parse_commandline(int argc, char **argv);

    void configMode(string mode);
};

#endif //LEARNVI_DRONE_SYSTEMCONFIGPARAM_H
