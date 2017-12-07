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

    static bool bRecord;
    static bool bLive;
    static bool bMAVonly;
    static bool bOffline;
    static bool bMAVRecord;

    static int getCamera2();

    static bool isBOffline();

    SystemConfigParam();
    SystemConfigParam(int argc, char **argv);

    ~SystemConfigParam();

    void readParams();

    static int getCamera1();
    static int getFps();
    static int getTimespace();
    static int getBaudrate();
    static string getUart_name();
    static string getVocabulary();
    static string getSetting();
    static string getConfig();
    static string getMode();
    static string getGui();
    static string getRecord_path();
    static string getMission_route();
    static bool isBRecord();
    static bool isBLive();
    static bool isBMAVonly();

private:
    void fps2Timespace(int fps);
    void parse_commandline(int argc, char **argv);

    void configMode(string mode);
};

#endif //LEARNVI_DRONE_SYSTEMCONFIGPARAM_H
