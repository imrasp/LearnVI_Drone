//
// Created by rasp on 5/31/17.
//

#ifndef DRONE_PROJECT_MAVLINK_CONTROL_H
#define DRONE_PROJECT_MAVLINK_CONTROL_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>

#include <common/mavlink.h>

#include "autopilot_interface.h"
#include "serial_port.h"
#include "location_manager.h"
#include "system_log.h"

using std::string;
using namespace std;


//int main(int argc, char **argv);
struct positiondata {
    float timestamp_pos;
    float x;
    float y;
    float z;

    float xacc, yacc, zacc;

    float roll;
    float pitch;
    float yaw;
    uint8_t satellites_visible;
    uint16_t hdop;
    int32_t lat;
    int32_t lon;
    int32_t alt;
};

class Mavlink_Control{
public:
    Mavlink_Control(int baudrate, char *&uart_name);
    ~Mavlink_Control();

    //int start(int argc, char **argv);
    //int start(int baudrate, char *&uart_name);
    void start();
    void stop();
    positiondata getCurrentPose();
private:

    void commands();
    void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

    void takeoff(float h, mavlink_set_position_target_local_ned_t &sp);
    void land(mavlink_set_position_target_local_ned_t &sp);
    void goto_local_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
    void goto_global_position(float lat, float lon, float alt, mavlink_set_position_target_local_ned_t &sp);
    void goto_velocity(float x, float y, float z, float speed, mavlink_set_position_target_local_ned_t &sp);
    void hold_position(int sec, mavlink_set_position_target_local_ned_t &sp);
    void yaw_rotation(float yaw, mavlink_set_position_target_local_ned_t &sp);
    void yaw_rotation_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

    bool IsInWaypointLocal(mavlink_local_position_ned_t current, mavlink_set_position_target_local_ned_t goal, float radious);
    bool IsInWaypointGlobal(mavlink_local_position_ned_t current, mavlink_global_position_int_t goal, float radius);
    bool IsInWaypointGlobal(mavlink_global_position_int_t current, mavlink_set_position_target_local_ned_t goal, float radius);

    // quit handler
    Autopilot_Interface *autopilot_interface_quit;
    Serial_Port *serial_port_quit;
    static void quit_handler( int sig );

    System_Log syslog;
    Location_Manager location_manager;
    Serial_Port serial_port;
    Autopilot_Interface* autopilot_interface;
    int testint;

    float maxRotation; // degree/second
};

#endif //DRONE_PROJECT_MAVLINK_CONTROL_H
