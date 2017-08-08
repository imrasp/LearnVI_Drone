#include "MAVControl/mavlink_control.h"

Mavlink_Control::Mavlink_Control() {

}
Mavlink_Control::Mavlink_Control(int baudrate, char *&uart_name, System_Log *system_log, Location_Manager *location_manager_) : location_manager(location_manager_)
{
    //   PORT and THREAD STARTUP
    serial_port = Serial_Port(uart_name, baudrate, system_log);

    autopilot_interface = new Autopilot_Interface(&serial_port, system_log, location_manager);

    // Setup interrupt signal handler

//    serial_port_quit         = &serial_port;
//    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT,quit_handler);

    //Start the port and autopilot_interface
    serial_port.start();
    autopilot_interface->start();

}
Mavlink_Control::~Mavlink_Control()
{}

void Mavlink_Control::start()
{
    //   RUN COMMANDS
    commands();

    // Give sometime for system to print out more information
    //sleep(10);

}
void Mavlink_Control::stop()
{
    //   THREAD and PORT SHUTDOWN
    autopilot_interface->stop();
    serial_port.stop();
}

void Mavlink_Control::commands()
{
    printf("start command...");

//    autopilot_interface->arm_control(); usleep(100);

    autopilot_interface->enable_offboard_control(); usleep(100);


//    printf("SEND OFFBOARD COMMANDS\n");
//    mavlink_set_position_target_local_ned_t sp;
//    mavlink_set_position_target_local_ned_t ip = autopilot_interface->initial_position;
//
//    autopilot_interface->switchUpdatePosition(true);
//    syslog.write2csv("Update position = true");
//    takeoff(ip.z - 5.0, sp);

    /*float ninetydeg = ( 90 * 180 ) / M_PI;
    mavlink_attitude_t catt = autopilot_interface->current_messages.attitude;
    printf("ATTITUDE : roll %lf pitch %lf yaw %lf \n", catt.roll, catt.pitch, catt.yaw);
    printf("first turn \n");
    yaw_rotation(ninetydeg,sp);
    sleep(5);
    catt = autopilot_interface->current_messages.attitude;
    printf("ATTITUDE : roll %lf pitch %lf yaw %lf \n", catt.roll, catt.pitch, catt.yaw);
*/
//	printf("second turn \n");
//	yaw_rotation(90,sp);
//	sleep(5);
//	catt = autopilot_interface->current_messages.attitude;
//	printf("ATTITUDE : roll %lf pitch %lf yaw %lf \n", catt.roll, catt.pitch, catt.yaw);
//
//	printf("last turn \n");
//	yaw_rotation(270,sp);
//	sleep(5);
//	catt = autopilot_interface->current_messages.attitude;
//	printf("ATTITUDE : roll %lf pitch %lf yaw %lf \n", catt.roll, catt.pitch, catt.yaw);

    //	mavlink_local_position_ned_t cp = autopilot_interface->current_messages.local_position_ned;
    //	goto_local_position(cp.x - 5.0, cp.y,  cp.z, sp);
    //	goto_local_position(cp.x- 5.0, cp.y- 5.0,  cp.z, sp);
    //
    //	autopilot_interface->switchUpdatePosition(false);
    //	syslog.write2csv("Update position = false");
    //	goto_local_position(cp.x, cp.y- 5.0,  cp.z, sp);
    //	goto_local_position(cp.x, cp.y,  cp.z, sp);
    //
    //	hold_position(5, sp);
    //	land(sp);
    //	sleep(1);
    //	cp = autopilot_interface->current_messages.local_position_ned;
    //	takeoff(cp.z - 5.0, sp);
    //
    //	cp = autopilot_interface->current_messages.local_position_ned;
    //	autopilot_interface->switchUpdatePosition(true);
    //	syslog.write2csv("Update position = true");
    //	goto_velocity(cp.x - 10.0, cp.y,  cp.z, 1.0, sp);
    //	goto_velocity(cp.x- 10.0, cp.y - 10.0, cp.z, 1.0, sp);
    //
    //	autopilot_interface->switchUpdatePosition(false);
    //	syslog.write2csv("Update position = false");
    //	goto_velocity(cp.x, cp.y - 10.0,  cp.z, 1.0, sp);
    //	goto_velocity(cp.x, cp.y, cp.z, 1.0, sp);
    //
    //	//	mavlink_global_position_int_t cpg = autopilot_interface->current_messages.global_position_int;
    //	//	goto_local_position(sp);

//    hold_position(5, sp);
//    land(sp);

//    autopilot_interface->disarm_control(); usleep(100);
    sleep(60);
    autopilot_interface->disable_offboard_control(); usleep(100);

    printf("end offboard command \n");

    return;

}

//   Quit Signal Handler
// this function is called when you press Ctrl-C
void Mavlink_Control::quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        //autopilot_interface_quit->handle_quit(sig);
        //autopilot_interface->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        //serial_port_quit->handle_quit(sig);
        //serial_port->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);
}
void Mavlink_Control::takeoff(float  h, mavlink_set_position_target_local_ned_t &sp)
{
    set_takeoff_position( 0.0, 0.0,  h, sp ); // [m, m, m, sp]
    autopilot_interface->update_setpoint(sp);
    while (autopilot_interface->current_messages.local_position_ned.z > h)
    {
        //printf("current height is %f \n", (float)autopilot_interface->current_messages.local_position_ned.z);
        system_log->write2txt("current takeoff position ", autopilot_interface->current_messages.local_position_ned);
        autopilot_interface->update_setpoint(sp);
        usleep(50000);
    }
    printf("current height after takeoff is %f \n", (float)autopilot_interface->current_messages.local_position_ned.z);
    printf("takeoff complete!");
}

void Mavlink_Control::land(mavlink_set_position_target_local_ned_t &sp)
{
    set_land_position( 0.0, 0.0,  0.0, sp ); // [m, m, m, sp]
    autopilot_interface->update_setpoint(sp);
    while (autopilot_interface->current_messages.local_position_ned.z <= 0.00)
    {
        //printf("current height is %f \n", (float)autopilot_interface->current_messages.local_position_ned.z);
        system_log->write2txt("current land position ", autopilot_interface->current_messages.local_position_ned);
        usleep(50000);
    }
    printf("current height after land is %f \n", (float)autopilot_interface->current_messages.local_position_ned.z);
    printf("land complete!");
}
void Mavlink_Control::goto_local_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    set_position( x, y, z, sp ); // [m, m, m, sp]
    autopilot_interface->update_setpoint(sp);
    printf("the next position is %f, %f, %f \n", x, y, z);
    while (!IsInWaypointLocal(autopilot_interface->current_messages.local_position_ned, sp, 0.5))
    {
        //printf("check IsInWaypoint? current x : %lf", autopilot_interface->current_messages.local_position_ned.x);
        system_log->write2txt("current position (after command to move) ", autopilot_interface->current_messages.local_position_ned);
        //set_position( x, y, z, sp);
        //autopilot_interface->update_setpoint(sp);
        usleep(50000);
    }
    printf("current x, y, z : %lf, %lf, %lf \n", autopilot_interface->current_messages.local_position_ned.x,
           autopilot_interface->current_messages.local_position_ned.y,
           autopilot_interface->current_messages.local_position_ned.z);
    printf("complete using local position move to %f, %f, %f \n", x, y, z);
    return;
}

void Mavlink_Control::goto_global_position(float lat, float lon, float alt, mavlink_set_position_target_local_ned_t &sp)
{
    mavlink_global_position_int_t gps_setpoint_pos;
    gps_setpoint_pos.lat = (int32_t)lat;
    gps_setpoint_pos.lon = (int32_t)lon;
    gps_setpoint_pos.alt = (int32_t)alt;

    Location_Manager locman(system_log);
    locman.initialize_coordinate(autopilot_interface->current_messages.global_position_int, autopilot_interface->current_messages.local_position_ned);
    Mat result = locman.geodetic2NED(gps_setpoint_pos);

    set_position( result.at<float>(0), result.at<float>(1), result.at<float>(2), sp ); // [m, m, m, sp]
    autopilot_interface->update_setpoint(sp);
    printf("the next position is %f, %f, %f \n", result.at<float>(0), result.at<float>(1), result.at<float>(2));
    while (!IsInWaypointGlobal(autopilot_interface->current_messages.local_position_ned, gps_setpoint_pos, 0.5))
    {
        //printf("check IsInWaypoint? current x : %lf", autopilot_interface->current_messages.local_position_ned.x);
        system_log->write2txt("current position (after command to move) ", autopilot_interface->current_messages.local_position_ned);
        locman.initialize_coordinate(autopilot_interface->current_messages.global_position_int, autopilot_interface->current_messages.local_position_ned);
        result = locman.geodetic2NED(gps_setpoint_pos);
        set_position( result.at<float>(0), result.at<float>(1), result.at<float>(2), sp);
        autopilot_interface->update_setpoint(sp);
        usleep(50000);
    }
    printf("current x, y, z : %lf, %lf, %lf \n", autopilot_interface->current_messages.local_position_ned.x,
           autopilot_interface->current_messages.local_position_ned.y,
           autopilot_interface->current_messages.local_position_ned.z);
    printf("complete using GPS position move to %f, %f, %f \n", result.at<float>(0), result.at<float>(1), result.at<float>(2));
    return;
}

void Mavlink_Control::goto_velocity(float x, float y, float z, float speed, mavlink_set_position_target_local_ned_t &sp)
{
    mavlink_local_position_ned_t current_pos = autopilot_interface->current_messages.local_position_ned;

    float resX = speed * cos(atan2((y - current_pos.y), (x - current_pos.x)));
    float resY = speed * sin(atan2((y - current_pos.y), (x - current_pos.x)));
    float resZ = speed * sin(atan2((z - current_pos.z), (y - current_pos.y)));

    printf(" velocity input : %lf %lf %lf\n", resX, resY, resZ);
    mavlink_set_position_target_local_ned_t sp2;
    set_position( x, y, z, sp2 );
    set_velocity( resX, resY, resZ, sp);
    autopilot_interface->update_setpoint(sp);

    while (!IsInWaypointLocal(autopilot_interface->current_messages.local_position_ned, sp2, speed))
    {
        printf("current x, y, z : %lf, %lf, %lf \n", autopilot_interface->current_messages.local_position_ned.x,
               autopilot_interface->current_messages.local_position_ned.y,
               autopilot_interface->current_messages.local_position_ned.z);
        //printf("check IsInWaypoint? current x : %lf", (float)autopilot_interface->current_messages.local_position_ned.x);
        system_log->write2txt("current position (after command to move) ", autopilot_interface->current_messages.local_position_ned);
        current_pos = autopilot_interface->current_messages.local_position_ned;

        resX = speed * cos(atan2((y - current_pos.y), (x - current_pos.x)));
        resY = speed * sin(atan2((y - current_pos.y), (x - current_pos.x)));
        resZ = speed * sin(atan2((z - current_pos.z), (y - current_pos.y)));
        printf(" velocity input : %lf %lf %lf\n", resX, resY, resZ);
        set_velocity( resX, resY, resZ, sp);
        autopilot_interface->update_setpoint(sp);
        usleep(50000);
    }
    printf("current x, y, z : %lf, %lf, %lf \n", autopilot_interface->current_messages.local_position_ned.x,
           autopilot_interface->current_messages.local_position_ned.y,
           autopilot_interface->current_messages.local_position_ned.z);
    printf("complete using velocity move to %f, %f, %f \n", x, y, z);
    return;
}

void Mavlink_Control::hold_position(int sec, mavlink_set_position_target_local_ned_t &sp)
{
    set_velocity( 0.0, 0.0, 0.0, sp);
    autopilot_interface->update_setpoint(sp);
    sleep(sec);

    return;
}

void Mavlink_Control::yaw_rotation(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    set_yaw( yaw, sp);
    autopilot_interface->update_setpoint(sp);
    return;
}

void Mavlink_Control::yaw_rotation_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    set_yaw_rate( yaw_rate, sp); //rad/second
    autopilot_interface->update_setpoint(sp);
    return;
}

bool Mavlink_Control::IsInWaypointLocal(mavlink_local_position_ned_t current, mavlink_set_position_target_local_ned_t goal, float radius)
{
    // Radios is in meters
    float dx = goal.x - current.x;
    float dy = goal.y - current.y;
    float dz = goal.z - current.z;
    float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

    printf("distance to next position : %lf \n", distance);

    if(distance < radius)
    {printf("Is in Waypoint"); return true;}
    else
        return false;
}
bool Mavlink_Control::IsInWaypointGlobal(mavlink_local_position_ned_t current, mavlink_global_position_int_t goal, float radius)
{

    Location_Manager locman(system_log);
    locman.initialize_coordinate(autopilot_interface->current_messages.global_position_int, autopilot_interface->current_messages.local_position_ned);
    Mat result = locman.geodetic2NED(goal);

    float dx = result.at<float>(0) - current.x;
    float dy = result.at<float>(1) - current.y;
    float dz = result.at<float>(2) - current.z;
    float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

    if(distance < radius)
        return true;
    else
        return false;
}
bool Mavlink_Control::IsInWaypointGlobal(mavlink_global_position_int_t current, mavlink_set_position_target_local_ned_t goal, float radius)
{

    Mat result = location_manager->geodetic2NED(current);

    float dx = goal.x - result.at<float>(0);
    float dy = goal.y - result.at<float>(1);
    float dz = goal.z - result.at<float>(2);
    float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

    if(distance < radius)
        return true;
    else
        return false;
}

positiondata Mavlink_Control::getCurrentPose()
{
    positiondata current_pose;
    if (autopilot_interface == nullptr)
    {
        cout<< "autopilot_interface pointer is null" <<endl;
        return current_pose;
    }
    //cout<< "getCurrentPose from sutopilot_interface" <<endl;
    mavlink_gps_raw_int_t gps_raw_int = autopilot_interface->current_messages.gps_raw_int;
    current_pose.satellites_visible = gps_raw_int.satellites_visible;
    current_pose.hdop = gps_raw_int.eph;

    mavlink_local_position_ned_t local_pos = autopilot_interface->current_messages.local_position_ned;
    current_pose.x = local_pos.x;
    current_pose.y = local_pos.y;
    current_pose.z = local_pos.z;

    mavlink_global_position_int_t global_pos = autopilot_interface->current_messages.global_position_int;
    current_pose.lat = global_pos.lat;
    current_pose.lon = global_pos.lon;
    current_pose.alt = global_pos.alt;

    mavlink_attitude_t attitude = autopilot_interface->current_messages.attitude;
    current_pose.roll = attitude.roll;
    current_pose.pitch = attitude.pitch;
    current_pose.yaw = attitude.yaw;

//    mavlink_raw_imu_t raw_imu = autopilot_interface->current_messages.raw_imu;
//    current_pose.xacc = raw_imu.xacc;
//    current_pose.yacc = raw_imu.yacc;
//    current_pose.zacc = raw_imu.zacc;

//    mavlink_scaled_imu_t scaled_imu = autopilot_interface->current_messages.scaled_imu;
//    current_pose.xacc = scaled_imu.xacc;
//    current_pose.yacc = scaled_imu.yacc;
//    current_pose.zacc = scaled_imu.zacc;

    mavlink_highres_imu_t highres_imu = autopilot_interface->current_messages.highres_imu;
    current_pose.xacc = highres_imu.xacc;
    current_pose.yacc = highres_imu.yacc;
    current_pose.zacc = highres_imu.zacc;

    return current_pose;
}