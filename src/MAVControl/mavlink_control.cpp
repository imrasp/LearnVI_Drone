#include "MAVControl/mavlink_control.h"

Mavlink_Control::Mavlink_Control() {

}
Mavlink_Control::Mavlink_Control(int baudrate, char *&uart_name, System_Log *system_log_, Location_Manager *location_manager_, char *mission_route) : location_manager(location_manager_), system_log(system_log_)
{

    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
     serial_port = new Serial_Port(uart_name, baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
     autopilot_interface = new Autopilot_Interface(serial_port, system_log, location_manager);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = serial_port;
    autopilot_interface_quit = autopilot_interface;
    //signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port->start();
    autopilot_interface->start();
}
Mavlink_Control::~Mavlink_Control()
{}

void Mavlink_Control::start()
{
    commands();
}
void Mavlink_Control::stop()
{
    //   THREAD and PORT SHUTDOWN
    autopilot_interface->stop();
    serial_port->stop();
}

void Mavlink_Control::commands()
{
    printf("start command...");

    autopilot_interface->set_message_interval(105,500); // msg_id,interval in microseconds, HIGHRES_IMU = 105
    autopilot_interface->arm_control();
    usleep(100);

    autopilot_interface->enable_offboard_control();
    usleep(100);

    //wait until slam tracking is ready
//    cout << "location_manager->getSALMTrackingStage() : " << location_manager->getSALMTrackingStage() << endl;
//    while(location_manager->getSALMTrackingStage() != 2) {
//        usleep(100);
//    }

        printf("SEND OFFBOARD COMMANDS\n");
        mavlink_set_position_target_local_ned_t sp;
        mavlink_set_position_target_local_ned_t ip = autopilot_interface->initial_position;

    autopilot_interface->enable_takeoff(5.0,0.5);
    autopilot_interface->enable_land();
    cout << "landed" << endl;

//-----------------------------------------------------------------------------------------------------new/old

//    autopilot_interface->switchUpdatePosition(false);
//    syslog.write2csv("Update position = true");
//
//        takeoff(ip.z - 5.0, sp);
//    }


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
        sleep(60);
        autopilot_interface->disarm_control();
        usleep(100);

        autopilot_interface->disable_offboard_control();
        usleep(100);
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
        autopilot_interface_quit->handle_quit(sig);
        autopilot_interface->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
        serial_port->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);
}

void Mavlink_Control::setVisionEstimatedPosition(float x, float y, float z, float roll, float pitch, float yaw, float time){
    mavlink_vision_position_estimate_t vpe;
    vpe.x = x;
    vpe.y = y;
    vpe.z = z;
    vpe.roll = roll;
    vpe.pitch = pitch;
    vpe.yaw = yaw;
    vpe.usec = (uint32_t)time;

    autopilot_interface->updateVisionEstimationPosition(vpe);
}
