#include "MAVControl/mavlink_control.h"

Mavlink_Control::Mavlink_Control() {

}

Mavlink_Control::Mavlink_Control(int baudrate, char *&uart_name, System_Log *system_log_,
                                 Location_Manager *location_manager_, char *mission_route_) : location_manager(
        location_manager_), system_log(system_log_), mission_route(mission_route_) {

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
    serial_port_quit = serial_port;
    autopilot_interface_quit = autopilot_interface;
    //signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port->start();
    autopilot_interface->start();
}

Mavlink_Control::~Mavlink_Control() {}

void Mavlink_Control::start() {
    commands();
}

void Mavlink_Control::stop() {
    //   THREAD and PORT SHUTDOWN
    autopilot_interface->stop();
    serial_port->stop();
}

void Mavlink_Control::commands() {

    printf("start command...");

    //autopilot_interface->set_message_interval(105,500); // msg_id,interval in microseconds, HIGHRES_IMU = 105
    autopilot_interface->enable_offboard_control(); usleep(100);

    ifstream input(mission_route);
    string line, temp;
    double param1, param2;

    if (input.is_open()) {
        while (getline(input,line)) {
            cout << line << endl;
            stringstream s (line);
            int i = 0;
            int mode = 0; // 1:hold, 2:gotoned

            // example : goto 9.0 8.0 7.0
            //          goto > i == 0
            //          9.0 > i == 1
            //          8.0 > i == 2
            //          7.0 > i == 3
            while(s>> temp) {
                cout << i << " : mode " << mode << " : " << temp << endl;
                if ( i == 0 && temp == "takeoff" ){
                    autopilot_interface->enable_takeoff(10, 0.5);
                } else if ( i == 0 && temp == "land" ){
                    autopilot_interface->enable_land();
                } else if ( i == 0 && temp == "hold" ){
                    mode = 1; i++;
                } else if ( i == 0 && temp == "gotoned" ){
                    mode = 2; i++;
                } else if ( i != 0 ) { // hold and goto
                    if( i == 1 && mode == 1 ){
                        autopilot_interface->enable_hold(stod(temp));
                    } else if ( i == 1 && mode == 2 ){
                        param1 = stod(temp); i++;
                    } else if (i != 1 ){
                        if( i == 2 ) {
                            param2 = stod(temp); i++;
                        } else if( i == 3 && mode == 2 ){
                            //autopilot_interface->goto_position_ned(param1,param2,stod(temp));
                        }
                    }
                }
            }
        }
    } else
        cout << "ERROR: Cannot Open  File" << '\n';

    sleep(60);

    autopilot_interface->disable_offboard_control();
    usleep(100);

    printf("end offboard command \n");

    return;

}

//   Quit Signal Handler
// this function is called when you press Ctrl-C
void Mavlink_Control::quit_handler(int sig) {
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
        autopilot_interface->handle_quit(sig);
    }
    catch (int error) {}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
        serial_port->handle_quit(sig);
    }
    catch (int error) {}

    // end program here
    exit(0);
}

void
Mavlink_Control::setVisionEstimatedPosition(float x, float y, float z, float roll, float pitch, float yaw, float time) {
    mavlink_vision_position_estimate_t vpe;
    vpe.x = x;
    vpe.y = y;
    vpe.z = z;
    vpe.roll = roll;
    vpe.pitch = pitch;
    vpe.yaw = yaw;
    vpe.usec = (uint32_t) time;

    autopilot_interface->updateVisionEstimationPosition(vpe);
}
