#include "MAVControl/autopilot_interface.h"

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

//   Con/De structors
Autopilot_Interface::Autopilot_Interface()
{}

Autopilot_Interface::Autopilot_Interface(Serial_Port *serial_port_, System_Log *system_log_, Location_Manager *location_manager_)
        : serial_port(serial_port_), system_log(system_log_), location_manager(location_manager_)
{
    // initialize attributes
    write_count = 0;

    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    takeoff_status = 0;
    land_status = 0;
    arm_status = 0;
    home_status = 0;
    time_to_exit   = false;  // flag to signal thread exit

    read_tid  = 0; // read thread id
    write_tid = 0; // write thread id

    system_id    = 0; // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id

    init_coord_conversion = 0;
    bUpdatePosition = false;

    imu_status = UNINITIAL_IMU;

    current_messages.sysid  = system_id;
    current_messages.compid = autopilot_id;

    //    serial_port = serial_port_; // serial port management object
    //    system_log = system_log_; // system log management object
    //    location_manager = location_manager_; // location manager management object
}

Autopilot_Interface::~Autopilot_Interface()
{
}

//   Update Setpoint
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{  current_setpoint = setpoint;  }

// ----------------------------------------------------------------------------------
//   Time
// ----------------------------------------------------------------------------------
uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

//   STARTUP
void Autopilot_Interface::start()
{
    int result;

    //   CHECK SERIAL PORT

    if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
    {
        fprintf(stderr,"ERROR: serial port not open\n");
        throw 1;
    }

    //   READ THREAD

    printf("START READ THREAD \n");

    result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    if ( result ) throw result;

    // now we're reading messages
    printf("\n");

    //   CHECK FOR MESSAGES

    printf("CHECK FOR MESSAGES\n");

    while ( not current_messages.sysid )
    {
        if ( time_to_exit )
            return;
        usleep(500000); // check at 2Hz
    }

    printf("Found\n");

    // now we know autopilot is sending messages
    printf("\n");

    //   GET SYSTEM and COMPONENT IDs

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if ( not system_id )
    {
        system_id = current_messages.sysid;
        printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
    }

    // Component ID
    if ( not autopilot_id )
    {
        autopilot_id = current_messages.compid;
        printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
        printf("\n");
    }

    //   GET INITIAL POSITION

    // Wait for initial position ned
    while ( not ( current_messages.time_stamps.local_position_ned &&
                  current_messages.time_stamps.attitude            )  )
    {
        if ( time_to_exit )
            return;
        usleep(500000);
    }

    // copy initial position ned
    Mavlink_Messages local_data = current_messages;
    initial_position.x        = local_data.local_position_ned.x;
    initial_position.y        = local_data.local_position_ned.y;
    initial_position.z        = local_data.local_position_ned.z;
    initial_position.vx       = local_data.local_position_ned.vx;
    initial_position.vy       = local_data.local_position_ned.vy;
    initial_position.vz       = local_data.local_position_ned.vz;
    initial_position.yaw      = local_data.attitude.yaw;
    initial_position.yaw_rate = local_data.attitude.yawspeed;

    printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
    printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    printf("\n");

    // we need this before starting the write thread

    //   WRITE THREAD
    printf("START WRITE THREAD \n");

    result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
    if ( result ) throw result;

    // wait for it to be started
    while ( not writing_status )
        usleep(100000); // 10Hz

    // now we're streaming setpoint commands
    printf("\n");


    // Done!
    return;

}

//   SHUTDOWN
void Autopilot_Interface::stop()
{
    //   CLOSE THREADS
    printf("CLOSE THREADS\n");

    // signal exit
    time_to_exit = true;

    // wait for exit
    pthread_join(read_tid ,NULL);
    pthread_join(write_tid,NULL);

    // now the read and write threads are closed
    printf("\n");

    // still need to close the serial_port separately
}

//   Read Thread
void Autopilot_Interface::start_read_thread()
{
    if ( reading_status != 0 )
    {
        fprintf(stderr,"read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }
}

//   Write Thread
void Autopilot_Interface::start_write_thread(void)
{
    if ( not writing_status == false )
    {
        fprintf(stderr,"write thread already running\n");
        return;
    }
    else
    {
        write_thread();
        return;
    }
}

//   Quit Handler
void Autopilot_Interface::handle_quit( int sig )
{
    disable_offboard_control();

    try {
        stop();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop autopilot interface\n");
    }
}

//   Read Thread
void Autopilot_Interface::read_thread()
{
    reading_status = true;

    while ( ! time_to_exit )
    {
        read_messages();
        usleep(100000); // Read batches at 10Hz
    }

    reading_status = false;

    return;
}

//   Write Thread
void Autopilot_Interface::write_thread(void)
{
    // signal startup
    writing_status = 2;

    // prepare an initial setpoint, just stay put
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
                   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx       = 0.0;
    sp.vy       = 0.0;
    sp.vz       = 0.0;
    sp.yaw_rate = 0.0;

    // set position target
    current_setpoint = sp;

    // write a message and signal writing
    write_setpoint();
    writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while ( !time_to_exit )
    {
        usleep(250000);   // Stream at 4Hz
        write_setpoint();
    }

    // signal end
    writing_status = false;

    return;
}

// End Autopilot_Interface


//  Pthread Starter Helper Functions
void* start_autopilot_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

void* start_autopilot_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_write_thread();

    // done!
    return NULL;
}

//   Read Messages
void Autopilot_Interface::read_messages()
{
    bool success;               // receive success flag
    bool received_all = false;  // receive only one message
    Time_Stamps this_timestamps;

    // Blocking wait for new data
    while ( !received_all and !time_to_exit )
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        success = serial_port->read_message(message);

        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if( success )
        {

            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid  = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {

                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                    current_messages.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED \n");
                    mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                    current_messages.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;

                    system_log->write2csv("ned", current_messages.local_position_ned);
                    location_manager->poseToSLAM(current_messages.local_position_ned);
                    //system_log->write2txt("ned", current_messages.local_position_ned);
                    if (imu_status == UNINITIAL_IMU)
                    {
                        imu_status = INITIAL_IMU;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("\n MAVLINK_MSG_ID_GLOBAL_POSITION_INT \n");
                    mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                    current_messages.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;

                    system_log->write2csv("int", current_messages.global_position_int);
                    location_manager->poseToSLAM(current_messages.global_position_int);

                    // set initial params for coordinate conversion
                    if(init_coord_conversion == false && imu_status == INITIAL_IMU )
                    {
                        //printf("SET INITIAL COORDINATE CONVERSION... \n");
                        system_log->write2csv("initial coordinate conversion");
                        init_coord_conversion = true;
                        location_manager->initialize_coordinate(current_messages.global_position_int, current_messages.local_position_ned);
                    }
                    else if(init_coord_conversion == true)
                    {
//                        update_geodetic2local(current_messages.global_position_int);
                        if(bUpdatePosition){
                            mavlink_vision_position_estimate_t vpe;
                            vpe.x = location_manager->current_pose.gpsx;
                            vpe.y = location_manager->current_pose.gpsy;
                            vpe.z = location_manager->current_pose.gpsz;
                            vpe.yaw = ( ((float)current_messages.global_position_int.hdg/100) * M_PI ) / 180;

                            write_vision_position(vpe);
                        }
                    }

                    break;
                }

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    //printf("MAVLINK_MSG_ID_GPS_STATUS\n");
                    mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
                    current_messages.time_stamps.gps_raw_int = get_time_usec();
                    this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;

                    location_manager->poseToSLAM(current_messages.gps_raw_int);
                    system_log->write2csv("raw", current_messages.gps_raw_int);

                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                    current_messages.time_stamps.sys_status = get_time_usec();
                    this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                    current_messages.time_stamps.battery_status = get_time_usec();
                    this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                    current_messages.time_stamps.radio_status = get_time_usec();
                    this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                    current_messages.time_stamps.position_target_local_ned = get_time_usec();
                    this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                    current_messages.time_stamps.position_target_global_int = get_time_usec();
                    this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                    current_messages.time_stamps.highres_imu = get_time_usec();
                    this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;

                    system_log->write2csv("HIGHRES_IMU", current_messages.highres_imu);
                    location_manager->poseToSLAM(current_messages.highres_imu);

                    break;
                }

                case MAVLINK_MSG_ID_RAW_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_raw_imu_decode(&message, &(current_messages.raw_imu));
                    current_messages.time_stamps.raw_imu = get_time_usec();
                    this_timestamps.raw_imu = current_messages.time_stamps.raw_imu;

                    system_log->write2csv("Raw_IMU", current_messages.raw_imu);

                    break;

                }case MAVLINK_MSG_ID_SCALED_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_scaled_imu_decode(&message, &(current_messages.scaled_imu));
                    current_messages.time_stamps.scaled_imu = get_time_usec();
                    this_timestamps.scaled_imu = current_messages.time_stamps.scaled_imu;

                    //system_log->write2csv("Scaled_IMU", current_messages.scaled_imu);

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                    current_messages.time_stamps.attitude = get_time_usec();
                    this_timestamps.attitude = current_messages.time_stamps.attitude;

                    system_log->write2csv("Attitude", current_messages.attitude);

                    break;
                }

                case MAVLINK_MSG_ID_HOME_POSITION:
                {
                    //printf("MAVLINK_MSG_ID_HOME_POSITION\n");
                    mavlink_msg_home_position_decode(&message, &(current_messages.home_position));
                    current_messages.time_stamps.home_position = get_time_usec();
                    this_timestamps.home_position = current_messages.time_stamps.home_position;
                    break;
                }

                    //case MAVLINK_MSG_ID_GPS_INPUT:
                case 232:
                {
                    //printf("MAVLINK_MSG_ID_GPS_INPUT\n");
                    mavlink_msg_gps_input_decode(&message, &(current_messages.gps_input));
                    current_messages.time_stamps.gps_input = get_time_usec();
                    this_timestamps.gps_input = current_messages.time_stamps.gps_input;

                    break;
                }

                case MAVLINK_MSG_ID_GPS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_GPS_STATUS\n");
                    mavlink_msg_gps_status_decode(&message, &(current_messages.gps_status));
                    current_messages.time_stamps.gps_status = get_time_usec();
                    this_timestamps.gps_status = current_messages.time_stamps.gps_status;

                    break;
                }

                    //ALTITUDE
                case MAVLINK_MSG_ID_ALTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ALTITUDE\n");
                    mavlink_msg_altitude_decode(&message, &(current_messages.altitude));
                    current_messages.time_stamps.altitude = get_time_usec();
                    this_timestamps.altitude = current_messages.time_stamps.altitude;

                    //printf("Altitude : %f \n", current_messages.altitude.altitude_local);
                    break;
                }

                default:
                {
                    // printf("Warning, did not handle message id %i\n",message.msgid);
                    break;
                }


            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        received_all =
                this_timestamps.heartbeat                  &&
                //				this_timestamps.battery_status             &&
                //				this_timestamps.radio_status               &&
                //				this_timestamps.local_position_ned         &&
                //				this_timestamps.global_position_int        &&
                //				this_timestamps.position_target_local_ned  &&
                //				this_timestamps.position_target_global_int &&
                //				this_timestamps.highres_imu                &&
                //				this_timestamps.attitude                   &&
                this_timestamps.sys_status                 //&&
            //this_timestamps.gps_input                  &&
            //this_timestamps.gps_status
                ;

        // give the write thread time to use the port
        if ( writing_status > false ) {
            usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all

    return;
}

//   Write Message
int Autopilot_Interface::write_message(mavlink_message_t message)
{
    // do the write
    int len = serial_port->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

//   Write Setpoint Message
void Autopilot_Interface::write_setpoint()
{
    //   PACK PAYLOAD
    // pull from position target
    mavlink_set_position_target_local_ned_t sp = current_setpoint;

    // double check some system parameters
    if ( not sp.time_boot_ms )
        sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    sp.target_system    = system_id;
    sp.target_component = autopilot_id;

    //   ENCODE
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


    //   WRITE
    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    //else
    //printf("Write position target \n");
    //printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    return;
}

void Autopilot_Interface::switchUpdatePosition(bool input)
{
    //default is false
    bUpdatePosition = input;
}
void Autopilot_Interface::update_geodetic2local(mavlink_gps_raw_int_t raw_pos)
{
    mavlink_global_position_int_t gps_pos;
    gps_pos.lat = raw_pos.lat;
    gps_pos.lon = raw_pos.lon;
    gps_pos.alt = raw_pos.alt;

    update_geodetic2local(gps_pos);
}
void Autopilot_Interface::update_geodetic2local(mavlink_global_position_int_t gps_pos)
{
    Mat result = location_manager->geodetic2NED(gps_pos);

    //calculate percent error
    mavlink_local_position_ned_t current_ned = current_messages.local_position_ned;
    float errX = 100 * abs((current_ned.x - result.at<float>(0)) / result.at<float>(0));
    float errY = 100 * abs((current_ned.y - result.at<float>(1)) / result.at<float>(1));
    float errZ = 100 * abs((current_ned.z - result.at<float>(2)) / result.at<float>(2));
    float errAvg = (errX + errY + errZ) / 3;

    //system_log->write2csv("GPS2NED ", result);
    system_log->write2csv("GPS2NED with %Err ", result, current_ned, errX, errY, errZ, errAvg);

    mavlink_vision_position_estimate_t vpe;
    vpe.x = result.at<float>(0);
    vpe.y = result.at<float>(1);
    vpe.z = result.at<float>(2);
    vpe.yaw = ( ((float)gps_pos.hdg/100) * M_PI ) / 180;

    if (bUpdatePosition)
        write_vision_position(vpe);


}

void Autopilot_Interface::write_vision_position(mavlink_vision_position_estimate_t &vision_position_estimate_)
{
    mavlink_vision_position_estimate_t vpe = vision_position_estimate_;
    // double check some system parameters
    if ( not vpe.usec )
        vpe.usec = (uint32_t) (get_time_usec()/1000000);

    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(system_id, companion_id, &message, &vpe);

    int len = write_message(message);

    if ( !len )
    {
        //printf("cannot write to VISION_POSITION_ESTIMATE");
        system_log->write2txt("cannot write to VISION_POSITION_ESTIMATE");
    }
    else
        system_log->write2txt("write to VISION_POSITION_ESTIMATE");
}

//   Start Off-Board Mode
void Autopilot_Interface::enable_offboard_control()
{
    // Should only send this command once
    if ( control_status == false )
    {
        printf("ENABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to go off-board
        int success = toggle_offboard_control( true );

        // Check the command was written
        if ( success )
            control_status = true;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if not offboard_status

}

//   Stop Off-Board Mode
void Autopilot_Interface::disable_offboard_control()
{

    // Should only send this command once
    if ( control_status == true )
    {
        printf("DISABLE OFFBOARD MODE\n");

        //   TOGGLE OFF-BOARD MODE

        // Sends the command to stop off-board
        int success = toggle_offboard_control( false );

        // Check the command was written
        if ( success )
            control_status = false;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if offboard_status

}

//   Toggle Off-Board Mode
int Autopilot_Interface::toggle_offboard_control( bool flag )
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0 };
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
    //com.command          = MAV_CMD_NAV_GUIDED_ENABLE | MAV_MODE_GUIDED_ARMED;

    // https://pixhawk.ethz.ch/mavlink/
    // MAV_MODE
    // These defines are predefined OR-combined mode flags.
    // There is no need to use values from this enum, but it simplifies the use of the mode flags.
    // Note that manual input is enabled in all modes as a safety override.

    com.confirmation     = true;
    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = serial_port->write_message(message);

    // Done!
    return len;
}

//   Start Arming
void Autopilot_Interface::arm_control()
{
    // Should only send this command once
    if ( arm_status == false )
    {
        printf("ARMIMG...\n");

        // ----------------------------------------------------------------------
        //   TOGGLE ARMING STAGE
        // ----------------------------------------------------------------------

        // Sends the command to arm
        int success = toggle_arm_control( true );

        // Check the command was written
        if ( success )
            arm_status = true;
        else
        {
            fprintf(stderr,"Error: Unable to arm, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if arm
}

void Autopilot_Interface::disarm_control()
{
    // Should only send this command once
    if ( arm_status == true )
    {
        printf("DISARMIMG...\n");

        // ----------------------------------------------------------------------
        //   TOGGLE ARMING STAGE
        // ----------------------------------------------------------------------

        // Sends the command to arm
        int success = toggle_arm_control( false );

        // Check the command was written
        if ( success )
            arm_status = false;
        else
        {
            fprintf(stderr,"Error: Unable to disarm, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");
    }
}

//   Toggle Arm State
int Autopilot_Interface::toggle_arm_control( bool flag )
{
    // Prepare command for off-board mode
    mavlink_command_long_t com;
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    //com.command          = MAV_MODE_FLAG_SAFETY_ARMED;
    com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = 0;
    com.param1 = flag ? 1.0f : 0.0f;


    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = serial_port->write_message(message);

    // Done!
    return len;
}

//   Set Home Position
void Autopilot_Interface::set_home()
{

    // Should only send this command once
    if ( home_status == false )
    {
        printf("SETTING HOME POSITION...\n");

        //   TOGGLE SET HOME
        mavlink_command_long_t com;
        com.target_system    = system_id;
        com.target_component = autopilot_id;
        com.command          = 	MAV_CMD_DO_SET_HOME;
        com.confirmation = 0;
        com.param1 = 1;


        // Encode
        mavlink_message_t message;
        mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

        // Send the message
        int len = serial_port->write_message(message);

        // Check the command was written
        if ( len )
        {
            home_status = true;

            // CHECK HOME POSITION
            mavlink_home_position_t home = current_messages.home_position;
            printf("\n----------------------------------------------------------------\n"
                           "1 Latitude (WGS84), in degrees, Longitude (WGS84, in degrees,  Altitude (AMSL), in meters * 1000 \n"
                           "2 : this position in the local coordinate frame "
                           "\n----------------------------------------------------------------\n");
            printf("1 CURRENT HOME POSITION - after set home (lat,lon,alt) = [ %f , %f, %f ] \n", (float)home.latitude, (float)home.longitude, (float)home.altitude);
            printf("2 CURRENT HOME POSITION - after set home (x,y,z) = [ %f , %f, %f ] \n", home.x, home.y, home.z);
        }
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    } // end: if offboard_status

}

// ----------------------------------------------------------------------------------
// Setpoint Helper Functions
// choose one of the next three
/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

void set_takeoff_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{	// this command will condider mainly on z value
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("POSITION TAKEOFF SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

void set_land_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{	// this command will ignore z value and focus on x and y only
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("POSITION LAND SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}



/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx  = vx;
    sp.vy  = vy;
    sp.vz  = vz;

    //printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}
/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

    // NOT IMPLEMENTED
    fprintf(stderr,"set_acceleration doesn't work yet \n");
    throw 1;


    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.afx  = ax;
    sp.afy  = ay;
    sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

    sp.yaw  = yaw;

    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

    sp.yaw_rate  = yaw_rate;
}




