/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "MAVControl/autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x   = x;
    sp.y   = y;
    sp.z   = z;

    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

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
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

    // NOT IMPLEMENTED
    fprintf(stderr,"set_acceleration doesn't work yet \n");
    throw 1;


    sp.type_mask =
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

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
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

    sp.yaw  = yaw;

    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
            MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

    sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_, System_Log *system_log_, Location_Manager *location_manager_)
        : serial_port(serial_port_), system_log(system_log_), location_manager(location_manager_)
{
    // initialize attributes
    write_count = 0;

    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit   = false;  // flag to signal thread exit

    read_tid  = 0; // read thread id
    write_tid = 0; // write thread id

    system_id    = 0; // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id

    current_messages.sysid  = system_id;
    current_messages.compid = autopilot_id;

    serial_port = serial_port_; // serial port management object

    arm_status = 0;
    home_status = 0;

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
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

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                    current_messages.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;

                    location_manager->setPose(current_messages.local_position_ned);

                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                    current_messages.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;

                    location_manager->setPose(current_messages.global_position_int);

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

                    location_manager->setPose(current_messages.highres_imu);

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                    current_messages.time_stamps.attitude = get_time_usec();
                    this_timestamps.attitude = current_messages.time_stamps.attitude;

                    location_manager->setPose(current_messages.attitude);

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

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    //printf("MAVLINK_MSG_ID_HOME_POSITION\n");
                    mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
                    current_messages.time_stamps.gps_raw_int = get_time_usec();
                    this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;
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
                this_timestamps.sys_status
                ;

        // give the write thread time to use the port
        if ( writing_status > false ) {
            //usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all

    return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
    // do the write
    int len = serial_port->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // pull from position target
    mavlink_set_position_target_local_ned_t sp = current_setpoint;

    // double check some system parameters
    if ( not sp.time_boot_ms )
        sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    sp.target_system    = system_id;
    sp.target_component = autopilot_id;


    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if ( len <= 0 )
        fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

    return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
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
        cout << "current_messages.heartbeat.base_mode : " << current_messages.heartbeat.base_mode << endl;
//        while(current_messages.heartbeat.base_mode != MAV_MODE_FLAG_GUIDED_ENABLED){
//            sleep(0.1);
//        }
        printf("\n");

    } // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

    // Should only send this command once
    if ( control_status == true )
    {
        printf("DISABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

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

//        while(current_messages.heartbeat.base_mode == MAV_MODE_FLAG_GUIDED_ENABLED){
//            sleep(0.1);
//        }

        printf("\n");

    } // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0 };
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
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


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
    int result, result2;

    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------

    if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
    {
        fprintf(stderr,"ERROR: serial port not open\n");
        throw 1;
    }


    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    printf("START READ THREAD \n");

    result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    if ( result ) throw result;

    // now we're reading messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

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


    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

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


    // --------------------------------------------------------------------------
    //   GET INITIAL POSITION
    // --------------------------------------------------------------------------

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

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
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


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
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

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
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


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
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


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

    disable_offboard_control();

    try {
        stop();

    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop autopilot interface\n");
    }

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
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


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
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


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    // run the object's read thread
    autopilot_interface->start_write_thread();

    // done!
    return NULL;
}


// ----------------------------------------------------------------------
// ARM AND DISARM CONTROL
// ----------------------------------------------------------------------
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
        sleep(1);
        // Check the command was written
        if ( success )
            arm_status = true;
        else
        {
            fprintf(stderr,"Error: Unable to arm, could not write message\n");
            //throw EXIT_FAILURE;
        }

        cout << "current_messages.heartbeat.base_mode : " << current_messages.heartbeat.base_mode << endl;
//        while(current_messages.heartbeat.base_mode != MAV_MODE_FLAG_SAFETY_ARMED){
//            sleep(0.1);
//        }


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
        sleep(1);
        // Check the command was written
        if ( success )
            arm_status = false;
        else
        {
            fprintf(stderr,"Error: Unable to disarm, could not write message\n");
            //throw EXIT_FAILURE;
        }
//        while(current_messages.heartbeat.base_mode == MAV_MODE_FLAG_SAFETY_ARMED){
//            sleep(0.1);
//        }

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

// ------------------------------------------------------------------------------
//SET AND UPDATE SETPOINT
// ------------------------------------------------------------------------------
void Autopilot_Interface::enable_takeoff(float height,float velocity)
{
    printf("Mode TAKEOFF\n");
    float precision_distance = 0.1; // [m]
    mavlink_set_position_target_local_ned_t sp_target;
//    sp_target.vx = 0;
//    sp_target.vy = 0;
//    sp_target.vz = -velocity;
    sp_target.z = -height;
    sp_target.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
    sp_target.coordinate_frame = MAV_FRAME_LOCAL_NED;

    update_setpoint(sp_target);

    //wait message to update
    while(-height != current_messages.position_target_local_ned.z){
        cout << "want h = " << -height << " current expected h " << current_messages.position_target_local_ned.z << endl;
        sleep(0.1);
    }

    while((current_messages.extended_sys_state.landed_state != MAV_LANDED_STATE_IN_AIR)
          && ( fabs(current_messages.local_position_ned.z - current_messages.position_target_local_ned.z) < precision_distance ))
    {
        cout << "current h is " << current_messages.local_position_ned.z << " expect " << current_messages.position_target_local_ned.z << endl;
        sleep(0.1);
    }
    mavlink_local_position_ned_t cp = current_messages.local_position_ned;
    cout << "takeoff complete! current position is " << cp.x << " , " << cp.y << " , " << cp.z  << endl;
}

void Autopilot_Interface::enable_land()
{
    printf("Mode land\n");
    mavlink_set_position_target_local_ned_t setpoint;
//    setpoint.vx = 0;
//    setpoint.vy = 0;
//    setpoint.vz = 0.5;
    setpoint.z = 0.00;
    setpoint.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;
    setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;

    update_setpoint(setpoint);

    //wait message to update
    while(0.0 != current_messages.position_target_local_ned.z){
        sleep(0.1);
    }
    while(current_messages.extended_sys_state.landed_state != MAV_LANDED_STATE_ON_GROUND || current_messages.local_position_ned.z < 0.005)
//    while(current_messages.local_position_ned.z <= -0.05)
    {
        cout << " current landing z is " << current_messages.local_position_ned.z << endl;
        sleep(0.1);
    }
    cout << "landed complete!\n" << endl;
}

void Autopilot_Interface::enable_hold(double sec)
{
    printf("Mode Hold Position\n");
    mavlink_set_position_target_local_ned_t setpoint;
    setpoint.vx = 0;
    setpoint.vy = 0;
    setpoint.vz = 0;
    setpoint.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ALT_HOLD;
    setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;

    update_setpoint(setpoint);

    //wait message to update
    while(0.0 != current_messages.position_target_local_ned.vx || 0.0 != current_messages.position_target_local_ned.vy || 0.0 != current_messages.position_target_local_ned.vz){
        sleep(0.1);
    }
    sleep(sec);
    cout << "Time up for holding!\n";
}

void Autopilot_Interface::goto_positon_ned(float x, float y, float z){

    printf("Goto Position\n");
    mavlink_set_position_target_local_ned_t setpoint;
    mavlink_local_position_ned_t cp = current_messages.local_position_ned;

    setpoint.x = cp.x + x;
    setpoint.y = cp.y + y;
    setpoint.z = cp.z + z;
    setpoint.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;

    update_setpoint(setpoint);
    cout << "current position : " << cp.x << " , " << cp.y << " , " << cp.z << " expected " << cp.x+x << " , " << cp.y + y << " , " << cp.z + z << endl;
    //wait message to update
    while( cp.x+x != current_messages.position_target_local_ned.x && cp.y+y != current_messages.position_target_local_ned.y && cp.z+z != current_messages.position_target_local_ned.z){
        sleep(0.1);
    }

    while(!IsInWaypointLocal(0.5)){
        //cout << "current x is " << current_messages.local_position_ned.x << " expect " << current_messages.position_target_local_ned.x << endl;
        sleep(0.1);
    }
    cout << "reached! \n";
}

void Autopilot_Interface::goto_positon_offset_ned(float x, float y, float z){

    printf("Goto Position\n");
    mavlink_set_position_target_local_ned_t setpoint;
    mavlink_local_position_ned_t cp = current_messages.local_position_ned;

    setpoint.x = x;
    setpoint.y = y;
    setpoint.z = z;
    setpoint.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    setpoint.coordinate_frame = MAV_FRAME_LOCAL_OFFSET_NED;

    update_setpoint(setpoint);
    cout << "current position : " << cp.x << " , " << cp.y << " , " << cp.z << " expected " << cp.x+x << " , " << cp.y + y << " , " << cp.z + z << endl;
    //wait message to update
//    while( cp.x+x != current_messages.position_target_local_ned.x && cp.y+y != current_messages.position_target_local_ned.y && cp.z+z != current_messages.position_target_local_ned.z){
        sleep(1);
//    }

    while(!IsInWaypointLocal(0.5)){
        //cout << "current x is " << current_messages.local_position_ned.x << " expect " << current_messages.position_target_local_ned.x << endl;
        sleep(0.1);
    }
    cout << "reached! \n";
}

// Request MSG streaming rate
// param 1 = message ID and param 2 = interval in microseconds
void Autopilot_Interface::set_message_interval( int msg_id, int hz )
{
    mavlink_command_long_t com;
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    com.command          = 	MAV_CMD_SET_MESSAGE_INTERVAL;
    com.confirmation = 0;
    com.param1 = MAVLINK_MSG_ID_HIGHRES_IMU;
    com.param2 = hz;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = serial_port->write_message(message);

    // Check the command was written
    if (!len){
        fprintf(stderr,"Error: Unable to set_message_interval, could not write message\n");
        throw EXIT_FAILURE;
    }
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
        }
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }

        printf("\n");

    }
}

void Autopilot_Interface::updateVisionEstimationPosition(mavlink_vision_position_estimate_t vpe)
{
    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(system_id, companion_id, &message, &vpe);

    int len = write_message(message);

    if ( !len )
        cout << "cannot write to VISION_POSITION_ESTIMATE \n";
    else
        cout << "write to VISION_POSITION_ESTIMATE \n";
}

bool Autopilot_Interface::IsInWaypointLocal(float radius)
{
    // Radios is in meters
    float dx = current_messages.position_target_local_ned.x - current_messages.local_position_ned.x;
    float dy = current_messages.position_target_local_ned.y - current_messages.local_position_ned.y;
    float dz = current_messages.position_target_local_ned.z - current_messages.local_position_ned.z;
    float distance = sqrtf(pow(dx,2) + pow(dy,2) + pow(dz,2));

    //printf("distance to next position : %lf \n", distance);

    if(distance < radius) {
        //printf("Is in Waypoint");
        return true;
    }
    else
        return false;
}