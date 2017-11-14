#include <iostream>

#include "MAVControl/mavlink_control.h"
#include "SLAMInterface/mono_live_viorb.h"
#include "SLAMInterface/mono_offline_viorb.h"
//#include "SLAMInterface/mono_record_viorb.h"

using namespace std;

int main(int argc, char **argv);

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, char *&vocabulary, char *&setting, char *&mode, char* &gui, char* &filename, int &timespace);

int main(int argc, char **argv) {
    try {
        // Default input arguments
#ifdef __APPLE__
        char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
        char *uart_name = (char *) "/dev/ttyUSB0";

#endif
        int baudrate = 921600; // 57600 or 921600 px4 companion link buadrate
        char *vocabulary = (char *) "../Vocabulary/ORBvoc.txt";
        char *setting = (char *) "../config/mobius.yaml";
        char *mode = (char *) "LIVE";
        char *gui = (char *) "ENABLE";
        char *filename = (char *) "Sample_data";
        int timespace = 1000000;
        char *mission = (char*) "./mission_route.txt";

        // do the parse, will throw an int if it fails
        parse_commandline(argc, argv, uart_name, baudrate, vocabulary, setting, mode, gui, filename, timespace);

        System_Log system_log(filename);

        cout << "Starting main in " << mode << " mode " << endl;
        if (string(mode) == "LIVE")
        {
            // if (std::string(getResult) == "something") to compare char need to make one to be string
            Mono_Live_VIORB mono_live_viorb(&system_log, string(gui)!="DISABLE");
            Location_Manager location_manager(&system_log,&mono_live_viorb,nullptr);
            mono_live_viorb.setLocationManager(&location_manager);
            Mavlink_Control mavlink_control(baudrate, uart_name, &system_log, &location_manager, mission);

            cout << "Start SLAM thread,..." << endl;
            mono_live_viorb.start(vocabulary, setting);
            location_manager.activateSLAM();
            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();

            //stop all thread in order
            mono_live_viorb.stop();
            mavlink_control.stop();
        }
        else if (string(mode) == "RECORD")
        {
            Mono_Record_VIORB mono_record_viorb(&system_log, true, nullptr, timespace);
            Location_Manager location_manager(&system_log,nullptr,&mono_record_viorb);
            Mavlink_Control mavlink_control(baudrate, uart_name, &system_log, &location_manager, mission);

            cout << "Start Record SLAM thread,..." << endl;
            mono_record_viorb.start(filename);
            location_manager.activateSLAM();
            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();

            //stop all thread in order
            mono_record_viorb.stop();
            mavlink_control.stop();

        }
        else if (string(mode) == "LIVERECORD")
        {
            // if (std::string(getResult) == "something") to compare char need to make one to be string
            Mono_Live_VIORB mono_live_viorb(&system_log, string(gui)!="DISABLE");
            Mono_Record_VIORB mono_record_viorb(&system_log, false, &mono_live_viorb, timespace);
            Location_Manager location_manager(&system_log,&mono_live_viorb,&mono_record_viorb);
            mono_live_viorb.setLocationManager(&location_manager);
            Mavlink_Control mavlink_control(baudrate, uart_name, &system_log, &location_manager, mission);
            location_manager.setMavlinkControl(&mavlink_control);

            cout << "Start SLAM thread,..." << endl;
            mono_live_viorb.start(vocabulary, setting);
            mono_record_viorb.start(filename);
            location_manager.activateSLAM();
            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();
//
//            //stop all thread in order
            mono_live_viorb.stop();
            mono_record_viorb.stop();
            mavlink_control.stop();
        }
        else if (string(mode) == "MAVONLY")
        {
            Location_Manager location_manager(&system_log,nullptr,nullptr);
            Mavlink_Control mavlink_control(baudrate, uart_name, &system_log, &location_manager, mission);
            location_manager.setMavlinkControl(&mavlink_control);

            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();
            mavlink_control.stop();
        }
        else if (string(mode) == "OFFLINE")
        {
            Mono_Offline_VIORB mono_live_viorb(&system_log);

            cout << "Start SLAM thread,..." << endl;
            mono_live_viorb.start(vocabulary, setting, filename);
        }


        else cout << "This mode is not implemented yet" << endl;
        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}

//   Parse Command Line
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, char *&vocabulary, char *&setting, char *&mode, char* &gui, char* &filename, int &timespace) {
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -v <path_to_vocabulary> -s <path_to_setting>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path to vocabulary
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--vocab") == 0) {
            if (argc > i + 1) {
                vocabulary = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path to setting
        if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--setting") == 0) {
            if (argc > i + 1) {
                setting = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Mode
        if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
            if (argc > i + 1) {
                mode = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Mode
        if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--gui") == 0) {
            if (argc > i + 1) {
                gui = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
        // Filename
        if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--foldername") == 0) {
            if (argc > i + 1) {
                filename = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Time space between frame
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--time") == 0) {
            if (argc > i + 1) {
                timespace = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    return;
}