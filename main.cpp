#include <iostream>
#include <thread>
#include <boost/thread.hpp>

#include "src/mavlink_control.h"
#include "src/cameraCapture.h"
#include "src/mono_live_viorb.h"
#include "src/mono_offline_viorb.h"
//#include "src/live_slam.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"
using namespace std;

int main(int argc, char **argv);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, char *&vocabulary, char *&setting);

int main(int argc, char **argv)
{
    try {
        // Default input arguments
#ifdef __APPLE__
        char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
        char *uart_name = (char *) "/dev/ttyUSB0";

#endif
        int baudrate = 57600;
        char *vocabulary = (char *) "./Vocabulary/ORBvoc.txt";
        char *setting = (char *) "./camera_calib/mobius.yaml";

        // do the parse, will throw an int if it fails
        parse_commandline(argc, argv, uart_name, baudrate, vocabulary, setting);

        cout << "start main..." << endl;
        boost::mutex mutex;

//        Mavlink_Control mavconn(baudrate, uart_name);
//        Camera_Capture camera_capture(&mutex, &mavconn);
//        Mono_Live_VIORB mono_live_viorb(&mutex, &camera_capture, &mavconn);
//        Mono_Offline_VIORB mono_offline_viorb(&mutex);

        int option = 3;
        cout << "Select an option (1:Live SLAM 2:Record Dataset 3:Offline SLAM)" << endl;
        cin >> option;

        if (option == 1)
        {
            Mavlink_Control mavconn(baudrate, uart_name);
            Camera_Capture camera_capture(&mutex, &mavconn);
            Mono_Live_VIORB mono_live_viorb(&mutex, &camera_capture, &mavconn);

            cout << "Start SLAM thread,..." << endl;
            mono_live_viorb.start(vocabulary, setting);

            cout << "Start Mavlink thread,..." << endl;
            mavconn.start();

            //stop all thread in order

            mono_live_viorb.stop();
            //camera_capture.stop();
            mavconn.stop();
        }
        else if (option == 2)
        {
            Mavlink_Control mavconn(baudrate, uart_name);
            Camera_Capture camera_capture(&mutex, &mavconn);

            cout << "Start Camera thread,..." << endl;
            camera_capture.start();
            cout << "Start Mavlink thread,..." << endl;
            mavconn.start();
            //stop all thread in order

            camera_capture.stop();
            //camera_capture.stop();
            mavconn.stop();
        }
        else if (option == 3)
        {
            Mono_Offline_VIORB mono_offline_viorb(&mutex);

            cout << "Start SLAM thread,..." << endl;
            mono_offline_viorb.start(vocabulary, setting);

           // mono_offline_viorb.stop();

        }
        return 0;
    }
    catch ( int error )
    {
        fprintf(stderr,"threw exception %i \n" , error);
        return error;
    }
}

//   Parse Command Line
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, char *&vocabulary, char *&setting)
{
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -v <path_to_vocabulary> -s <path_to_setting>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n",commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path to vocabulary
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--vocab") == 0) {
            if (argc > i + 1) {
                vocabulary = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path to setting
        if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--setting") == 0) {
            if (argc > i + 1) {
                setting = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    return;
}

#pragma clang diagnostic pop