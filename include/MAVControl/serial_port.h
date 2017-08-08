#ifndef LEARNVI_DRONE_SERIAL_PORT_H
#define LEARNVI_DRONE_SERIAL_PORT_H

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <exception>

#include <mavlink/v1.0/common/mavlink.h>
#include "Utility/system_log.h"

//   Defines

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// Status flags
#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

//class Serial_Port;



// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class Serial_Port {

public:

    Serial_Port();

    Serial_Port(const char *uart_name_, int baudrate_, System_Log *system_log_);

    void initialize_defaults();

    ~Serial_Port();

    bool debug;
    const char *uart_name;
    int baudrate;
    int status;

    int read_message(mavlink_message_t &message);

    int write_message(const mavlink_message_t &message);

    void open_serial();

    void close_serial();

    void start();

    void stop();

    void handle_quit(int sig);

private:

    System_Log *system_log;

    int fd;
    mavlink_status_t lastStatus;
    pthread_mutex_t lock;

    int _open_port(const char *port);

    bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

    int _read_port(uint8_t &cp);

    int _write_port(char *buf, unsigned len);
};

#endif //LEARNVI_DRONE_SERIAL_PORT_H
