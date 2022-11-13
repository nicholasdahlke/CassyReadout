//
// Created by Nicholas Dahlke on 29.09.2022.
//

#ifndef CASSYINTERFACE_SERIALPORT_H
#define CASSYINTERFACE_SERIALPORT_H
#include <string>
#include <vector>
#include <cstdint>
#ifdef _WIN32
#include <windows.h>
#endif
#ifdef linux
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <filesystem>
#include <libexplain/tcsetattr.h>
#endif

class SerialPort {
public:
    enum BaudRate
    {
        SER_BAUD_110 = 100,
        SER_BAUD_300 = 300,
        SER_BAUD_600 = 600,
        SER_BAUD_1200 = 1200,
        SER_BAUD_2400 = 2400,
        SER_BAUD_4800 = 4800,
        SER_BAUD_9600 = 9600,
        SER_BAUD_14400 = 14400,
        SER_BAUD_19200 = 19200,
        SER_BAUD_38400 = 38400,
        SER_BAUD_57600 = 57600,
        SER_BAUD_115200 = 115200,
        SER_BAUD_128000 = 128000,
        SER_BAUD_256000 = 256000,
        SER_BAUD_230400 = 230400
    };

    SerialPort();
    ~SerialPort();

    int write_bytes(std::vector<uint8_t> data);
    int write_bytes(char* data, int length);
    std::vector<uint8_t> read_bytes(int length);
    int read_bytes(uint8_t * buf, int length);
    int connect();
    void disconnect();

    std::vector<std::string> serial_ports;
    bool connected = false;
    std::string serial_port_name;
    BaudRate baud;
private:
#ifdef _WIN32
    HANDLE serial_handle;
    DCB serial_parameters = {0};
    COMMTIMEOUTS serial_timeouts = {0};
#endif
#ifdef linux
    struct termios tty;
    int serial_handle;
#endif

    std::vector<std::string> get_serial_ports();


};


#endif //CASSYINTERFACE_SERIALPORT_H
