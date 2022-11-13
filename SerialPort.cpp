//
// Created by Nicholas Dahlke on 29.09.2022.
//

#include "SerialPort.h"
#include <iostream>

int SerialPort::connect()
{
#ifdef _WIN32
    const char *serial_port_name_cstr = serial_port_name.c_str();
    serial_handle = CreateFile(serial_port_name_cstr, GENERIC_READ | GENERIC_WRITE,
                               0,
                               0,
                               OPEN_EXISTING,
                               FILE_ATTRIBUTE_NORMAL,
                               0);

    if(serial_handle == INVALID_HANDLE_VALUE)
    {
        if(GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            std::cerr << "Serial port doesn't exist\n";
            return -1;
        }
    }

    serial_parameters.DCBlength = sizeof(serial_parameters);

    if(!GetCommState(serial_handle, &serial_parameters))
    {
        std::cerr << "Error getting serial parameters using GetCommState \n";
        return -2;
    }

    serial_parameters.BaudRate = baud;
    serial_parameters.ByteSize = 8;
    serial_parameters.StopBits = ONESTOPBIT;
    serial_parameters.Parity = NOPARITY;

    if(!SetCommState(serial_handle, &serial_parameters))
    {
        std::cerr << "Error setting new serial parameters \n";
        return -3;
    }

    serial_timeouts.ReadIntervalTimeout = 60;
    serial_timeouts.ReadTotalTimeoutConstant = 60;
    serial_timeouts.ReadTotalTimeoutMultiplier = 15;
    serial_timeouts.WriteTotalTimeoutConstant = 60;
    serial_timeouts.WriteTotalTimeoutMultiplier = 15;

    if(!SetCommTimeouts(serial_handle, &serial_timeouts))
    {
        std::cerr << "Error setting serial port timeouts\n";
        return -4;
    }
    connected = true;

    if(!SetCommMask(serial_handle, EV_RXCHAR))
    {
        std::cerr << "Error setting com mask\n";
        return -5;
    }
#endif
#ifdef linux
    const char *serial_port_name_cstr = serial_port_name.c_str();
    serial_handle = open(serial_port_name_cstr, O_RDWR);

    if (serial_handle < 0)
    {
        std::cerr << "Error opening serial port\n";
        return -1;
    }

    if(tcgetattr(serial_handle, &tty) != 0)
    {
        std::cerr << "Error getting Serial Attributes\n";
        return -2;
    }

    tty.c_cflag &= ~PARENB;  //No parity bit
    tty.c_cflag &= ~CSTOPB;  //Use one stop bit
    tty.c_cflag &= ~CSIZE;   //Clear size bit
    tty.c_cflag |= CS8;      //Set size to 8 bit
    tty.c_cflag &= ~CRTSCTS; //Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  //Allow reading and disable control lines

    tty.c_lflag &= ~ICANON;  //Disable canoncial mode
    tty.c_lflag &= ~ECHO;    //Disable ECHO
    tty.c_lflag &= ~ECHOE;   //
    tty.c_lflag &= ~ECHONL;  //
    tty.c_lflag &= ~ISIG;   //Disable signal characters

    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  //Disable special handling of recieved data
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                           //Disable software flow control
    
    tty.c_oflag &= ~OPOST; //Disable special handling of output data
    tty.c_oflag &= ~ONLCR; //Prevent newline conversion
    
    tty.c_cc[VTIME] = 0;  //Set timeout for reading in deciseconds
    tty.c_cc[VMIN]  = 0;   //Wait for any byte to be recieved

    speed_t baud_rate;

    switch (baud) {
        case SER_BAUD_110:
            baud_rate = B110;
            break;
        case SER_BAUD_300:
            baud_rate = B300;
            break;
        case SER_BAUD_600:
            baud_rate = B600;
            break;
        case SER_BAUD_1200:
            baud_rate = B1200;
            break;
        case SER_BAUD_2400:
            baud_rate = B2400;
            break;
        case SER_BAUD_4800:
            baud_rate = B4800;
            break;
        case SER_BAUD_9600:
            baud_rate = B9600;
            break;
        case SER_BAUD_19200:
            baud_rate = B19200;
            break;
        case SER_BAUD_38400:
            baud_rate = B38400;
            break;
        case SER_BAUD_57600:
            baud_rate = B57600;
            break;
        case SER_BAUD_115200:
            baud_rate = B115200;
            break;
        case SER_BAUD_230400:
            baud_rate = B230400;
            break;
        default:
            std::cerr << "Baudrate not recognized\n";
            return -3;
    }

    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    if(tcsetattr(serial_handle, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error updating serial configuration\n";
        std::cerr << explain_tcsetattr(serial_handle, TCSANOW, &tty);
        return -4;
    }

    connected = true;
#endif
    return 0;
}

SerialPort::SerialPort()
{
    serial_ports = get_serial_ports();
}

int SerialPort::write_bytes(std::vector<uint8_t> data)
{
    uint8_t send_buf[data.size()];

    for(int i = 0; i < data.size(); i++)
    {
        send_buf[i] = data[i];
    }
#ifdef _WIN32
    DWORD dwWrite = 0;

    if(!WriteFile(serial_handle, send_buf, data.size(), &dwWrite, NULL))
    {
        std::cerr << "Error writing data\n";
        return -1;
    }

    if(dwWrite != data.size())
    {
        std::cerr << "None or not all bytes written\n";
        return -2;
    }
#endif
#ifdef linux
    int dwWrite = 0;
    dwWrite = write(serial_handle, send_buf, sizeof(send_buf));

    if(dwWrite <= 0)
    {
        std::cerr << "Error writing to serial port\n";
        return -1;
    }
    if (dwWrite != sizeof(send_buf))
    {
        std::cerr << "None or not all bytes written";
        return -1;
    }
#endif
    return dwWrite;
}
int SerialPort::write_bytes(char* data, int length)
{

#ifdef _WIN32
    DWORD dwWrite = 0;

    if(!WriteFile(serial_handle, data, length, &dwWrite, NULL))
    {
        std::cerr << "Error writing data\n";
        std::cout << GetLastError() << "\n";
        return -1;
    }
    FlushFileBuffers(serial_handle);
    if(dwWrite != length)
    {
        std::cerr << "None or not all bytes written\n";
        return -2;
    }
#endif
#ifdef linux
    int dwWrite = 0;
    dwWrite = write(serial_handle, data, sizeof(data));

    if(dwWrite <= 0)
    {
        std::cerr << "Error writing to serial port\n";
        return -1;
    }
    if (dwWrite != sizeof(data))
    {
        std::cerr << "None or not all bytes written";
        return -1;
    }
#endif
    return dwWrite;
}


std::vector<uint8_t> SerialPort::read_bytes(int length)
{

    uint8_t read_buf[length + 1];
    std::vector<uint8_t> return_vector;

#ifdef _WIN32
    DWORD dwRead = 0;
    while(dwRead == 0)
    {
        if(!ReadFile(serial_handle, read_buf, length, &dwRead, NULL))
        {
            std::cerr << "Error reading data \n";
            return return_vector;
        }
    }
#endif
#ifdef linux
    int dwRead = 0;

    dwRead = read(serial_handle, &read_buf, sizeof(read_buf));
    if(dwRead <= 0)
    {
        std::cerr << "Error reading data \n";
        return return_vector;
    }
#endif

    for(int i = 0; i < length; i++)
    {
        return_vector.push_back(read_buf[i]);
    }

    return return_vector;
}

int SerialPort::read_bytes(uint8_t *buf, int length)
{
#ifdef _WIN32
    DWORD dwRead = 0;
    while(dwRead == 0)
    {
        if(!ReadFile(serial_handle, buf, length, &dwRead, NULL))
        {
            std::cerr << "Error reading data \n";
            return -1;
        }
    }
#endif
#ifdef linux
    int dwRead = 0;

    dwRead = read(serial_handle, buf, length);
    if(dwRead < 0)
    {
        std::cerr << "Error reading data \n";
        return -1;
    }
#endif
    return dwRead;
}


std::vector<std::string> SerialPort::get_serial_ports()
{
    std::vector<std::string> return_vector;
#ifdef linux
    std::string tty_path = "/sys/class/tty";
    for (const auto & entry : std::filesystem::directory_iterator(tty_path))
    {
        if(std::filesystem::exists(entry.path().string() + "/device") && entry.path().filename().string().find('S',3) == std::string::npos)
            return_vector.push_back("/dev/" + entry.path().filename().string());
    }
#endif
#ifdef _WIN32
    char lpTargetPath[5000];
    for (int i = 0; i < 255; ++i) {
        std::string str = "COM" + std::to_string(i);
        DWORD test = QueryDosDevice(str.c_str(), lpTargetPath, 5000);
        if(test != 0)
            return_vector.push_back(str);
        if(GetLastError() == ERROR_INSUFFICIENT_BUFFER) {}
    }
#endif
    if(return_vector.empty())
        return_vector.push_back("No serial device connected");
    return return_vector;
}

void SerialPort::disconnect()
{
    if(!connected)
        std::cerr << "Not connected\n";
#ifdef _WIN32
    CloseHandle(serial_handle);
#endif
#ifdef linux
    close(serial_handle);
#endif
    connected = false;
}

SerialPort::~SerialPort()
{
    disconnect();
}