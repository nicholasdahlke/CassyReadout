//
// Created by nicholas on 13.11.22.
//
#include <iostream>
#include "Cassy.h"
#include "SerialPort.h"
#include <string>
#include <fstream>
#include <chrono>

#define NEWLINE "\n"
#define DELIMITER ","

Cassy* cassy_handle;
SerialPort* serial_handle;
std::string input;
std::string capture_path;
std::string serial_port;

std::ofstream cap_file;

struct DataCap
{
    int index;
    double time;
    double u_a1 = 0;
    double u_a2 = 0;
    double u_b2 = 0;
    double angle;

};

std::vector<DataCap> capture;

DataCap read(std::chrono::high_resolution_clock::time_point t1)
{
    DataCap tempObject;
    static int index = 0;
    tempObject.index = index;
    index++;
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span_from_start = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    tempObject.time = time_span_from_start.count();
    //tempObject.u_a1 = cassy_handle->read_voltage(cassy_handle->voltage_channels[0], Cassy::v5);
    //tempObject.u_a2 = cassy_handle->read_voltage(cassy_handle->voltage_channels[1], Cassy::v5);
    //tempObject.u_b2 = cassy_handle->read_voltage(cassy_handle->voltage_channels[3], Cassy::v5);
    uint8_t buf = {0};
    serial_handle->read_bytes(&buf, 1);
    tempObject.angle = static_cast<double>(buf);
    return tempObject;
}

void writeHeader()
{
    cap_file << "index";
    cap_file << DELIMITER;
    cap_file << "time";
    cap_file << DELIMITER;
    cap_file << "u_a1";
    cap_file << DELIMITER;
    cap_file << "u_a2";
    cap_file << DELIMITER;
    cap_file << "u_b2";
    cap_file << DELIMITER;
    cap_file << "angle";
    cap_file << NEWLINE;
}

void writeCSV(DataCap row)
{
    cap_file << std::to_string(row.index);
    cap_file << DELIMITER;
    cap_file << std::to_string(row.time);
    cap_file << DELIMITER;
    cap_file << std::to_string(row.u_a1);
    cap_file << DELIMITER;
    cap_file << std::to_string(row.u_a2);
    cap_file << DELIMITER;
    cap_file << std::to_string(row.u_b2);
    cap_file << DELIMITER;
    cap_file << std::to_string(row.angle);
    cap_file << NEWLINE;
}

int main()
{
    cassy_handle = new Cassy();
    serial_handle = new SerialPort();

    /*std::cout << "Connect to Cassy ? (J/N): ";
    std::cin >> input;
    if(input == "J" || input == "j" || input == "y")
    {
        if (!cassy_handle->connect())
            return -2;
    }
    else
        return -1;
*/
    for(const std::string& value: serial_handle->serial_ports)
        std::cout << value << "\n";

    if (serial_handle->serial_ports.size() == 1)
        serial_port = serial_handle->serial_ports[0];
    else
    {
        std::cout << "Enter Serial Port: ";
        std::cin >> serial_port;
    }

    std::cout << "Connect to Serial Device ? (J/N): ";
    std::cin >> input;
    serial_handle->serial_port_name = serial_port;
    serial_handle->baud = SerialPort::SER_BAUD_230400;
    if(input == "J" || input == "j" || input == "y")
    {
        serial_handle->connect();
        if (!serial_handle->connected)
            return -2;
    }
    else
        return -1;

    std::cout << "Enter  Capture Path: ";
    std::cin.ignore();
    std::getline(std::cin, capture_path);
    cap_file.open(capture_path);
    if(!cap_file.is_open())
        return -1;



    std::cout << "Start Capture? (J/N): ";
    std::cin >> input;
    if(!(input == "J" || input == "j" || input == "y"))
        return -1;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    for (;;)
    {
        capture.push_back(read(t1));
        writeCSV(capture.back());
        if(capture.size() > 1000)
            break;
    }
    cap_file.close();
    serial_handle->disconnect();
    //cassy_handle->disconnect();
    return 0;
}


