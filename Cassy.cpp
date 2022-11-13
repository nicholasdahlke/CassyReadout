//
// Created by nicholas on 09.09.22.
//

#include "Cassy.h"
#include <iostream>

#define ADC_CALIB_ACCURACY 2000
#define TIMEOUT 100

bool Cassy::connect()
{
    hid_res = hid_init();
        cassy_handle = hid_open(vendor, product, NULL);

    if (cassy_handle == NULL)
    {
        std::cerr << "Error while connecting to cassy\n";
        return false;
    }
    get_cassys();
    get_voltage_channels();
    get_relays();
    connected = true;
    return true;
}

void Cassy::disconnect()
{
    hid_close(cassy_handle);
    hid_res = hid_exit();
    connected = false;
}

#ifdef linux
#define CASSY_MESSAGE_LENGTH 64
int Cassy::send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t *res_buf)
{
    uint8_t message[CASSY_MESSAGE_LENGTH];
    message[0] = 3;
    message[1] = cassy_id_prefix;
    message[2] = cassy_id_param;
    message[3] = command;

    hid_res = hid_write(cassy_handle, message, CASSY_MESSAGE_LENGTH);
    if (hid_res == -1)
    {
        std::cerr << "Error writing command to device\n";
        -2;
    }

    hid_res = hid_read_timeout(cassy_handle, res_buf, CASSY_MESSAGE_LENGTH, TIMEOUT);
    return hid_res;
}

int Cassy::send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t *res_buf, uint8_t *parameters, int parameter_length)
{
    uint8_t message_length = 3;
    uint8_t message[CASSY_MESSAGE_LENGTH];
    message[1] = cassy_id_prefix;
    message[2] = cassy_id_param;
    message[3] = command;
    for (int i = 0; i < parameter_length; ++i) {
        message[4+i] = parameters[i];
        message_length++;
    }
    message[0] = message_length;

    hid_res = hid_write(cassy_handle, message, CASSY_MESSAGE_LENGTH);
    if (hid_res == -1)
    {
        std::cerr << "Error writing command to device\n";
        -2;
    }

    hid_res = hid_read_timeout(cassy_handle, res_buf, CASSY_MESSAGE_LENGTH, TIMEOUT);
    return hid_res;
}
#endif

#ifdef _WIN32
#define CASSY_MESSAGE_LENGTH 65
int Cassy::send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t *res_buf)
{
    uint8_t message[CASSY_MESSAGE_LENGTH];
    message[0] = 0;
    message[1] = 3;
    message[2] = cassy_id_prefix;
    message[3] = cassy_id_param;
    message[4] = command;

    hid_res = hid_write(cassy_handle, message, CASSY_MESSAGE_LENGTH);
    //print_hex(message, sizeof(message)/ sizeof(uint8_t));
    if (hid_res == -1)
    {
        std::cerr << "Error writing command to device\n";
        return -2;
    }

    hid_res = hid_read_timeout(cassy_handle, res_buf, CASSY_MESSAGE_LENGTH, TIMEOUT);
    return hid_res;
}

int Cassy::send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t *res_buf, uint8_t *parameters, int parameter_length)
{
    uint8_t message_length = 3;
    uint8_t message[CASSY_MESSAGE_LENGTH];
    message[0] = 0;
    message[2] = cassy_id_prefix;
    message[3] = cassy_id_param;
    message[4] = command;
    for (int i = 0; i < parameter_length; ++i) {
        message[5+i] = parameters[i];
        message_length++;
    }
    message[1] = message_length;

    hid_res = hid_write(cassy_handle, message, CASSY_MESSAGE_LENGTH);
    //print_hex(message, sizeof(message)/ sizeof(uint8_t));
    if (hid_res == -1)
    {
        std::cerr << "Error writing command to device\n";
        return -2;
    }

    hid_res = hid_read_timeout(cassy_handle, res_buf, CASSY_MESSAGE_LENGTH, TIMEOUT);
    return hid_res;
}
#endif

void Cassy::get_cassys() {
    for (int i = 0; i < sizeof(multiple_cassy_ids) / sizeof(uint8_t) ; ++i)
    {
        uint8_t res_buf[CASSY_MESSAGE_LENGTH];
        int res = send_command(multiple_cassy_ids[i], get_hardware_version, res_buf);
        if (res == 64)
            cassys.push_back(multiple_cassy_ids[i]);
    }
}

void Cassy::get_voltage_channels()
{
    int channel_id = 0;
    for (std::vector<uint8_t>::size_type i = 0; i != cassys.size(); i++)
    {
        voltage_channel temp1;
        temp1.cassy_id = cassys[i];
        temp1.input_channel = a;
        temp1.channel_id = channel_id;

        channel_id++;

        voltage_channel temp2;
        temp2.cassy_id = cassys[i];
        temp2.input_channel = b;
        temp2.channel_id = channel_id;

        channel_id++;

        voltage_channels.push_back(temp1);
        voltage_channels.push_back(temp2);
    }
}

void Cassy::get_relays()
{
    int relay_id = 0;
    for (std::vector<uint8_t>::size_type i = 0; i != cassys.size(); i++)
    {
        relay temp;
        temp.cassy_id = cassys[i];
        temp.relay_id = relay_id;
        relay_id++;
        relays.push_back(temp);
    }
}

int Cassy::set_relay(Cassy::relay relay_channel, bool value)
{
    uint8_t parameters[1];
    if(value)
        parameters[0] = 1;
    else
        parameters[0] = 0;
    uint8_t res_buf[CASSY_MESSAGE_LENGTH];
    hid_res = send_command(relay_channel.cassy_id, set_relay_value, res_buf, parameters, 1);
    if (res_buf[1] != 0x1)
    {
        std::cerr << "Error setting relay\n";
        return -1;
    }
    return 0;

}

double Cassy::convert_adc_raw(uint8_t *voltage_adc_buf, Cassy::CassyVoltageRanges range)
{
    int16_t raw_adc_voltage = 0;
    raw_adc_voltage = voltage_adc_buf[2] << 8;
    raw_adc_voltage = raw_adc_voltage | voltage_adc_buf[3];
    float range_plus_minus = 0;
    switch (range) {
        case v1:
            range_plus_minus = 250;
            break;
        case v2:
            range_plus_minus = 100;
            break;
        case v3:
            range_plus_minus = 30;
            break;
        case v4:
            range_plus_minus = 10;
            break;
        case v5:
            range_plus_minus = 3;
            break;
        case v6:
            range_plus_minus = 1;
            break;
        case v7:
            range_plus_minus = 0.3;
            break;
        case v8:
            range_plus_minus = 0.1;
            break;
    }

    const double volt_per_adc = range_plus_minus / ADC_CALIB_ACCURACY;
    return volt_per_adc * raw_adc_voltage;
}

double Cassy::read_voltage(Cassy::voltage_channel channel, Cassy::CassyVoltageRanges range)
{
    CassyCommands command;
    if (channel.input_channel == a)
        command = get_input_value_a;
    if (channel.input_channel == b)
        command = get_input_value_b;
    uint8_t voltage_buf[CASSY_MESSAGE_LENGTH];
    uint8_t parameters[1];
    parameters[0] = range;
    hid_res = send_command(channel.cassy_id, command, voltage_buf, parameters, 1);
    if (hid_res < 1)
        std::cerr << "Error reading voltage\n";
    //switch(1){ //Uncomment to ignore errors
    switch (voltage_buf[1]) {
        case 0:
            std::cerr << "Something went wrong(Return code 0)\n";
            break;
        case 1:
            return convert_adc_raw(voltage_buf, range);
        case 2:
            std::cerr << "Overflow on channel" <<  static_cast<int>(channel.input_channel) << " on cassy " << std::hex << static_cast<int>(channel.cassy_id) << "\n";
            break;
        case 3:
            std::cerr << "Underflow on channel " << static_cast<int>(channel.input_channel) << " on cassy " << std::hex << static_cast<int>(channel.cassy_id) << "\n";
            break;

    }
    return 0;
}

void Cassy::print_hex(uint8_t *buf, int len)
{
    for (int i = 0; i < len; ++i) {
        std::cout << std::hex << static_cast<int>(buf[i]);
    }
    std::cout << "\n";
}
