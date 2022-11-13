//
// Created by nicholas on 09.09.22.
//

#ifndef CASSYINTERFACE_CASSY_H
#define CASSYINTERFACE_CASSY_H
#include <cstdint>
#include "hidapi.h"
#include <vector>

class Cassy {
public:

    enum CassyVoltageRanges
    {
        v1 = 137,    //+-250V
        v2 = 136,    //+-100V
        v3 = 168,    //+-30V
        v4 = 200,    //+-10V
        v5 = 232,    //+-3V
        v6 = 192,    //+-1V
        v7 = 224,    //+-0.3V
        v8 = 143     //+-0.1V
    };

    enum input
    {
        a,
        b
    };

    struct voltage_channel
    {
        uint8_t cassy_id;
        input input_channel;
        int channel_id;
    };

    struct relay
    {
        uint8_t cassy_id;
        int relay_id;
    };

    std::vector<voltage_channel> voltage_channels; //vector of all voltage channels
    std::vector<relay> relays; //vector of all relays
    std::vector<uint8_t> cassys; //vector of all cassy ids

    bool connect();
    void disconnect();

    double read_voltage(voltage_channel channel, CassyVoltageRanges range); //reads and returns the voltage
    int set_relay(relay relay_channel, bool value);

    void print_voltage_channels();
    void print_relays();
    bool connected = false;
    ~Cassy() {disconnect();}

private:

    uint16_t vendor = 0xf11;
    uint16_t product = 0x1001;


    uint8_t cassy_id_prefix = 0x1b; //prefix in front of the actual cassy id
    uint8_t multiple_cassy_ids[8] = {0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x80}; //id of the cassy from left to right

    enum CassyCommands
    {
        get_hardware_version = 1,   //returns the type of cassy connected
        get_firmware_version = 2,   //returns the firmware version
        get_sensor_box_a = 126,     //get the type of sensor box connected to input a
        get_sensor_box_b = 127,     //get the type fo sensor box connected to input b
        set_relay_value = 43,       //sets the relay to a value
        get_input_value_a = 18,     //gets the input value while setting the range of input a
        get_input_value_b = 19      //gets the input value while setting the range of input b
    };



    hid_device *cassy_handle = NULL;
    int hid_res;

    void get_cassys(); //returns an array of all available cassy ids
    void get_voltage_channels(); //returns an array of all available voltage channels
    void get_relays(); //returns an array of all available relays

    int send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t* res_buf, uint8_t* parameters, int parameter_length);
    int send_command(uint8_t cassy_id_param, CassyCommands command, uint8_t* res_buf);
    double convert_adc_raw(uint8_t * voltage_adc_buf, CassyVoltageRanges range);

    void print_hex(uint8_t *buf, int len);
};


#endif //CASSYINTERFACE_CASSY_H
