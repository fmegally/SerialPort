#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <string>
#include <vector>
#include <stdint.h>
#include <termios.h>

std::vector<std::string> getSerialDevices(std::string dev_dir);

class SerialPort {
        public:

        enum state_t {
                PORTCLOSED,
                PORTOPEN
        };

        enum parity_t {
                NOPARITY,
                EVEN,
                ODD
        };

        enum stopbits_t {
                STPBIT_1,
                STPBIT_2
        };

        enum bytesize_t {
                BS_5,
                BS_6,
                BS_7,
                BS_8
        };

        SerialPort(std::string device = "",
                   uint32_t baud = 9600,
                   bytesize_t bs = BS_8,
                   parity_t parity = NOPARITY,
                   stopbits_t stopbits = STPBIT_1);
        ~SerialPort();
        
        void set_device(std::string device);
        void set_baud(uint32_t baud);
        void set_parity(parity_t parity);
        void set_byte_size(bytesize_t bs);
        void set_stop_bits(stopbits_t stopbits);

        void open_port();
        void close_port();

        int receive(std::string &s, size_t nbytes);
        int receive(uint8_t buffer[], size_t nbytes);
        int receive(std::vector<uint8_t> &buffer, size_t nbytes);
        int transmit(const uint8_t buffer[], size_t nbytes);
        int transmit(std::string buffer);
        int transmit(std::vector<uint8_t> buffer);
        uint8_t get_byte();
        
        private:
        std::string device;
        int device_file_descriptor;

        uint32_t baud_rate;
        state_t port_state;
        bytesize_t byte_size;
        parity_t parity;
        stopbits_t stop_bits;

        struct termios port_config;
        
        //Helper functions
        int baud_to_flags(uint32_t baud);
        bool device_exists(const std::string &device);
};
#endif
