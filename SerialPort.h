#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <string>
#include <vector>
#include <stdint.h>

/*
enum Baud {
        B_0,
        B_1200,
        B_2400,
        B_4800,
        B_9600,
        B_19200,
        B_38400,
        B_57600,
        B_115200,
        B_250000
};
*/

std::vector<std::string> getSerialDevices(std::string dev_dir);

class SerialPort {
        enum state_t {
                closed,
                open
        };

        enum parity_t {
                noParity,
                even,
                odd
        };

        enum stopbits_t {
                stpbit_1,
                stpbit_2
        };

        enum bytesize_t {
                bs_5,
                bs_6,
                bs_7,
                bs_8
        };

        private:
        std::string device;
        int deviceFileDescriptor;

        uint32_t baudRate;
        state_t portState;
        bytesize_t byteSize;
        parity_t parity;
        stopbits_t stopBits;

        struct termios portConfig;
        
        bool deviceExists(const std::string &device);

        public:
        SerialPort(std::string device = "",
                   uint32_t baud = 0,
                   bytesize_t bs = cs_8,
                   parity_t parity = noParity,
                   stopbits_t stopbits = stpbit_1);

        void setDevice(std::string device);
        void setBaud(baud_t baud);
        void setParity(parity_t parity);
        void setByteSize(bytesize_t bs);
        void setStopBits(stopbits_t stopbits);


        void Open();
        void Close();

        int Read(std::string &s, size_t nbytes);
        int Read(uint8_t buffer[], size_t nbytes);
        int Read(std::vector<uint8_t> &buffer, size_t nbytes);
        int Write(const uint8_t buffer[], size_t nbytes);
        int Write(std::string buffer);
        int Write(std::vector<uint8_t> buffer);

};
#endif
