#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <string>
#include <vector>


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
        enum State {
                closed,
                open
        };

        enum Parity {
        	noParity,
        	even,
        	odd
        };

        enum StopBits {
        	stpbit_1,
        	stpbit_2
        };

        enum CharSize {
        	cs_5,
        	cs_6,
        	cs_7,
        	cs_8
        };

	private:
	std::string device;
	State portState;
        CharSize portCharSize;
        Parity portParity;
        StopBits portStopBits 
	unsigned int baud_rate;
	int dev_file_descriptor;
	struct termios port_config;

	public:
	SerialPort();
	SerialPort(const std::string device = "",
	           const unsigned int baud = 0,
	           const CharSize cs = cs_8,
	           const Parity parity = noParity,
	           const StopBits stopbits = stpbit_1);

	~SerialPort();

	void setDevice(const std::string device);
	void setBaud(const Baud baud);
	void setParity(const Parity parity);
 	void setCharSize(const CharSize cs);
	void setStopBits(const StopBits stopbits);


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
