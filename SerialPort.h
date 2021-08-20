#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_
#include <string>
#include <string>
#include <vector>
#include <termios.h>

enum State {
    CLOSED,
    OPEN
};

enum Parity {
	NONE,
	EVEN,
	ODD
};

enum StopBits {
	STPBIT_1,
	STPBIT_2
};

enum CharSize {
	CS_5,
	CS_6,
	CS_7,
	CS_8
};

enum Baud {
	B_0,
	B_1200,
	B_2400,
	B_4800,
	B_9600,
	B_19200,
	B_38400,
	B_57600,
	B_115200
};


std::vector<std::string> getSerialDevices(std::string dev_dir);

class SerialPort {
	private:
	State state;
	std::string device;
	speed_t baud;
	int file_descriptor;
	struct termios port_config;

	public:
	SerialPort();
	SerialPort(const std::string device = "",
	           const speed_t baud = B115200,
	           const CharSize cs = CS_8,
	           const Parity parity = PARITY_DISABLED,
	           const StopBits stopbits = STPBIT_1);

	~SerialPort();

	void setDevice(const std::string device);
	void setBaud(const Baud baud);
	void setParity(const Parity parity);
 	void setCharSize(const CharSize cs);
	void setStopBits(const StopBits stopbits);


	void Connect();
	void Disconnect();
	int Read(std::string &s, size_t nbytes);
	int Read(uint8_t buffer[], size_t nbytes);
	int Read(std::vector<uint8_t> &buffer, size_t nbytes);
	int Write(const uint8_t buffer[], size_t nbytes);
	int Write(std::string buffer);
	int Write(std::vector<uint8_t> buffer);

};
#endif
