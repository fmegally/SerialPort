#include "SerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <regex>

std::vector<std::string> getSerialDevices(std::string dev_dir)
{
	const char* devices_folder = dev_dir.c_str();
	std::vector<std::string> port_names;
	std::regex pattern ("tty(USB|ACM|S).*");

	DIR *dp;
	struct dirent *dep;

	if ((dp = opendir(devices_folder)) == NULL){
		perror("Error openning /dev folder");
		return port_names;
	}

	while((dep = readdir(dp)) != NULL)
	{
		if (std::regex_match(std::string(dep->d_name), pattern)){
			port_names.push_back(dep->d_name);
		} else {
			continue;
		}
	}

	sort(port_names.begin(),port_names.end());

	return port_names;
}

SerialPort::SerialPort()
{
	device = "";
	state = CLOSED;

	port_config.c_cflag &= ~CRTSCTS;
	port_config.c_cflag &= ~CSTOPB;

	port_config.c_cflag &= ~PARENB;
	port_config.c_cflag &= ~PARODD;

	port_config.c_cflag |=  (CLOCAL | CREAD) ;	

	port_config.c_cflag &=  CSIZE;
	port_config.c_cflag |=  CS8;

	port_config.c_iflag &= ~(IXON | IXOFF | IXANY);
	port_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | ICRNL | INLCR | IGNCR | IUTF8);
	port_config.c_iflag &= ~(INPCK | IGNPAR);
	port_config.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

	port_config.c_oflag &= ~OPOST;

	return;
}

SerialPort::SerialPort(const std::string device, const speed_t baud, const CharSize cs, const Parity parity, const StopBits stopbits)
{
	int fh = open(device.c_str(),O_RDONLY);

	if (fh > 0){
		this->device = device;
		std::cout << "device assigned successfully" << std::endl;
		close(fh);
   	} else {
		throw std::invalid_argument("device does not exist.");
	}

	state = CLOSED;

	//c_cflag settings
	//disable RTS/CTS hardware flow control
	port_config.c_cflag &= ~CRTSCTS;

	if (stopbits==STPBIT_2 ){
		port_config.c_cflag |= CSTOPB;
	} else {
		port_config.c_cflag &= ~CSTOPB;
	}

	if (parity == EVEN){
		port_config.c_cflag |= PARENB;
		port_config.c_cflag &= ~PARODD;
	} else if (parity == ODD){
		port_config.c_cflag |= PARENB;
		port_config.c_cflag |= PARODD;

	} else {
		port_config.c_cflag &= ~PARENB;
		port_config.c_cflag &= ~PARODD;
	}

	//enable local mode (no modem control lines)
	//enable receiver (reading)
	port_config.c_cflag |=  (CLOCAL | CREAD) ;	

	//character size setting. must clear size bits before assigning
	//set to 8-bit byte size
	port_config.c_cflag &=  CSIZE;
	switch(cs){
	case CS_8:
		port_config.c_cflag |=  CS8;
		break;
	case CS_7:
		port_config.c_cflag |=  CS7;
		break;
	case CS_6:
		port_config.c_cflag |=  CS6;
		break;
	case CS_5:
		port_config.c_cflag |=  CS5;
		break;
	default:
		perror("error setting serial communication char size. SerialPort object not created.");
		return;
	}

	//c_iflag settings
	port_config.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software (in-band) flow-control
	port_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | ICRNL | INLCR | IGNCR | IUTF8); //ignore BREAK condition on input
	port_config.c_iflag &= ~(INPCK | IGNPAR);
	
	//c_lflag settings
	port_config.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

	//c_oflag settings
	port_config.c_oflag &= ~OPOST;

	cfsetispeed(&port_config, baud);
	cfsetospeed(&port_config, baud);

	return;
}

SerialPort::~SerialPort()
{
		close(file_descriptor);
		return;
}

void SerialPort::Connect()
{
	if (device.empty()){
		throw std::runtime_error("device not set.");
	}

	if (cfgetospeed(&port_config)==0 && cfgetispeed(&port_config)==0)
	{
		std::cout << cfgetospeed(&port_config) << std::endl;
		std::cout << cfgetispeed(&port_config) << std::endl;
		throw std::runtime_error("baud rate is 0");
	}

	if(state == CLOSED){
		file_descriptor = open(device.c_str(), O_RDWR | O_NOCTTY);
		if (file_descriptor == -1){
			throw std::runtime_error("could not open device");
		} else {
			fcntl(file_descriptor, F_SETFL, 0);
			tcsetattr(file_descriptor, TCSANOW, &port_config);
			state = OPEN;
		}
	} else {
		throw std::logic_error("port is open.");
	}
}

void SerialPort::Disconnect()
{
	if(state == OPEN){
		close(file_descriptor);
		file_descriptor = 0;
		state = CLOSED;
	} else {
		throw std::logic_error("port not open");
	}

	return;
}

int SerialPort::Read(std::vector<uint8_t> &dst, size_t nbytes)
{
	port_config.c_cc[VMIN] = nbytes;
	port_config.c_cc[VTIME] = 0;

	tcsetattr(file_descriptor, TCSANOW, &port_config);

	if(state == CLOSED){
		throw std::logic_error("can't read from closed port.");
	}
	
	char c_buffer[nbytes];

	int n = read(file_descriptor, c_buffer, nbytes);
	if (n == -1) {
		throw std::runtime_error("could not read from ttyS*");
	}

	dst.assign(c_buffer, c_buffer + nbytes);	
	return n;
}

int SerialPort::Read(uint8_t buffer[], size_t nbytes)
{
	port_config.c_cc[VMIN] = nbytes;
	port_config.c_cc[VTIME] = 0;

	tcsetattr(file_descriptor, TCSANOW, &port_config);

	if(state == CLOSED){
		throw std::logic_error("can't read from closed port.");
	}

	int n = read(file_descriptor, buffer, nbytes);
	if (n == -1) {
		throw std::runtime_error("could not read from ttyS*");
	}
	return n;
}

int SerialPort::Read(std::string &dst, size_t nbytes)
{
	port_config.c_cc[VMIN] = nbytes;
	port_config.c_cc[VTIME] = 0;
	
	char c_buffer[nbytes];

	tcsetattr(file_descriptor, TCSANOW, &port_config);

	if(state == CLOSED){
		throw std::logic_error("can't read from closed port.");
	}

	int n = read(file_descriptor, c_buffer, nbytes);
	if (n == -1) {
		throw std::runtime_error("could not read from ttyS*");
	}

	dst.assign(c_buffer, nbytes);
	return n;
}

int SerialPort::Write(const uint8_t buffer[], size_t nbytes)
{
	if(state == CLOSED){
		throw std::logic_error("can't write to closed port.");
	}

	int n = write(file_descriptor, buffer, nbytes);
	if (n == -1) {
		throw std::runtime_error("could not write to  ttyS*");
	}
	return n;
}

int SerialPort::Write(std::string buffer)
{
	if(state == CLOSED){
		throw std::logic_error("can't write to closed port.");
	}

	int n = write(file_descriptor, buffer.c_str(), buffer.length());
	if (n == -1) {
		throw std::runtime_error("could not write to  ttyS*");
	}

	return n;
}

int SerialPort::Write(std::vector<uint8_t> buffer)
{
	if(state == CLOSED){
		throw std::logic_error("can't write to closed port.");
	}

	int n = write(file_descriptor, &buffer[0], buffer.size());

	if (n == -1) {
		throw std::runtime_error("could not write to  ttyS*");
	}

	return n;
}	

void SerialPort::setDevice(const std::string device)
{
	if (state == CLOSED) {
		int fh = open(device.c_str(),O_RDONLY);

		if (fh > 0){
			this->device = device;
			close(fh);
   		} else {
			throw std::invalid_argument("device does not exist.");
		}
	} else {
		throw std::logic_error("can't change device while the port is open.");
	}
}

void SerialPort::setBaud(const Baud baud)
{
	if (state == CLOSED) {
		switch(baud){
				case B_0:
						cfsetospeed(&port_config, B0);
						cfsetispeed(&port_config, B0);
						break;
				case B_9600:
						cfsetospeed(&port_config, B9600);
						cfsetispeed(&port_config, B9600);
						break;
				case B_57600:
						cfsetospeed(&port_config, B57600);
						cfsetispeed(&port_config, B57600);
						break;
				case B_115200:
						cfsetospeed(&port_config, B115200);
						cfsetispeed(&port_config, B115200);
						break;
				default:
						cfsetospeed(&port_config, B0);
						cfsetispeed(&port_config, B0);
						break;
		}
	} else {
		throw std::logic_error("can't change baud while the port is open.");
	}
}

void SerialPort::setParity(const Parity parity)
{
	if (state == CLOSED) {
		if (parity == EVEN){
			port_config.c_cflag |= PARENB;
		} else {
			port_config.c_cflag &= ~PARENB;
		}
	} else {
		throw std::logic_error("can't change parity mode while the port is open.");
	}
}

void SerialPort::setCharSize(const CharSize cs)
{
	if (state == CLOSED) {
		switch(cs){
		case CS_8:
			port_config.c_cflag |=  CS8;
			break;
		case CS_7:
			port_config.c_cflag |=  CS7;
			break;
		case CS_6:
			port_config.c_cflag |=  CS6;
			break;
		case CS_5:
			port_config.c_cflag |=  CS5;
			break;
		default:
			throw std::invalid_argument("argument did not match any of the switch cases.");
		}

		return;

	} else {
		throw std::logic_error("can't change char size while the port is open");
	}
}

void SerialPort::setStopBits(const StopBits stopbits)
{
	if (state == CLOSED) {
		if (stopbits==STPBIT_2 ){
			port_config.c_cflag |= CSTOPB;
		} else {
			port_config.c_cflag &= ~CSTOPB;
		}
	} else {
			throw std::logic_error("can't change stop buts while port is open");
	}
}

