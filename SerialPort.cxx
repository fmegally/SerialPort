#include "SerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <regex>
#include <termios.h>
#include <sys/stat.h>


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

bool SerialPort::deviceExists(const std::string &device_name)
{
        struct stat buff;
        return (stat(device_name.c_str(),&buff) == 0);
}

SerialPort::SerialPort(std::string device, uint32_t baud, bytesize_t bs, parity_t parity, stopbits_t stopbits)
{
        //Boilerplate setup for most relevant UART applications i.e communication with MCUs
        //enable local mode (no modem control lines) & receiver (reading)
        portConfig.c_cflag |=  (CLOCAL | CREAD) ;      
        portConfig.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software (in-band) flow-control
        portConfig.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | ICRNL | INLCR | IGNCR | IUTF8); //ignore BREAK condition on input
        portConfig.c_iflag &= ~(INPCK | IGNPAR); 
        portConfig.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN); // Disables canonical mode and echo to Rx
        portConfig.c_oflag &= ~OPOST; // Disables implementation-defined ouput processsing. 
        portConfig.c_cflag &= ~CRTSCTS; //Disables hardware flow control lines Clear-to-Send and Ready-to-Send

        setStopBits(stopbits);
        setParity(parity);
        setByteSize(bs);
        setBaud(baud);

        if (!device.empty()) {
                /* Try and open the device specified */
                this->deviceFileDescriptor = open(device.c_str(),O_RDWR);

                if (deviceFileDescriptor > 0){
                        this->device = device;
                } else {
                        throw std::invalid_argument("device does not exist.");
                }

                int t = tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);

                if(t == 0){
                        this->portState = portOpen;
                } else {
                        throw std::runtime_error("Could not set port attributes");
                }
        }
        return;
}

SerialPort::~SerialPort()
{
                close(deviceFileDescriptor);
                return;
}

void SerialPort::Open()
{
        /* Check if device name is set before trying to open. */
        if (device.empty()){
                throw std::runtime_error("device not set.");
        }

        if(portState == portClosed){
                deviceFileDescriptor = open(device.c_str(), O_RDWR | O_NOCTTY);
                if (deviceFileDescriptor == -1) throw std::runtime_error("could not open device");
                fcntl(deviceFileDescriptor, F_SETFL, 0);
                setBaud(baudRate);
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
                portState = portOpen;
               
        } else {
                throw std::logic_error("port is already open.");
        }
}

void SerialPort::Close()
{
        if(portState == portOpen){
                tcdrain(deviceFileDescriptor);
                cfsetspeed(&portConfig, B0);
                tcsetattr(deviceFileDescriptor, TCSADRAIN, &portConfig);
                close(deviceFileDescriptor);
                deviceFileDescriptor = 0;
                portState = portClosed;
        } else {
                throw std::logic_error("port not open");
        }

        return;
}

int SerialPort::Read(std::vector<uint8_t> &dst, size_t nbytes)
{
        if(portState == portClosed){
                throw std::logic_error("can't read from closed port.");
        }

        portConfig.c_cc[VMIN] = nbytes;
        portConfig.c_cc[VTIME] = 0;

        tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
        
        char c_buffer[nbytes];

        int n = read(deviceFileDescriptor, c_buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }

        dst.assign(c_buffer, c_buffer + nbytes);        
        return n;
}

int SerialPort::Read(uint8_t buffer[], size_t nbytes)
{
        portConfig.c_cc[VMIN] = nbytes;
        portConfig.c_cc[VTIME] = 0;

        tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);

        if(portState == portClosed){
                throw std::logic_error("can't read from closed port.");
        }

        int n = read(deviceFileDescriptor, buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }
        return n;
}

int SerialPort::Read(std::string &dst, size_t nbytes)
{
        if(portState == portClosed){
                throw std::logic_error("can't read from closed port.");
        }

        portConfig.c_cc[VMIN] = nbytes;
        portConfig.c_cc[VTIME] = 0;
        
        char c_buffer[nbytes];

        tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);

        int n = read(deviceFileDescriptor, c_buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }

        dst.assign(c_buffer, nbytes);
        return n;
}

int SerialPort::Write(const uint8_t buffer[], size_t nbytes)
{
        if(portState == portClosed){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(deviceFileDescriptor, buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }
        return n;
}

int SerialPort::Write(std::string buffer)
{
        if(portState == portClosed){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(deviceFileDescriptor, buffer.c_str(), buffer.length());
        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }

        return n;
}

int SerialPort::Write(std::vector<uint8_t> buffer)
{
        if(portState == portClosed){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(deviceFileDescriptor, &buffer[0], buffer.size());

        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }

        return n;
}       

void SerialPort::setDevice(std::string new_device)
{
        if (device != new_device) {
                if (deviceExists(new_device)){
                        if (portState == portOpen){
                                close(deviceFileDescriptor);
                                deviceFileDescriptor = open(new_device.c_str(), O_RDWR | O_NOCTTY);
                                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
                        } 
                        device = new_device;
                        
                } else {
                        throw std::invalid_argument("device does not exist");
                } 
        }
        return;
}

int SerialPort::convBaudToFlags(uint32_t baud)
{
        switch(baud)
                {
                        case 1200: 
                        return B1200;
                        break;

                        case 2400: 
                        return  B2400;
                        break;

                        case 4800: 
                        return B4800;
                        break;

                        case 9600: 
                        return B9600;
                        break;

                        case 19200: 
                        return B19200;
                        break;

                        case 38400: 
                        return B38400;
                        break;

                        case 57600: 
                        return B57600;
                        break;

                        case 115200: 
                        return B115200;
                        break;

                        default:
                        throw std::invalid_argument("baud rate not supported.");
                }
}

void SerialPort::setBaud(uint32_t baud)
{
        this->baudRate = baud;
        cfsetspeed(&portConfig, convBaudToFlags(baud));
        if (portState == portOpen) {
                tcdrain(deviceFileDescriptor);
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
        }
        return;
}

void SerialPort::setParity(parity_t parity)
{
        if (parity == even){
                portConfig.c_cflag |= PARENB;
        } else {
                portConfig.c_cflag &= ~PARENB;
        }
        
        if (portState == portOpen) {
                tcdrain(deviceFileDescriptor);
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
        }
        return;
}

void SerialPort::setByteSize(bytesize_t bs)
{
        switch(bs){
        case bs_8:
                portConfig.c_cflag |=  CS8;
                break;
        case bs_7:
                portConfig.c_cflag |=  CS7;
                break;
        case bs_6:
                portConfig.c_cflag |=  CS6;
                break;
        case bs_5:
                portConfig.c_cflag |=  CS5;
                break;
        default:
                throw std::invalid_argument("argument did not match any of the switch cases.");
        }
        
        if (portState == portOpen) {
                tcdrain(deviceFileDescriptor);
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
        }

        return;

}

void SerialPort::setStopBits(stopbits_t stopbits)
{
        if (stopbits==stpbit_2 ){
                portConfig.c_cflag |= CSTOPB;
        } else {
                portConfig.c_cflag &= ~CSTOPB;
        }

        if (portState == portOpen) {
                tcdrain(deviceFileDescriptor);
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
        }
        
        return;
}

