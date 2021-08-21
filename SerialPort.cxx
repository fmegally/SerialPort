#include "SerialPort.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <regex>
#include <termios.h>

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


SerialPort::SerialPort(std::string device, uint32_t baud, bytesize_t bs, parity_t parity, stopbits_t stopbits)
{
        //Boilerplate setup for most relevant UART applications i.e communication with MCUs
        //enable local mode (no modem control lines) & receiver (reading)
        portConfig.c_cflag |=  (CLOCAL | CREAD) ;      
        portConfig.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software (in-band) flow-control
        portConfig.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | ICRNL | INLCR | IGNCR | IUTF8); //ignore BREAK condition on input
        portConfig.c_iflag &= ~(INPCK | IGNPAR);
        portConfig.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
        portConfig.c_oflag &= ~OPOST;
        
        /* Disable RTS/CTS hardware flow control */
        portConfig.c_cflag &= ~CRTSCTS;

        /* Setup stop bits */
        if (stopbits==stpbit_2 ){
                portConfig.c_cflag |= CSTOPB;
        } else {
                portConfig.c_cflag &= ~CSTOPB;
        }

        /* Setup parity */

        if (parity == even){
                portConfig.c_cflag |= PARENB;
                portConfig.c_cflag &= ~PARODD;
        } else if (parity == odd){
                portConfig.c_cflag |= PARENB;
                portConfig.c_cflag |= PARODD;

        } else {
                portConfig.c_cflag &= ~PARENB;
                portConfig.c_cflag &= ~PARODD;
        }


        /* Byte size setting. must clear size bits before assigning
           set to 8-bit byte size */

        portConfig.c_cflag &= ~CSIZE;
        switch(bs){
        case bs_8:
                portConfig.c_cflag |=  cs8;
                break;
        case bs_7:
                portConfig.c_cflag |=  cs7;
                break;
        case bs_6:
                portConfig.c_cflag |=  cs6;
                break;
        case bs_5:
                portConfig.c_cflag |=  cs5;
                break;
        default:
                perror("error setting serial communication char size. SerialPort object not created.");
                return;
        }


        /* Setup baud rate */

        switch(baud){
        case 0: 
                this->baudRate =  0;
                cfsetispeed(&portConfig, B0);
                cfsetospeed(&portConfig, B0);
                break;

        case 1200: 
                this->baudRate =  1200;
                cfsetispeed(&portConfig, B1200);
                cfsetospeed(&portConfig, B1200);
                break;

        case 2400: 
                this->baudRate = 2400;
                cfsetispeed(&portConfig, B2400);
                cfsetospeed(&portConfig, B2400);
                break;

        case 4800: 
                this->baudRate = 4800;
                cfsetispeed(&portConfig, B4800);
                cfsetospeed(&portConfig, B4800);
                break;

        case 9600: 
                this->baudRate = 9600;
                cfsetispeed(&portConfig, B9600);
                cfsetospeed(&portConfig, B9600);
                break;

        case 19200: 
                this->baudRate = 19200;
                cfsetispeed(&portConfig, B19200);
                cfsetospeed(&portConfig, B19200);
                break;

        case 38400: 
                this->baudRate = 38400;
                cfsetispeed(&portConfig, B38400);
                cfsetospeed(&portConfig, B38400);
                break;

        case 57600: 
                this->baudRate = 57600;
                cfsetispeed(&portConfig, B57600);
                cfsetospeed(&portConfig, B57600);
                break;

        case 115200: 
                this->baudRate = 115200;
                cfsetispeed(&portConfig, B115200);
                cfsetospeed(&portConfig, B115200);
                break;

        case 250000: 
                this->baudRate = 250000;
                cfsetispeed(&portConfig, B250000);
                cfsetospeed(&portConfig, B250000);
                break;
        
        default:
                throw std::invalid_argument("baud rate not supported.");
        }

        if (!device.empty()) {
                this->deviceFileDescriptor = open(device.c_str(),O_RDWR);

                if (deviceFileDescriptor > 0){
                        this->device = device;
                } else {
                        throw std::invalid_argument("device does not exist.");
                }

                int t = tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
                if(t == 0){
                        this->state = open;
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


        /* Check if baud rate is set. */ 
        if (cfgetospeed(&portConfig)==0 && cfgetispeed(&portConfig)==0)
        {
                throw std::runtime_error("baud rate is 0");
        }

        if(state == closed){
                deviceFileDescriptor = open(device.c_str(), O_RDWR | O_NOCTTY);
                if (deviceFileDescriptor == -1){
                        throw std::runtime_error("could not open device");
                } else {
                        fcntl(deviceFileDescriptor, F_SETFL, 0);
                        tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
                        state = open;
                }
        } else {
                throw std::logic_error("port is open.");
        }
}

void SerialPort::Close()
{
        if(state == open){
                cfsetospeed(&portConfig, B0);          
                cfsetispeed(&portConfig, B0);          
                tcsetattr(deviceFileDescriptor, TCSANOW, &portConfig);
                close(deviceFileDescriptor);
                deviceFileDescriptor = 0;
                state = closed;
        } else {
                throw std::logic_error("port not open");
        }

        return;
}

int SerialPort::Read(std::vector<uint8_t> &dst, size_t nbytes)
{
        if(state == closed){
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

        if(state == closed){
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
        if(state == closed){
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
        if(state == closed){
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
        if(state == closed){
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
        if(state == closed){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(deviceFileDescriptor, &buffer[0], buffer.size());

        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }

        return n;
}       

void SerialPort::setDevice(std::string device)
{
        state_t original_state = this->state;
        if (state != closed) this->Close();

        int fh = open(device.c_str(),O_RDONLY);
        if (fh > 0){
                this->device = device;
                close(fh);
        }  else {
                throw std::invalid_argument("device does not exist.");
        }

        if (original_state == open) this->Open();
        return;
}

void SerialPort::setBaud(uint32_t baud)
{
        state_t original_state = this->state;
        if (state == closed) this->Close();

        switch(baud){
                case 0: 
                this->baudRate = 0;
                cfsetispeed(&portConfig, B0);
                cfsetospeed(&portConfig, B0);
                break;

                case 1200: 
                this->baudRate = 1200;
                cfsetispeed(&portConfig, B1200);
                cfsetospeed(&portConfig, B1200);
                break;

                case 2400: 
                this->baudRate = 2400;
                cfsetispeed(&portConfig, B2400);
                cfsetospeed(&portConfig, B2400);
                break;

                case 4800: 
                this->baudRate = 4800;
                cfsetispeed(&portConfig, B4800);
                cfsetospeed(&portConfig, B4800);
                break;

                case 9600: 
                this->baudRate = 9600;
                cfsetispeed(&portConfig, B9600);
                cfsetospeed(&portConfig, B9600);
                break;

                case 19200: 
                this->baudRate = 19200;
                cfsetispeed(&portConfig, B19200);
                cfsetospeed(&portConfig, B19200);
                break;

                case 38400: 
                this->baudRate = 38400;
                cfsetispeed(&portConfig, B38400);
                cfsetospeed(&portConfig, B38400);
                break;

                case 57600: 
                this->baudRate = 57600;
                cfsetispeed(&portConfig, B57600);
                cfsetospeed(&portConfig, B57600);
                break;

                case 115200: 
                this->baudRate = 115200;
                cfsetispeed(&portConfig, B115200);
                cfsetospeed(&portConfig, B115200);
                break;

                case 250000: 
                this->baudRate = 250000;
                cfsetispeed(&portConfig, B250000);
                cfsetospeed(&portConfig, B250000);
                break;
                
                default:
                throw std::invalid_argument("baud rate not supported.");
        
        if (original_state == open) this->Open();
        return;
}

void SerialPort::setParity(parity_t parity)
{
        state_t original_state = this->state;
        if (state == closed) this->Close();
        
        if (parity == even){
                portConfig.c_cflag |= PARENB;
        } else {
                portConfig.c_cflag &= ~PARENB;
        }

        if (original_state == open) this->Open();
        return;
}

void SerialPort::setCharSize(bytesize_t cs)
{
        state_t original_state = this->state;
        if (state == closed) this->Close();

        switch(cs){
        case CS_8:
                portConfig.c_cflag |=  CS8;
                break;
        case CS_7:
                portConfig.c_cflag |=  CS7;
                break;
        case CS_6:
                portConfig.c_cflag |=  CS6;
                break;
        case CS_5:
                portConfig.c_cflag |=  CS5;
                break;
        default:
                throw std::invalid_argument("argument did not match any of the switch cases.");
        }

        if (original_state == open) this->Open();
        return;

}

void SerialPort::setStopBits(stopbits_t stopbits)
{
        state_t original_state = this->state;
        if (state == closed) this->Close();
        
        if (stopbits==stpbit_2 ){
                portConfig.c_cflag |= CSTOPB;
        } else {
                portConfig.c_cflag &= ~CSTOPB;
        }

        if (original_state == open) this->Open();
        return;
}

