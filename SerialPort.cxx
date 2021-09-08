#include "SerialPort.hxx"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <regex>
#include <termios.h>
#include <sys/stat.h>


std::vector<std::string> get_serial_devices(std::string dev_dir)
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

bool SerialPort::device_exists(const std::string &device_name)
{
        struct stat buff;
        return (stat(device_name.c_str(),&buff) == 0);
}

SerialPort::SerialPort(std::string device, uint32_t baud, bytesize_t bs, parity_t parity, stopbits_t stop_bits)
{
        //Boilerplate setup for most relevant UART applications i.e communication with MCUs
        //enable local mode (no modem control lines) & receiver (reading)
        port_config.c_cflag |=  (CLOCAL | CREAD) ;      
        port_config.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software (in-band) flow-control
        port_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | ICRNL | INLCR | IGNCR | IUTF8); //ignore BREAK condition on input
        port_config.c_iflag &= ~(INPCK | IGNPAR); 
        port_config.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN); // Disables canonical mode and echo to Rx
        port_config.c_oflag &= ~OPOST; // Disables implementation-defined ouput processsing. 
        port_config.c_cflag &= ~CRTSCTS; //Disables hardware flow control lines Clear-to-Send and Ready-to-Send

        set_stop_bits(stop_bits);
        set_parity(parity);
        set_byte_size(bs);
        set_baud(baud);

        if (!device.empty()) {
                /* Try and open the device specified */
                this->device_file_descriptor = open(device.c_str(),O_RDWR);

                if (device_file_descriptor > 0){
                        this->device = device;
                } else {
                        throw std::invalid_argument("device does not exist.");
                }

                int t = tcsetattr(device_file_descriptor, TCSANOW, &port_config);

                if(t == 0){
                        this->port_state = PORTOPEN;
                } else {
                        throw std::runtime_error("Could not set port attributes");
                }
        }
        return;
}

SerialPort::~SerialPort()
{
                close(device_file_descriptor);
                return;
}

void SerialPort::open_port()
{
        /* Check if device name is set before trying to open. */
        if (device.empty()){
                throw std::runtime_error("device not set.");
        }

        if(port_state == PORTCLOSED){
                device_file_descriptor = open(device.c_str(), O_RDWR | O_NOCTTY);
                if (device_file_descriptor == -1) throw std::runtime_error("could not open device");
                fcntl(device_file_descriptor, F_SETFL, 0);
                set_baud(baud_rate);
                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
                port_state = PORTOPEN;
               
        } else {
                throw std::logic_error("port is already open.");
        }
}

void SerialPort::close_port()
{
        if(port_state == PORTOPEN){
                tcdrain(device_file_descriptor);
                cfsetspeed(&port_config, B0);
                tcsetattr(device_file_descriptor, TCSADRAIN, &port_config);
                close(device_file_descriptor);
                device_file_descriptor = 0;
                port_state = PORTCLOSED;
        } else {
                throw std::logic_error("port not open");
        }

        return;
}

int SerialPort::receive(std::vector<uint8_t> &dst, size_t nbytes)
{
        if(port_state == PORTCLOSED){
                throw std::logic_error("can't read from closed port.");
        }

        port_config.c_cc[VMIN] = nbytes;
        port_config.c_cc[VTIME] = 0;

        tcsetattr(device_file_descriptor, TCSANOW, &port_config);
        
        char c_buffer[nbytes];

        int n = read(device_file_descriptor, c_buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }

        dst.assign(c_buffer, c_buffer + nbytes);        
        return n;
}

int SerialPort::receive(uint8_t buffer[], size_t nbytes)
{
        port_config.c_cc[VMIN] = nbytes;
        port_config.c_cc[VTIME] = 0;

        tcsetattr(device_file_descriptor, TCSANOW, &port_config);

        if(port_state == PORTCLOSED){
                throw std::logic_error("can't read from closed port.");
        }

        int n = read(device_file_descriptor, buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }
        return n;
}

int SerialPort::receive(std::string &dst, size_t nbytes)
{
        if(port_state == PORTCLOSED){
                throw std::logic_error("can't read from closed port.");
        }

        port_config.c_cc[VMIN] = nbytes;
        port_config.c_cc[VTIME] = 0;
        
        char c_buffer[nbytes];

        tcsetattr(device_file_descriptor, TCSANOW, &port_config);

        int n = read(device_file_descriptor, c_buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }

        dst.assign(c_buffer, nbytes);
        return n;
}

int SerialPort::transmit(const uint8_t buffer[], size_t nbytes)
{
        if(port_state == PORTCLOSED){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(device_file_descriptor, buffer, nbytes);
        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }
        return n;
}

uint8_t SerialPort::get_byte(void)
{
        uint8_t c;

        port_config.c_cc[VMIN] = 1;
        port_config.c_cc[VTIME] = 0;

        tcsetattr(device_file_descriptor, TCSANOW, &port_config);

        if(port_state == PORTCLOSED){
                throw std::logic_error("can't read from closed port.");
        }

        int n = read(device_file_descriptor, &c, sizeof(uint8_t));
        if (n == -1) {
                throw std::runtime_error("could not read from ttyS*");
        }
        return c;
}

int SerialPort::transmit(std::string buffer)
{
        if(port_state == PORTCLOSED){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(device_file_descriptor, buffer.c_str(), buffer.length());
        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }

        return n;
}

int SerialPort::transmit(std::vector<uint8_t> buffer)
{
        if(port_state == PORTCLOSED){
                throw std::logic_error("can't write to closed port.");
        }

        int n = write(device_file_descriptor, &buffer[0], buffer.size());

        if (n == -1) {
                throw std::runtime_error("could not write to  ttyS*");
        }

        return n;
}       

void SerialPort::set_device(std::string new_device)
{
        if (device != new_device) {
                if (device_exists(new_device)){
                        if (port_state == PORTOPEN){
                                close(device_file_descriptor);
                                device_file_descriptor = open(new_device.c_str(), O_RDWR | O_NOCTTY);
                                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
                        } 
                        device = new_device;
                        
                } else {
                        throw std::invalid_argument("device does not exist");
                } 
        }
        return;
}

int SerialPort::baud_to_flags(uint32_t baud)
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

void SerialPort::set_baud(uint32_t baud)
{
        this->baud_rate = baud;
        cfsetspeed(&port_config, baud_to_flags(baud));
        if (port_state == PORTOPEN) {
                tcdrain(device_file_descriptor);
                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
        }
        return;
}

void SerialPort::set_parity(parity_t parity)
{
        if (parity == EVEN){
                port_config.c_cflag |= PARENB;
        } else {
                port_config.c_cflag &= ~PARENB;
        }
        
        if (port_state == PORTOPEN) {
                tcdrain(device_file_descriptor);
                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
        }
        return;
}

void SerialPort::set_byte_size(bytesize_t bs)
{
        switch(bs){
        case BS_8:
                port_config.c_cflag |=  CS8;
                break;
        case BS_7:
                port_config.c_cflag |=  CS7;
                break;
        case BS_6:
                port_config.c_cflag |=  CS6;
                break;
        case BS_5:
                port_config.c_cflag |=  CS5;
                break;
        default:
                throw std::invalid_argument("argument did not match any of the switch cases.");
        }
        
        if (port_state == PORTOPEN) {
                tcdrain(device_file_descriptor);
                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
        }

        return;

}

void SerialPort::set_stop_bits(stopbits_t sb)
{
        stop_bits = sb;

        if (sb==STPBIT_2 ){
                port_config.c_cflag |= CSTOPB;
        } else {
                port_config.c_cflag &= ~CSTOPB;
        }

        if (port_state == PORTOPEN) {
                tcdrain(device_file_descriptor);
                tcsetattr(device_file_descriptor, TCSANOW, &port_config);
        }
        
        return;
}

