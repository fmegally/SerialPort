#include <iostream>
#include <unistd.h>
#include "SerialPort.h"

int main (int argc, char* argv[])
{
        SerialPort p("/dev/ttyn0",115200,SerialPort::bs_8,SerialPort::noParity);
        p.Write("Oh hello world!\n\r");
        p.Write("The quick brown fox jumped over the lazy dog\n\r");
        p.Close();
        
        
        return 0;

}

