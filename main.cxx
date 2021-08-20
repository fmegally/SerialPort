#include <iostream>
#include "SerialPort.h"


int printMulti(int a, int b, int c = 100, int d = 1000, int e =10000){
        std::cout << "a : " << a << std::endl;
        std::cout << "b : " << b << std::endl;
        std::cout << "c : " << c << std::endl;
        std::cout << "d : " << d << std::endl;
        std::cout << "e : " << e << std::endl;
        return 0;
}

int main (int argc, char* argv[])
{
        SerialPort p("/dev/ttyn0");
        printMulti(2,4,400);
        p.Connect();
        
        return 0;

}

