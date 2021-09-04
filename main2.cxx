#include <iostream>
#include <ncurses.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include "SerialPort.hxx"
#include "Telegram.hxx"


constexpr int packet_repr_width = ((2+1) * 12) - 1;
const int packet_hist_height = 40;

std::vector<uint8_t> break_uint32(uint32_t x)
{
        uint8_t byte;
        std::vector<uint8_t> result;
        uint8_t i;

        for (i=0; i<4; i++){
                byte = x & 0xff;
                x >>= 8;
                result.push_back(byte);
        }

        return result;
}

int main (int argc, char* argv[])
{
        uint32_t count = 0;
        SerialPort port("/dev/ttyn0",115200,SerialPort::bs_8,SerialPort::noParity);
        
        initscr();
        start_color();
        init_pair(1,COLOR_BLACK,COLOR_GREEN);

        printw("Serial Communication Monitor\n"); 
        refresh(); 
        int height_w = getmaxy(stdscr);
        int width_w = getmaxx(stdscr);

        WINDOW* header = newwin(1,width_w,2,0);
        WINDOW* telegram_count = newwin(70,10,3,1);
        wbkgd(header, COLOR_PAIR(1));
        wclear(header);
        wprintw(header, " Telegram   ID PDU                  CHKSUM   Function      ");
        wrefresh(header);
        getch();
        scrollok(telegram_count,TRUE);
                 
        int i;
        for(i=0; i<72; i++){
                wprintw(telegram_count, "0x%0.8x", i);
                wrefresh(telegram_count);
        }


        wclear(telegram_count);
        refresh();
        
        getch();
        endwin();

        //std::vector<uint8_t> mm = break_uint32(255*255*255);
        //Telegram tlgrm(8);
        
        //std::vector<uint8_t> t = tlgrm.serialize();
        //port.Write(t);
        return 0;

}

