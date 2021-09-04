#include <unistd.h>
#include <stdio.h>
#include <ncurses.h>
#include <thread>

/*
+---------------------------------------------------------------+
|Serial Communication Monitor / Interface                       |
+---------------------------------------------------------------+
+---------------------------------------------------------------++---------------------------------------------------------------+
|                                                               ||                                                               |
|Num.    ID           PDU            CHK         Function       ||Num.    ID           PDU            CHK         Function       |
+---------------------------------------------------------------++---------------------------------------------------------------+
+------++--++-----------------------++--++----------------------++------++--++-----------------------++--++----------------------+
|0x0000||11||22 33 44 55 66 77 88 99||AA||                      ||0x0000||11||22 33 44 55 66 77 88 99||AA||                      |

*/

int colornum(int fg, int bg)
{
        int B, bbb, ffff;
        B = 0 << 7;
        bbb = (7 & bg) << 4;
        ffff = 7 & fg;
        return (B | bbb | ffff);
}

class UserInterface
{
        private:
                int toplevel_w ;
                int toplevel_h ;
                int title_h = 1 ;
                int title_w ;
                int headerbar_h = 2 ;
                int headerbar_w = (toplevel_w - 2) / 2 ;
                int statusbar_h =  1 ;
                int statusbar_w = toplevel_w ; 
                
                uint32_t rx_telegrams_counter ;
                uint32_t tx_telegrams_counter ;

                WINDOW* title ;

                WINDOW* rx_headerbar ;
                WINDOW* rx_telegram_num ;       
                WINDOW* rx_telegram_id ;
                WINDOW* rx_telegram_data ;
                WINDOW* rx_telegram_chksum ;

                WINDOW* tx_headerbar ; 
                WINDOW* tx_telegram_num ;
                WINDOW* tx_telegram_id;
                WINDOW* tx_telegram_data ;
                WINDOW* tx_telegram_chksum ;

                WINDOW* status_bar ;

        public:
                UserInterface();
                ~UserInterface();
                void printRxTelegram();
                void printTxTelegram();
                int getUserCommand();

};


int UserInterface::getUserCommand()
{
        char c = wgetch(status_bar);
        if (c == 'k') {
                rx_telegrams_counter++;
                return 0;
        } else if (c == 'j') {
                rx_telegrams_counter--;
                return 0;
        } else {
                return -1;
        }
}


UserInterface::UserInterface()
{
        enum 
        { rx_header_colors = 1,
          tx_header_colors = 2,
          rx_telegram_num_colors = 3,
          rx_telegram_data_colors = 4,
          tx_telegram_num_colors = 5,
          tx_telegram_data_colors = 6
        };

          
        initscr();
        start_color();

        init_pair(rx_header_colors, COLOR_BLACK, COLOR_GREEN);
        init_pair(tx_header_colors, COLOR_BLACK, COLOR_CYAN);

        init_pair(rx_telegram_num_colors, COLOR_WHITE, COLOR_BLACK);
        init_pair(rx_telegram_data_colors, COLOR_YELLOW, COLOR_BLACK);
        init_pair(tx_telegram_num_colors, COLOR_WHITE, COLOR_BLACK );
        init_pair(tx_telegram_data_colors, COLOR_YELLOW, COLOR_BLACK);


        toplevel_h = getmaxy(stdscr);
        toplevel_w = getmaxx(stdscr);

        title = newwin(title_h, title_w,1,1);

        rx_headerbar = newwin(headerbar_h, 66,3,1);
        rx_telegram_num = newwin(toplevel_h - title_h - statusbar_h - 2, 6, 6, 1);
        rx_telegram_id = newwin(toplevel_h - title_h - statusbar_h - 2, 2, 6,9 );
        rx_telegram_data = newwin(toplevel_h - title_h - statusbar_h - 2, 24, 6,13 );
        rx_telegram_chksum = newwin(toplevel_h - title_h - statusbar_h - 2, 2, 6,38 );
        
        tx_headerbar = newwin(headerbar_h, 66,3,70);
        tx_telegram_num = newwin(toplevel_h - title_h - statusbar_h - 2, 6, 6, 70);
        tx_telegram_id = newwin(toplevel_h - title_h - statusbar_h - 2, 2, 6,78 );
        tx_telegram_data = newwin(toplevel_h - title_h - statusbar_h - 2, 24, 6,82 );
        tx_telegram_chksum = newwin(toplevel_h - title_h - statusbar_h - 2, 2, 6,82 + 25);
        
        wprintw(title,"Serial Communication Interface");
        wrefresh(title);

        wclear(rx_headerbar); 
        wbkgd(rx_headerbar,COLOR_PAIR(tx_header_colors));
        wattron(rx_headerbar, A_BOLD);
        waddstr(rx_headerbar, "                          RX Telegrams                      \n ");
        wattroff(rx_headerbar, A_BOLD);
        waddstr(rx_headerbar, "Num.   ID           PDU             CHK         Function      ");
        wrefresh(rx_headerbar);
       
        wclear(tx_headerbar); 
        wbkgd(tx_headerbar,COLOR_PAIR(rx_header_colors));
        wattron(tx_headerbar, A_BOLD);
        waddstr(tx_headerbar, "                          TX Telegrams                      \n ");
        wattroff(tx_headerbar, A_BOLD);
        waddstr(tx_headerbar, "Num.   ID           PDU             CHK         Function      ");
        wrefresh(tx_headerbar);

        status_bar = newwin(statusbar_h, toplevel_w, toplevel_h,0);


//        wbkgd(rx_telegram_num,COLOR_PAIR(rx_telegram_num_colors));
        wclear(rx_telegram_num);
        
 //       wbkgd(rx_telegram_id,COLOR_PAIR(rx_telegram_data_colors));
        wclear(rx_telegram_id);
        
        wclear(tx_telegram_data);
//        wbkgd(tx_telegram_data,COLOR_PAIR(tx_telegram_data_colors));

        wclear(tx_telegram_chksum);
//        wbkgd(tx_telegram_data,COLOR_PAIR(tx_telegram_data_colors));
        
        curs_set(0);
        scrollok(rx_telegram_num, TRUE);
        scrollok(rx_telegram_data, TRUE);
        scrollok(tx_telegram_num, TRUE);
        scrollok(tx_telegram_data, TRUE);

        rx_telegrams_counter = 0;
        tx_telegrams_counter = 0;

        return;
}

UserInterface::~UserInterface()
{
        endwin();
        delwin(title);

        delwin( rx_headerbar );
        delwin( rx_telegram_num );
        delwin( rx_telegram_data );

        delwin( tx_headerbar );
        delwin( tx_telegram_num );
        delwin( tx_telegram_data );
        return;
}

void UserInterface::printRxTelegram()
{
        wprintw( rx_telegram_num, "0x%.4x",rx_telegrams_counter);
        wprintw( rx_telegram_id, "%.2x",rx_telegrams_counter+1);
        for (int i = 0; i < 8; i++) wprintw( rx_telegram_data, "%.2x ",rx_telegrams_counter);
        wprintw( rx_telegram_chksum, "%.2x",rx_telegrams_counter);
        wrefresh( rx_telegram_num );
        wrefresh( rx_telegram_id );
        wrefresh( rx_telegram_data );
        wrefresh( rx_telegram_chksum );
        rx_telegrams_counter++;
        return;
}


void UserInterface::printTxTelegram()
{
        wprintw( tx_telegram_num, "0x%.4x",tx_telegrams_counter);
        wprintw( tx_telegram_id, "%.2x",tx_telegrams_counter+1);
        for (int i = 0; i < 8; i++) wprintw( tx_telegram_data, "%.2x ",tx_telegrams_counter);
        wprintw( tx_telegram_chksum, "%.2x",tx_telegrams_counter);
        wrefresh( tx_telegram_num );
        wrefresh( tx_telegram_id );
        wrefresh( tx_telegram_data );
        wrefresh( tx_telegram_chksum );
        tx_telegrams_counter++;
        return;
}

int main (int argc, char* argv[])
{
        UserInterface *main_interface = new UserInterface;
        int c;
        std::thread handle_input([=](){
                                        while(1) {
                                                int r = main_interface->getUserCommand();
                                                if (r != 0) break;}
                                      }
                                 );

        std::thread display_stuff([&c,&main_interface](){
                for (c = 0; c < 40; c++) {
                        main_interface->printRxTelegram();
                        main_interface->printTxTelegram();
                        usleep(1000000);
                };
        });

        display_stuff.join();
        handle_input.join();

        delete main_interface;

        return 0;
}


