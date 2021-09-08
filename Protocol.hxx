#include <array>
#include <string>
#include "SerialPort.hxx"
#include "Telegram.hxx"


/* Design notes:
        The controller PC is always master driving the communication. The boards can only
        reply to and fullfil requests. Therefore the protocol will not have to handle 
        asynchronus messages.
*/


template <int size>
struct Telegram
{
        uint8_t id;
        uint8_t data[size];
};

class Protocol
{
        private:
        static const uint8_t SD = 0xAA;
        static const uint8_t ED = 0x55;
        static const uint8_t PDU_SIZE = 8;
 
        enum states{
                START,
                FLAGS,
                ID,
                PDU,
                FCS,
                ED,
                HALT
        };
   
        enum flag_bits{
                   CTL = (1 << 0), //Control Flag - No payload if enabled
                   SYN = (1 << 1), //Sync - check if board is online
                   RTS = (1 << 2), //Request to send
                   CTS = (1 << 3), //Clear to send
                   ACK = (1 << 4), // 
                   ERR = (1 << 5),
                   RST = (1 << 6)
        };
   
        struct Frame {
                uint8_t sd;
                uint8_t flags;
                Telegram<PDU_SIZE> telegram;
                uint8_t fcs;
                uint8_t ed;
        };


        SerialPort& port;
        struct Frame rx_frame_buffer;
        struct Frame tx_frame_buffer;

        Telegram<PDU_SIZE> fetch_telegram();

        public:
        Protocol();
        Protocol(SerialPort& serial_port);
        ~Protocol();
                
        void assign_port(SerialPort& serial_port);
        int send_telegram(Telegram<PDU_SIZE> tx_tlgrm, void (*rxCallback)(Telegram<PDU_SIZE> reply) = NULL) ;
};