#include <SerialPort.hxx>
#include <Telegram.hxx>
#include <string>

/* Design notes:
        The controller PC is always master driving the communication. The boards can only
        reply to and fullfil requests. Therefore the protocol will not have to handle 
        asynchronus messages.
*/

class Protocol
{
        private:
                enum {
                        FSM_SD,
                        FSM_FLAGS,
                        FSM_ID,
                        FSM_PDU,
                        FSM_FCS,
                        FSM_ED,
                        FSM_HALT
                };

                enum {
                        CTRL_FRAME = (1 << 0);
                        SYN = (1 << 1),
                        RTS = (1 << 2),
                        CTS = (1 << 3),
                        ACK = (1 << 4),
                        ERR = (1 << 5),
                        RST = (1 << 6)
                };

                static const uint8_t SD = 0xAA;
                static const uint8_t ED = 0x55;
                static const uint8_t PDU_Size = 8;

                bool clearToSend;
                bool replyAvailable;

                SerialPort& port;
                Telegram rx_tlgrm_buffer(PDU_Size);
                Telegram tx_tlgrm_buffer(PDU_Size);
                
                void (*rxCallback)(Telegram rxed_tlgrm);
                void (*txCallback)(Telegram rxed_tlgrm);

        public:
                Protocol(const SerialPort& serial_port);
                ~Protocol();
                
                void assignPort(SerialPort& serial_port);
                void assignRxCallback(void (*func)(Telegram rxed_tlgrm));
                void assignTxCallback(void (*func)(Telegram txed_tlgrm)):
                int sendTelegram(Telegram tx_tlgrm, void (*rxCallback)(Telegram reply) = NULL) ;
                 
                


                      
              
