#include <SerialPort.hxx>
#include <Telegram.hxx>
#include <string>

class Protocol
{
        private:
                static const uint8_t SD = 0xAA;
                static const uint8_t ED = 0x55;
                static const uint8_t PDU_Size = 8;

                enum sc_state {
                        FSM_SD,
                        FSM_ID,
                        FSM_PDU,
                        FSM_FCS,
                        FSM_ED,
                        FSM_HALT
                };

                enum telegram_id {
                        NULL_ID,
                        ECHO,
                        ACK=0x06,
                        CONFIRM=0x07,
                        GPIO_CMD=0x08,
                        ADC_CMD=0x09,
                        NAK=0x15,
                        EXCEPTION,0x17,
                        ID_TABLE_SIZE
                };
                
                Telegram rx_telegram(PDU_Size);
                Telegram tx_telegram(PDU_Size);

                SerialPort port;

                int sendCommand(bool waitForReply);

        public:
                Interface(std::string serial_device);
                ~Interface();

                sendCommand



                      
              
