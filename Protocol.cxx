#include "Protocol.hxx"
#include <stdint.h>




Protocol::Protocol(SerialPort& serial_port):
        port(serial_port)
{
        return;
}

Telegram<Protocol::PDU_SIZE> Protocol::fetch_telegram()
{
        uint8_t state = START;
        uint8_t ch_buffer;
        uint8_t n = PDU_SIZE;
        
}

int Protocol::send_telegram(Telegram<PDU_SIZE> tx_tlgrm, void (*rxCallback)(Telegram<PDU_SIZE> reply))
{
        port.transmit((uint8_t*)(&tx_tlgrm),sizeof(tx_tlgrm));
        Telegram<PDU_SIZE> rx_tlgrm = fetch_telegram();
        if ()




