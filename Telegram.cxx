#include "Telegram.hxx"


void Telegram::calcChksum(void)
{
        uint16_t acc = 0;
        uint8_t i;

        acc += id;
	for(auto i:data) acc += i;
	acc = (acc & 0xFF) + (acc >> 8);
	chksum =  ~acc;

        return;
}

int Telegram::checkTelegram(void)
{
        uint16_t acc = 0;
        uint8_t i;

	acc += id;
	for(auto i:data) acc += i;
	acc += chksum;
	
        acc = (acc & 0xFF) + (acc >> 8);
        return ~acc;
}

Telegram::Telegram(uint8_t size)
{
        id = 0xAA;
        data = std::vector<uint8_t>(size);
        calcChksum();
        return;
}

Telegram::Telegram(uint8_t id, std::vector<uint8_t> data){
        this->id = id;
        this->data = data;
        calcChksum();
        return;
}

Telegram::Telegram(std::vector<uint8_t> bytes){
        this->id = bytes[0];
        this->data = std::vector<uint8_t>(bytes.begin() + 1, bytes.end());
        calcChksum();
}

std::vector<uint8_t> Telegram::serialize(void)
{
        std::vector<uint8_t> bytes{id,chksum};
        bytes.insert(bytes.begin() + 1, data.begin(), data.end());
        return bytes;
}


void Telegram::assign(std::vector<uint8_t> &bytes)
{
        this->data = bytes;
        calcChksum();
        return;
}


void Telegram::assign(uint8_t id){
        this->id = id;
        calcChksum();
        return;
}        

void Telegram::assign(uint8_t id,std::vector<uint8_t> &bytes)
{
        this->id = id;
        this->data = bytes;
        calcChksum();
        return;
}

