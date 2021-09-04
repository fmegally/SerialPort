#include <stdint.h>
#include <vector>
#include <string>

class Telegram {
        private:
        uint8_t id;
        std::vector<uint8_t> data;
	uint8_t chksum;

        void calcChksum(void);
        int checkTelegram(void);
    
        public:
        Telegram(uint8_t size);
        Telegram(uint8_t id, std::vector<uint8_t> data);
        Telegram(std::vector<uint8_t> bytes);
        std::vector<uint8_t> serialize(void);
        void assign(std::vector<uint8_t> &bytes);
        void assign(uint8_t id);
        void assign(uint8_t id, std::vector<uint8_t> &bytes);
};

