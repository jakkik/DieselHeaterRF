#ifndef DieselHeaterRF_h
#define DieselHeaterRF_h

#include <Arduino.h>

#define HEATER_SCK_PIN   18
#define HEATER_MISO_PIN  19
#define HEATER_MOSI_PIN  23
#define HEATER_SS_PIN    5
#define HEATER_GDO2_PIN  4

#define HEATER_CMD_WAKEUP 0x23
#define HEATER_CMD_MODE   0x24
#define HEATER_CMD_POWER  0x2b
#define HEATER_CMD_UP     0x3c
#define HEATER_CMD_DOWN   0x3e

#define HEATER_STATE_OFF            0x00
#define HEATER_STATE_STARTUP        0x01
#define HEATER_STATE_WARMING        0x02
#define HEATER_STATE_WARMING_WAIT   0x03
#define HEATER_STATE_PRE_RUN        0x04
#define HEATER_STATE_RUNNING        0x05
#define HEATER_STATE_SHUTDOWN       0x06
#define HEATER_STATE_SHUTTING_DOWN  0x07
#define HEATER_STATE_COOLING        0x08

#define HEATER_TX_REPEAT    10 // Number of times to re-transmit command packets
#define HEATER_RX_TIMEOUT   5000

typedef struct {
  uint8_t state       = 0;
  uint8_t power       = 0;
  float voltage       = 0;
  int8_t ambientTemp  = 0;
  uint8_t caseTemp    = 0;
  int8_t setpoint     = 0;
  uint8_t autoMode    = 0;
  float pumpFreq      = 0;
  int16_t rssi        = 0;
} heater_state_t;

class DieselHeaterRF
{

    public:

        DieselHeaterRF() { 
            _pinSck = HEATER_SCK_PIN;
            _pinMiso = HEATER_MISO_PIN;
            _pinMosi = HEATER_MOSI_PIN;
            _pinSs = HEATER_SS_PIN;
            _pinGdo2 = HEATER_GDO2_PIN;
        }

        DieselHeaterRF(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t gdo2) { 
            _pinSck = sck;
            _pinMiso = miso;
            _pinMosi = mosi;
            _pinSs = ss;
            _pinGdo2 = gdo2;
        }
        
        ~DieselHeaterRF() {
        }        

        void begin();
        void begin(uint32_t heaterAddr);

        void setAddress(uint32_t heaterAddr);
        bool getState(heater_state_t *state);
        bool getState(heater_state_t *state, uint32_t timeout);
        bool getState(uint8_t *state, uint8_t *power, float *voltage, int8_t *ambientTemp, uint8_t *caseTemp, int8_t *setpoint, float *pumpFreq, uint8_t *autoMode, int16_t *rssi, uint32_t timeout);
        void sendCommand(uint8_t cmd);
        void sendCommand(uint8_t cmd, uint32_t addr);
        void sendCommand(uint8_t cmd, uint32_t addr, uint8_t numTransmits);
        uint32_t findAddress(uint16_t timeout);

    private:

        uint8_t _pinSck;
        uint8_t _pinMiso;
        uint8_t _pinMosi;
        uint8_t _pinSs;
        uint8_t _pinGdo2;

        uint32_t _heaterAddr = 0;
        uint8_t _packetSeq = 0;

        void initRadio();

        void txBurst(uint8_t len, char *bytes);
        void txFlush();

        void rx(uint8_t len, char *bytes);
        void rxFlush();
        void rxEnable();  

        uint8_t writeReg(uint8_t addr, uint8_t val);
        void writeBurst(uint8_t addr, uint8_t len, char *bytes);
        void writeStrobe(uint8_t addr);

        void spiStart(void);
        void spiEnd(void);

        bool receivePacket(char *bytes, uint16_t timeout);
        uint32_t parseAddress(char *buf);

        uint16_t crc16_2(char *buf, int len);

};

#endif
