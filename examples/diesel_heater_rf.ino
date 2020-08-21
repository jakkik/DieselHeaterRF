/*
 * 
 *    ESP32         CC1101
 *    -----         ------
 *    4   <-------> GDO2
 *    18  <-------> SCK
 *    3v3 <-------> VCC
 *    23  <-------> MOSI
 *    19  <-------> MISO
 *    5   <-------> CSn
 *    GND <-------> GND
 * 
 */

#include <SPI.h>
#include "cc1101_heater.h"

#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define SS_PIN    5
#define GDO2_PIN   4

#define HEATER_POLL_INTERVAL  500

uint32_t heaterAddr = 0x56d24eae;

uint8_t sState       = 0;
uint8_t sPower       = 0;
float sVoltage       = 0;
int8_t sAmbientTemp  = 0;
int8_t sCaseTemp     = 0;
int8_t sSetpoint     = 0;
uint8_t sAutoMode    = 0;
float sPumpFreq      = 0;
int16_t sRssi        = 0;

CC1101_Heater heater(SCK_PIN, MOSI_PIN, MISO_PIN, SS_PIN, GDO2_PIN);

void setup() {

  Serial.begin(115200);

  heater.begin(heaterAddr);

  // Test reading the CC1101 version number
  uint8_t tmp = heater.writeReg(0xF1, 0xFF);
  Serial.printf("CC1101 version is %d\n", tmp);

  // Transmit a wake-up message (twice, even though the real remote does it three times)
  for (int i = 0; i < 2; i++) {
    heater.sendCommand(HEATER_CMD_WAKEUP, heaterAddr, 15);
  }

  while (1) {

    heater.sendCommand(HEATER_CMD_WAKEUP, heaterAddr, 10);

    if (heater.getState(&sState, &sPower, &sVoltage, &sAmbientTemp, &sCaseTemp, &sSetpoint, &sPumpFreq, &sAutoMode, &sRssi, 1000)) {
      Serial.printf("State: %d, Power: %d, Voltage: %f, Ambient: %d, Case: %d, Setpoint: %d, PumpFreq: %f, Auto: %d, RSSI: %d\n", sState, sPower, sVoltage, sAmbientTemp, sCaseTemp, sSetpoint, sPumpFreq, sAutoMode, sRssi); 
    } else {
      Serial.println("Failed to get the state");
    }
    
    delay(HEATER_POLL_INTERVAL + 3000UL);

  }
  
}

void loop() {

}
