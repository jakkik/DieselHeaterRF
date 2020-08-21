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

#include "DieselHeaterRF.h"

#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define SS_PIN    5
#define GDO2_PIN   4

#define HEATER_POLL_INTERVAL  4000

uint32_t heaterAddr = 0x56d24eae; // Heater address is a 32 bit unsigned int. Use the findAddress() to get your heater's address.

uint8_t sState       = 0;
uint8_t sPower       = 0;
float sVoltage       = 0;
int8_t sAmbientTemp  = 0;
int8_t sCaseTemp     = 0;
int8_t sSetpoint     = 0;
uint8_t sAutoMode    = 0;
float sPumpFreq      = 0;
int16_t sRssi        = 0;

DieselHeaterRF heater(SCK_PIN, MOSI_PIN, MISO_PIN, SS_PIN, GDO2_PIN);

void setup() {

  Serial.begin(115200);

  heater.begin(heaterAddr);

  Serial.println("Started pairing, press and hold the DOWN button on the heater's LCD panel...");

  uint32_t address = heater.findAddress(60000UL);

  if (address) {
    heaterAddr = address;
    Serial.print("Got address: ");
    Serial.println(address, HEX);
    // Store the address somewhere, eg. NVS
  } else {
    Serial.println("Failed to find a heater");   
    while(1) {}
  }
  
}

void loop() {

  heater.sendCommand(HEATER_CMD_WAKEUP, heaterAddr, 10);

  if (heater.getState(&sState, &sPower, &sVoltage, &sAmbientTemp, &sCaseTemp, &sSetpoint, &sPumpFreq, &sAutoMode, &sRssi, 1000)) {
    Serial.printf("State: %d, Power: %d, Voltage: %f, Ambient: %d, Case: %d, Setpoint: %d, PumpFreq: %f, Auto: %d, RSSI: %d\n", sState, sPower, sVoltage, sAmbientTemp, sCaseTemp, sSetpoint, sPumpFreq, sAutoMode, sRssi); 
  } else {
    Serial.println("Failed to get the state");
  }
  
  delay(HEATER_POLL_INTERVAL);

}