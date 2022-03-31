#ifndef __NFC_H_
#define __NFC_H_

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>

class NFC
{
public:
  NFC(uint8_t power_pin, void (*printDebug)(const String&, const String&), void (*printFault)(const String&, const String&));

  bool powerOn();
  void powerOff();

  // Authenticates and gets a block of data. Assumes an RFID tag is in proximity
  bool readBlock(uint8_t block, uint8_t (&data)[16]);

  bool writeBlock(uint8_t block, uint8_t data[16]);

  bool isPowered();

private:
  bool powered = false;

  uint8_t power_pin;

  PN532_I2C* pn532i2c = nullptr;
  PN532* nfc_lib = nullptr;

  bool authBlock(uint8_t block);

  void (*printDebug)(const String&, const String&);
  void (*printFault)(const String&, const String&);
};

#endif