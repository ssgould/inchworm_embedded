#ifndef __NFC_H_
#define __NFC_H_

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>

class NFC
{
public:
  NFC(uint8_t power_pin, void (*printDebug)(String), void (*printFault)(String));
  ~NFC();

  bool powerOn();
  void powerOff();

  // Authenticates and gets a block of data. Assumes an RFID tag is in proximity
  bool getBlock(uint8_t block, uint8_t (&data)[16]);

private:
  bool powered = false;

  uint8_t power_pin;

  PN532_I2C* pn532i2c = nullptr;
  PN532* nfc_lib = nullptr;

  void (*printDebug)(String);
  void (*printFault)(String);
};

#endif