#include <nfc.h>

NFC::NFC(uint8_t power_pin, void (*printDebug)(const String&, const String&), void (*printFault)(const String&, const String&)) {
  this->power_pin = power_pin;
  this->pn532i2c = new PN532_I2C(Wire);
  this->printDebug = printDebug;
  this->printFault = printFault;

  this->nfc_lib = new PN532(*pn532i2c);

  this->nfc_lib->begin();

  uint32_t versiondata = this->nfc_lib->getFirmwareVersion();

  this->printDebug("Got firmware version", "");

  if(!versiondata) {
    this->printFault("Failed to get NFC version data.", "");
  }

  this->printDebug("Got NFC version data", "");

  this->nfc_lib->SAMConfig();
}

bool NFC::powerOn() {
  this->printDebug("Powering on board", "");
  digitalWrite(this->power_pin, HIGH);

  if(this->nfc_lib != nullptr) {
    delete this->nfc_lib;
  }

  this->printDebug("Constructing PN532 object", "");

  

  this->printDebug("Constructed", "");

  //this->printDebug("begin() caled", "");

  // Maybe add this->nfc_lib->begin(), but that may reset the whole Wire lib
  // (also used for Encoders)


  // TODO: May not be necessary
  // delay(250);

  this->printDebug("Getting firmware version", "");

  uint32_t versiondata = this->nfc_lib->getFirmwareVersion();

  this->printDebug("Got firmware version", "");

  if(!versiondata) {
    this->printFault("Failed to get NFC version data.", "");
    return false;
  }

  this->printDebug("Got NFC version data", "");

  this->nfc_lib->SAMConfig();
  powered = true;

  return true;
}

void NFC::powerOff() {
  this->printDebug("Powering off board", "");
  if(this->nfc_lib != nullptr)
    delete this->nfc_lib;

  digitalWrite(this->power_pin, LOW);

  powered = false;
}

bool NFC::authBlock(uint8_t block) {
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;
  uint8_t key_universal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  this->printDebug("Initialized uint8_t's", "");

  success = this->nfc_lib->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 2000);

  if(!success)
    return false;

  this->printDebug("Read passive target id", "");

  success = this->nfc_lib->mifareclassic_AuthenticateBlock(uid, uidLength, block, 1, key_universal);

  if(!success)
    return false;

  return true;
}

bool NFC::readBlock(uint8_t block, uint8_t (&data)[16]) {
  bool success = authBlock(block);
  //

  if(!success)
    return false;

  this->printDebug("Authenticated block", "");

  uint8_t success_t = this->nfc_lib->mifareclassic_ReadDataBlock(block, data, 1000);
  

  if(!success_t)
    return false;

  this->printDebug("Read success", "");

  return true;
}

bool NFC::writeBlock(uint8_t block, uint8_t data[16]) {
  bool success = authBlock(block);
  
  if(!success)
    return false;

  this->printDebug("Authenticated block", "");

  uint8_t success_t = this->nfc_lib->mifareclassic_WriteDataBlock(block, data);

  if(!success_t)
    return false;

  this->printDebug("Read success", "");

  return true;
}

bool NFC::isPowered() {
  return powered;
}