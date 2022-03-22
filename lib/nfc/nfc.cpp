#include <nfc.h>

NFC::NFC(uint8_t power_pin, void (*printDebug)(String), void (*printFault)(String)) {
  Serial.println("Begin construct NFC");
  this->power_pin = power_pin;
  this->pn532i2c = new PN532_I2C(Wire);
  this->printDebug = printDebug;
  this->printFault = printFault;

  pinMode(this->power_pin, OUTPUT);
  Serial.println("Constructed NFC");
}

NFC::~NFC() {
  delete this->pn532i2c;
}

bool NFC::powerOn() {
  digitalWrite(this->power_pin, HIGH);

  if(this->nfc_lib != nullptr) {
    delete this->nfc_lib;
  }

  this->nfc_lib = new PN532(*pn532i2c);

  // Maybe add this->nfc_lib->begin(), but that may reset the whole Wire lib
  // (also used for Encoders)

  // TODO: May not be necessary
  delay(250);

  uint32_t versiondata = this->nfc_lib->getFirmwareVersion();

  if(!versiondata) {
    this->printFault("Failed to get version data.");
    return false;
  }

  this->nfc_lib->SAMConfig();
  powered = true;
}

void NFC::powerOff() {
  if(this->nfc_lib != nullptr)
    delete this->nfc_lib;

  powered = false;
}

bool NFC::getBlock(uint8_t block, uint8_t (&data)[16]) {
  this->printDebug("Start getBlock");
  if(!powered)
    return false;

  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;
  uint8_t key_universal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  this->printDebug("Initialized uint8_t's");

  success = this->nfc_lib->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if(!success)
    return false;

  this->printDebug("Read passive target id");

  success = this->nfc_lib->mifareclassic_AuthenticateBlock(uid, uidLength, block, 1, key_universal);

  if(!success)
    return false;

  this->printDebug("Authenticated block");

  success = this->nfc_lib->mifareclassic_ReadDataBlock(block, data);

  if(!success)
    return false;

  this->printDebug("Read success");

  return true;
}