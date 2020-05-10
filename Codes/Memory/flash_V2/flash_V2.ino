#include "FLASH.h"

flash eeprom;
uint8_t data = 55;

void setup() {
  Serial.begin(115200);
  Serial.println(data, HEX);
  //  eeprom.writeInt32(0x01024040);
  //  eeprom.writeInt32(0x01024040);
  //  eeprom.readInt32(data);
  //
  //  eeprom.readInt32(data);


  FLASH_Unlock();
  FLASH_ProgramHalfWord(Page0, 31);
  FLASH_Lock();

  uint32_t a = Page0 + 2;
  FLASH_Unlock();
  FLASH_ProgramHalfWord(a, 31);
  FLASH_Lock();

  FLASH_Unlock();
  FLASH_ProgramHalfWord(Page1, 31);
  FLASH_Lock();

  FLASH_Unlock();
  FLASH_ProgramHalfWord(Page2, 31);
  FLASH_Lock();
  Serial.println("write done");
  /*
    for (int i = 0; i < 10; i++) {
    FLASH_Unlock();
    FLASH_ProgramHalfWord(Page0 + i, 31);
    FLASH_Lock();
    Serial.println("write done");
    }

    for (int i = 0; i < 10; i++) {
    eeprom.readByte(data);
    Serial.println(data, HEX);

    }*/
}

void loop() {

}
