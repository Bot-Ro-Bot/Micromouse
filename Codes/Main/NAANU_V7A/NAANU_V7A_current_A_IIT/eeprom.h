#pragma once

void eepromWrite(uint16_t data) {
  FLASH_Unlock();
  FLASH_ProgramHalfWord(baseAddress + writeOffset, data);
  FLASH_Lock();
  writeOffset += 2;
}

uint16_t eepromRead() {
  uint16_t data;
  data = ( * (uint16_t *) (baseAddress + readOffset));
  readOffset += 2;
  return data;
}

void testEeprom() {
  path = 0;
  //  eepromWrite(88);
  //  path++;
  //  eepromWrite(88);
  //  path++;
  //  eepromWrite(88);
  //  path++;
  //  eepromWrite(66);
  //  path++;
  //  eepromWrite(88);
  //  path++;
  //  eepromWrite(44);
  //  path++;
  //  eepromWrite(88);
  //  path++;
  //  eepromWrite(55);
  //  path++;

  Serial.println("Writing to eeprom ....... ");
//  for (path = 0 ; path < 32; path++)
//  {
//    eepromWrite(anu[path]);
//  }
  //  eepromWrite(31);
  path++;

  FLASH_Unlock();
  FLASH_ProgramHalfWord(flagAddress, 31);
  FLASH_Lock();

  FLASH_Unlock();
  FLASH_ProgramHalfWord(pathAddress, path);
  FLASH_Lock();

  Serial.println("Written to eeprom ....... ");
}
