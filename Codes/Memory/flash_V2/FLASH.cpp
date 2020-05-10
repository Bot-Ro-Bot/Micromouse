#include"FLASH.h"

flash::flash() {
  readOffset = 0;
  writeOffset = 0;
  FLASH_Unlock();
  FLASH_ErasePage(Page0);
  FLASH_ErasePage(Page1);
  FLASH_ErasePage(Page2);
  FLASH_ErasePage(Page3);
  FLASH_Lock();
}

FLASH_Status flash::Erase(uint32_t PageNo) {
  FLASH_Unlock();
  FLASH_ErasePage(PageNo);
  FLASH_Lock();
}

FLASH_Status flash::writeByte(uint8_t writeData) {
  FLASH_Status error;
  FLASH_Unlock();
  error  = FLASH_ProgramHalfWord(baseAddress + writeOffset, writeData);
  FLASH_Lock();
  writeOffset += 1;
  return error;
}

FLASH_Status flash::writeInt16(uint16_t writeData) {
  FLASH_Status error;
  FLASH_Unlock();
  error  = FLASH_ProgramHalfWord(baseAddress + writeOffset, writeData);
  FLASH_Lock();
  writeOffset += 2;
  return error;
}

FLASH_Status flash::writeInt32(uint32_t writeData) {
  FLASH_Status error;
  FLASH_Unlock();
  error  = FLASH_ProgramHalfWord(baseAddress + writeOffset, (uint16_t)writeData);
  error  = FLASH_ProgramHalfWord(baseAddress + writeOffset + 2, (uint16_t)(writeData % 65536));
  FLASH_Lock();
  writeOffset += 4;
  return error;
}

void flash:: readByte(uint8_t &readData) {
  readData = ( * (uint8_t *) baseAddress + readOffset);
  readOffset += 1;
}

void flash:: readInt16(uint16_t &readData) {
  readData = ( * (uint16_t *) baseAddress + readOffset);
  readOffset += 2;
}

void flash:: readInt32(uint32_t &readData) {
  readData = ( * (uint16_t *) baseAddress + readOffset) << 8 + ( * (uint16_t *) baseAddress + readOffset + 2);
  //readData = readData << 16;
  //readData &= ( * (uint32_t *) baseAddress + readOffset + 2);
  readOffset += 4;
}
