#ifndef FLASH_H
#define FLASH_H

#include "wirish.h"
#include "flash_stm32.h"

//TOTAL 4 KB AS EEPROM

#define Page0 0x0801F000
#define Page1 0x0801F400
#define Page2 0x0801F800
#define Page3 0x0801FC00

#define PageSize 0x400  //1 kb size

//static void FLASH_ErasePage(void);
void FLASH_SetPageAddress(uint8_t page, uint32_t address);

FLASH_Status FLASH_Write(uint32_t address, int16_t writeData);


FLASH_Status FLASH_WriteByte(uint16_t addressOffset,  uint8_t writeData);
FLASH_Status FLASH_WriteInt16(uint16_t addressOffset,  uint16_t writeData);
FLASH_Status FLASH_WriteInt32(uint16_t addressOffset,  uint32_t writeData);

void FLASH_ReadByte(uint16_t addressOffset, uint8_t *readData);
void FLASH_ReadInt16(uint16_t addressOffset, uint16_t *readData);
void FLASH_ReadInt32(uint16_t addressOffset, uint32_t *readData);

#endif
