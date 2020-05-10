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


#define baseAddress 0x0801F000

class flash {
//  private:
    uint16_t writeOffset;
    uint16_t readOffset;
    
  public:
    flash();
    FLASH_Status Erase(uint32_t PageNo);
    void FLASH_SetPageAddress(uint8_t page, uint32_t address);

    FLASH_Status writeByte(uint8_t writeData);
    FLASH_Status writeInt16(uint16_t writeData);
    FLASH_Status writeInt32(uint32_t writeData);

    void readByte(uint8_t &readData);
    void readInt16(uint16_t &readData);
    void readInt32(uint32_t &readData);
};


#endif
