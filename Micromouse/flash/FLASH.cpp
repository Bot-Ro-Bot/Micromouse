#include "FLASH.h"

static uint32_t PageAddress;
static uint8_t PageNumber;


//static void FLASH_ErasePage(void)
//{
//  FLASH_Unlock();
//  FLASH_ErasePage();
//  FLASH_Lock();
//}

void FLASH_SetPageAddress(uint8_t page, uint32_t address)
{
  PageAddress = address;
  PageNumber = page;
}

FLASH_Status FLASH_Write(uint32_t address, int16_t writeData)
{
  //uint32_t flashAddress = PageAddress + address;

  FLASH_Status error;

 // FLASH_ErasePage(PageAddress);

  FLASH_Unlock();

  error  = FLASH_ProgramHalfWord(address, writeData);

  FLASH_Lock();

  return error;
}


void FLASH_Read(uint32_t address, int16_t  *readData)
{
  //uint32_t flashAddress = PageAddress + address;
//  readData =  * (int16_t *)address;

}
