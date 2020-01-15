#include "FLASH.h"

void setup() {
  Serial.begin(115200);
  FLASH_SetPageAddress(127, 0x0801FC00);


  Serial.print("STATRTTT  ");
  FLASH_Status error;
  //Serial.print("Initial value of d:  ");  Serial.println(d, HEX);
  FLASH_Write(0x0801FC00, 0xFFFF);
  Serial.print("error:  ");  Serial.println(error);

  FLASH_Write(0x0801FC02, 0xCCCC);
  Serial.print("error:  ");  Serial.println(error);

  FLASH_Write(0x0801FC04, 0xDDDD);
  Serial.print("error:  ");  Serial.println(error);

  FLASH_Write(0x0801FC06, 0xEEEE);
  Serial.print("error:  ");  Serial.println(error);

  FLASH_Write(0x0801FC08, 0xFFFF);
  Serial.print("error:  ");  Serial.println(error);
  delay(100);
  //

  Serial.print("Data from memory 1: "); Serial.println(( * (uint16_t *) 0x0801FC00), HEX);
  Serial.print("Data from memory 2: "); Serial.println(( * (uint16_t *) 0x0801FC02), HEX);
  Serial.print("Data from memory 3: "); Serial.println(( * (uint16_t *) 0x0801FC04), HEX);
  Serial.print("Data from memory 4: "); Serial.println(( * (uint16_t *) 0x0801FC06), HEX);
  Serial.print("Data from memory 5: "); Serial.println(( * (uint16_t *) 0x0801FC08), HEX);

  Serial.print("Data from memory 1: "); Serial.println(( * (uint16_t *) 0x0801FC00), HEX);
  Serial.print("Data from memory 2: "); Serial.println(( * (uint16_t *) 0x0801FC02), HEX);
  Serial.print("Data from memory 3: "); Serial.println(( * (uint16_t *) 0x0801FC04), HEX);
  Serial.print("Data from memory 4: "); Serial.println(( * (uint16_t *) 0x0801FC06), HEX);
  Serial.print("Data from memory 5: "); Serial.println(( * (uint16_t *) 0x0801FC08), HEX);

  FLASH_Unlock();
  FLASH_ErasePage(0x0801FC00);
  FLASH_Lock();
  Serial.print("Data from memory 1: "); Serial.println(( * (uint16_t *) 0x0801FC00), HEX);
  Serial.print("Data from memory 2: "); Serial.println(( * (uint16_t *) 0x0801FC02), HEX);
  Serial.print("Data from memory 3: "); Serial.println(( * (uint16_t *) 0x0801FC04), HEX);
  Serial.print("Data from memory 4: "); Serial.println(( * (uint16_t *) 0x0801FC06), HEX);
  Serial.print("Data from memory 5: "); Serial.println(( * (uint16_t *) 0x0801FC08), HEX);

}

void loop() {
  // put your main code here, to run repeatedly:

}
