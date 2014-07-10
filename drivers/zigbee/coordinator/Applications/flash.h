#ifndef FLASH_H
#define FLASH_H

#define     F930_FLASHPAGE_SIZE				0x400


extern uint8_t   TempFlashPageBuffer[F930_FLASHPAGE_SIZE];


extern void FLASH_PageErase (uint32_t addr);
extern uint8_t FLASH_ByteRead (uint32_t addr);
extern void FLASH_ByteWrite (uint32_t addr, char byte);
extern void FLASH_ReadTotalPageData(uint32_t addr);
extern void FLASH_WriteTotalPageData(uint32_t addr);
extern uint8_t CheckModelIntegrality();
extern uint8_t CompareMacAddress();

#endif
