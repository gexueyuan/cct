/**************************************************************************//**
  \file  halFlash.h

  \brief Interface of the hardware dependent part of the Flash module.
    Only minimal set of routines is supported to simulate EEPROM.

  \author
      Atmel Corporation: http://www.atmel.com \n
      Support email: avr@atmel.com

    Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
    Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
     24.12.10 A. Taradov - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALFLASH_H
#define _HALFLASH_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <halTaskManager.h>
#include <eeprom.h>
#include <AT91SAM3S4.h>
#include <core_cm3.h>

/******************************************************************************
                   Defines section
******************************************************************************/
#define IFLASH_BYTES_PER_PAGE      256
#define IFLASH_PAGES               1024
#define IFLASH_SIZE                (IFLASH_PAGES * IFLASH_BYTES_PER_PAGE)
#define IFLASH_BASE_ADDRESS        AT91C_IFLASH

#define FLASH_BOOTLOADER_PAGES     16

#define FLASH_EEPROM_PAGES         (EEPROM_DATA_MEMORY_SIZE / IFLASH_BYTES_PER_PAGE)
#define FLASH_EEPROM_BASE_PAGE     (IFLASH_PAGES - FLASH_EEPROM_PAGES - FLASH_BOOTLOADER_PAGES)
#define FLASH_EEPROM_BASE_ADDRESS  \
    (AT91C_IFLASH + FLASH_EEPROM_BASE_PAGE * IFLASH_BYTES_PER_PAGE)
#define FLASH_EEPROM_LOCK_REGION   15

#define FLASH_CMD_EWP              0x03 // Erase page and write page
#define FLASH_CMD_EWPL             0x04 // Erase page and write page then lock
#define FLASH_CMD_GLB              0x0A // Get Lock Bit
#define FLASH_CMD_CLB              0x09 // Clear Lock Bit
#define FLASH_CMD_SLB              0x08 // Set Lock Bit

#define FLASH_WRITE_ENABLE_MAGIC   (0x5A << 24)

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
  \brief Prepares flash for write operations.
******************************************************************************/
void halFlashPrepareForWrite(void);

/**************************************************************************//**
  \brief Restores Flash state changed by halPrepareFlashForWrite()
******************************************************************************/
void halFlashRestoreFromWrite(void);

/**************************************************************************//**
  \brief Prepares single page for write, updates eeprom write request structure
  \param[in] eeprom - EEPROM write request structure
  \return prepared page number
******************************************************************************/
uint16_t halFlashPreparePage(HAL_EepromParams_t *eeprom);

/**************************************************************************//**
  \brief Writes page to flash optionally locking Flash after write
  \param[in] page - page number to write to
  \param[in] lock - true if locking is required after write
******************************************************************************/
void halFlashWritePage(uint16_t page, bool lock);

/**************************************************************************//**
  \brief Unlocks page in Flash
  \param[in] page - page number to unlock
******************************************************************************/
void halFlashUnlockPage(uint16_t page);

/******************************************************************************
                   Inline static functions section
******************************************************************************/

/**************************************************************************//**
  \brief Checks if last lock region is locked.
    Note:
      1. EEPROM contents must be entirely palced in the last lock region.
      2. Maximum 32 lock regions are supported.
  \return true if last lock region is locked.
******************************************************************************/
INLINE bool halFlashIsLocked(void)
{
  EFC->EEFC_FCR = FLASH_WRITE_ENABLE_MAGIC | FLASH_CMD_GLB;
  return (EFC->EEFC_FRR & (1 << FLASH_EEPROM_LOCK_REGION)) > 0;
}

/**************************************************************************//**
  \brief Reads byte from EEPROM.
  \param[in] address - address of a byte.
  \return read value.
******************************************************************************/
INLINE uint8_t halFlashRead(uint16_t address)
{
  return *(uint8_t *)(FLASH_EEPROM_BASE_ADDRESS + address);
}

#endif // _HALFLASH_H

// eof halFlash.h
