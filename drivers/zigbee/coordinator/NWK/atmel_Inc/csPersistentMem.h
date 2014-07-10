/******************************************************************************
  \file csPersistentMem.h

  \brief
    Configuration Server persistent memory to store in EEPROM

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    28.10.10 A. Razinkov - Created.
******************************************************************************/

#ifndef _CSPERSISTENTMEM_H_
#define _CSPERSISTENTMEM_H_

/******************************************************************************
                    Includes section
******************************************************************************/
#include <configServer.h>

/******************************************************************************
                   Defines section
******************************************************************************/
// Offsets in EEPROM
#define SYSTEM_BASE_EEPROM_ADDRESS    0x0004

/******************************************************************************
                    Types section
******************************************************************************/
typedef struct _CS_PersistantData_t
{
  void*   value;
  int16_t offset;
} CS_PersistantData_t;

#endif /* _CSPERSISTENTMEM_H_ */
