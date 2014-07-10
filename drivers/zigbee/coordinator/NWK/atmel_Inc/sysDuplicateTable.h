/*****************************************************************************
  \file  sysDuplicateEntry.h

  \brief 

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    11/20/2009 I.Vagulin - Created
******************************************************************************/


#ifndef _SYSDUPLICATEENTRY_H_
#define _SYSDUPLICATEENTRY_H_

#include <types.h>

typedef enum
{
  SYS_DUPLICATE_TABLE_ANSWER_FOUND,
  SYS_DUPLICATE_TABLE_ANSWER_ADDED,
  SYS_DUPLICATE_TABLE_ANSWER_FULL, // check can return Full only if removeOldest set to false
} SysDuplicateTableAnswer_t ;

typedef struct _SYS_DuplicateTableEntry_t
{
  uint16_t address;
  uint8_t seqNumber;
  uint8_t ttl;
  uint8_t mask;
} SYS_DuplicateTableEntry_t ;

typedef struct _SYS_DuplicateTable_t
{
  bool removeOldest:1;
  uint8_t size:7;
  SYS_DuplicateTableEntry_t *entries;

  uint16_t agingPeriod;
  uint32_t lastStamp;
  uint8_t maxTTL;
} SYS_DuplicateTable_t ;

/*****************************************************************************
  Prepare duplicate table to real work :)

  Parameters: table - pointer to allocated table,
              entrySize - count of entries in table
              entries - pointer to array of entries
              agingPeriod - aging period ms
              removeOldest - change behaviour on full table
  Returns:    nothing
*****************************************************************************/
void SYS_DuplicateTableReset(SYS_DuplicateTable_t *table,
    SYS_DuplicateTableEntry_t *entries, uint8_t tableSize,
    uint16_t agingPeriod, uint8_t maxTTL, bool removeOldest);

/*****************************************************************************
  Search for record in table, add if not found
  Parameters: table - pointer to allocated table,
              address, seqNumber - record to search for or to add if not found
  Returns:    true if record is found false otherwise.
*****************************************************************************/
SysDuplicateTableAnswer_t SYS_DuplicateTableCheck(SYS_DuplicateTable_t *table,
    uint16_t address, uint8_t seqNumber);

/*****************************************************************************
  Search for record in table, remove if found.
  Parameters: table - pointer to allocated table,
              address, seqNumber - record to search for and to remove if found
  Returns:    nothing
*****************************************************************************/
void SYS_DuplicateTableClear(SYS_DuplicateTable_t *table,
  uint16_t address, uint8_t seqNumber);

#endif /* _SYSDUPLICATEENTRY_H_ */
