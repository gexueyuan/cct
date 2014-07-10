/**************************************************************************//**
  \file nwkBTT.h

  \brief Broadcast transaction table.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2007-09-01 V. Panov - Created.
    2008-11-17 M. Gekk - Correct clearing of record in BTT. In case of overflow
                       of the table function nwkCheckBTT returns an error.
                       BTT moved in ConfigServer.
    2009-04-29 M. Gekk - Refactoring.
    2009-11-24 Igor Vagulin - reimplemented using sysDuplicateTable.
   Last change:
    $Id: nwkBTT.h 17946 2011-07-13 09:26:59Z mgekk $
 ******************************************************************************/
#if !defined _NWK_BTT_H
#define _NWK_BTT_H

/******************************************************************************
                                Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <mac.h>
#include <sysDuplicateTable.h>

/******************************************************************************
                                 Types section
 ******************************************************************************/
/** Broadcast transaction table */
typedef struct _NwkBTT_t
{
  /** Pointer to BTT sysDuplicateTable descriptor */
  SYS_DuplicateTable_t btt;
} NwkBTT_t;

/******************************************************************************
                             Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Reseting of the broadcast transaction table.
 ******************************************************************************/
NWK_PRIVATE void nwkResetBTT(void);

/**************************************************************************//**
  \brief Check entry for short address and sequence number.

  \param[in] shortAddr - source address of broadcast packet.
  \param[in] sequenceNumber - a sequence number of broadcast packet.
 ******************************************************************************/
NWK_PRIVATE SysDuplicateTableAnswer_t nwkCheckBTT(const ShortAddr_t shortAddr,
  const uint8_t sequenceNumber);

/**************************************************************************//**
  \brief Clear the existing entry in the broadcast transaction table.

  \param[in] shortAddr - a short address of broadcast packet,
  \param[in] sequenceNumber - a sequence number of broadcast packet.

  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkClearBTR(const ShortAddr_t shortAddr,
  const uint8_t sequenceNumber);

/**************************************************************************//**
  \brief Add new BTR to BTT with own address and given sequence number

  \param[in] sequenceNumber - a sequence number of broadcast packet.
  \return None.
 ******************************************************************************/

#if !defined NWK_ALWAYS_REJECT_OWN_BROADCAST
NWK_PRIVATE void nwkAddOwnBTR(const uint8_t sequenceNumber);

#else
#define nwkAddOwnBTR(sequenceNumber) (void)0
#endif /* NWK_ALWAYS_REJECT_OWN_BROADCAST */
#endif /* _NWK_BTT_H */
/** eof nwkBTT.h */

