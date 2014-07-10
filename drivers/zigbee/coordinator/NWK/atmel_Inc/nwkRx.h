/**************************************************************************//**
  \file nwkRx.h

  \brief Functions of processing of input data

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2007-06-14 V. Panov - Created.
    2008-11-25 M. Gekk  - Counters of buffers have been remote.
    2008-12-10 M. Gekk  - Memory optimization.
    2009-07-08 M. Gekk  - Redesign.
   Last change:
    $Id: nwkRx.h 18298 2011-08-19 12:42:09Z mgekk $
 ******************************************************************************/
#if !defined _NWK_RX_H
#define _NWK_RX_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <nwkCommon.h>
#include <nwkSystem.h>
#include <nwkFrame.h>
#include <nldeData.h>
#include <nwkTx.h>
#if defined _SECURITY_
#include <sspSfp.h>
#endif /* _SECURITY_ */

/******************************************************************************
                              Definition section
 ******************************************************************************/
#define NWK_UNICAST_COMMAND_TRANSIT_TX_PARAMETERS \
  {NWK_TX_DELAY_UNICAST_COMMAND, NULL, NULL, true}
#define NWK_UNICAST_DATA_TRANSIT_TX_PARAMETERS \
  {NWK_TX_DELAY_TRANSIT_DATA, NULL, NULL, true}
#define NWK_BROADCAST_TRANSIT_TX_PARAMETERS \
  {NWK_TX_DELAY_BROADCAST, NULL, NULL, true}
#define NWK_MULTICAST_MEMBER_TRANSIT_TX_PARAMETERS \
  {NWK_TX_DELAY_MULTICAST, NULL, NULL, true}

/******************************************************************************
                                 Types section
 ******************************************************************************/
/** Input packet meta information. */
typedef struct _NwkInputPacket_t
{
  bool retransmit      : 1;
  bool decryptRequired : 1;
  bool indicate        : 1;
  NwkBitField_t txId   : 5;
  uint8_t *data;
  NwkLength_t length;
  NwkParseHeader_t parsedHeader;
  union
  {
    NWK_DataInd_t dataInd;
#if defined _SECURITY_
    SSP_DecryptFrameReq_t decryptReq;
#endif /* _SECURITY_ */
  } primitive;
} NwkInputPacket_t;

/******************************************************************************
                             Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief The upper layer has completed data processing.

  \param[in] resp - NLDE-DATA indication primitive's parameters structure.
  \return None.
 ******************************************************************************/
void NWK_DataResp(NWK_DataResp_t *resp);

#endif /* _NWK_RX_H */
/** eof nwkRx.h */

