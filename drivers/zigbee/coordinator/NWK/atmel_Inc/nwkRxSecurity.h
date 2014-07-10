/**************************************************************************//**
  \file nwkRxSecurity.h

  \brief Interface of security processing of incoming packets.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2011-02-03 M. Gekk - Created.
   Last change:
    $Id: nwkRxSecurity.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NWK_RX_SECURITY_H
#define _NWK_RX_SECURITY_H

#if defined _SECURITY_
/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <nwkSecurity.h>

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Update the incoming frame counter by key sequence number.

  \param[in] seqNum - key sequence number.
  \param[in] srcExtAddr - an extended source address.
  \param[in] newValue - a new value of the incoming frame counter.

  \return 'true' if incoming frame counter is updated otherwise 'false'.
 ******************************************************************************/
NWK_PRIVATE bool nwkUpdateInFrameCounter(const NWK_KeySeqNum_t seqNum,
  const ExtAddr_t srcExtAddr, const NwkInFrameCounter_t counter);

/**************************************************************************//**
  \brief Get network key by key sequence number.

  \return Pointer to value of a network key.
 ******************************************************************************/
NWK_PRIVATE const uint8_t* NWK_GetKey(const NWK_KeySeqNum_t seqNum);

#endif /* _SECURITY_ */
#endif /* _NWK_RX_SECURITY_H */
/** eof nwkRxSecurity.h */

