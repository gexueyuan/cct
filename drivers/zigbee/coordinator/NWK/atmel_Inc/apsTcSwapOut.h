/*************************************************************************//**
  \file apsTcSwapOut.h

  \brief Interface of APS Trust Centre Swap Out.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-11-28 Max Gekk - Created.
   Last change:
    $Id: apsTcSwapOut.h 17813 2011-07-04 14:11:12Z dloskutnikov $
 *****************************************************************************/
#if !defined _APS_TC_SWAP_OUT_H
#define _APS_TC_SWAP_OUT_H
/**//**
 *
 *  In order to protect the data that is being backed up, a hash on the TC
 * link key will be performed and that will be the key stored externally.
 * The actual link key used for operational networks never be transported
 * out of the ESI. See ZigBee-095310r23ZB, page 82.
 *
 **/

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <apsCommon.h>

#if defined _SECURITY_ && defined _LINK_SECURITY_ && defined _TC_SWAPOUT_

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Creates link key hash for the Trust Centre and store it inside.
 ******************************************************************************/
void APS_EnableLinkKeyHashChecking(void);

/**************************************************************************//**
  \brief Writes invalid value to link key hash for the Trust Centre.
 ******************************************************************************/
void APS_DisableLinkKeyHashChecking(void);

/**************************************************************************//**
  \brief Gets pointer to stored link key hash for the Trust Centre.
 ******************************************************************************/
uint8_t *APS_GetLinkKeyHash(void);

#endif /* _SECURITY_ and _LINK_SECURITY_ and _TC_SWAPOUT_ */

#endif /* _APS_TC_SWAP_OUT_H */
/** eof apsTcSwapOut.h */

