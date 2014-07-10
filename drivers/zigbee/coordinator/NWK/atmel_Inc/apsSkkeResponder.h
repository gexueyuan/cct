/**************************************************************************//**
  \file  apsSkkeResponder.h

  \brief Private interface of APS SKKE Responder.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2011-04-24 Max Gekk - Created.
   Last change:
    $Id: apsSkkeResponder.h 17813 2011-07-04 14:11:12Z dloskutnikov $
 ******************************************************************************/
#if !defined _APS_SKKE_RESPONDER_H
#define _APS_SKKE_RESPONDER_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwk.h>

#if defined _HI_SECURITY_ 
/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Main task handler of SKKE responder.
 ******************************************************************************/
APS_PRIVATE void apsSkkeResponderTaskHandler(void);

/**************************************************************************//**
  \brief Process SKKE-1 command frame from a initiator.

  \param[in] ind - pointer to NWK parameters of received SKKE-1 frame.
  \return None.
 ******************************************************************************/
APS_PRIVATE void apsRSkke1Ind(NWK_DataInd_t *ind);

/**************************************************************************//**
  \brief Process SKKE-3 command frame from a initiator.

  \param[in] ind - pointer to NWK parameters of received SKKE-3 frame.
  \return None.
 ******************************************************************************/
APS_PRIVATE void apsRSkke3Ind(NWK_DataInd_t *ind);

#else  /* _HI_SECURITY_ */


#define apsSkkeResponderTaskHandler NULL

#endif /* _HI_SECURITY_ */

#endif /* _APS_SKKE_RESPONDER_H */
/** eof apsSkkeResponder.h */
