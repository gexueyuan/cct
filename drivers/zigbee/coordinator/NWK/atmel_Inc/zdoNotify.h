/**************************************************************************//**
  \file zdoNotify.h

  \brief Functions for raising ZDO notifications

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    13.11.09 A. Taradov - Created
******************************************************************************/
#ifndef _ZDONOTIFY_H
#define _ZDONOTIFY_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <zdo.h>
#include <aps.h>

/******************************************************************************
                    Types section
******************************************************************************/

/******************************************************************************
                    Prototypes section
******************************************************************************/

/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() only with status information.
  Parameters:
    status - update status
  Return:
    none
******************************************************************************/
void ZDO_NotifySimple(ZDO_Status_t status);

/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() with prepared notification information
  Parameters:
    notf - notification information
  Return:
    none
******************************************************************************/
void ZDO_NotifyGeneral(ZDO_MgmtNwkUpdateNotf_t *notf);

/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() with child update information.
  Parameters:
    status - update status
    childInfo - updated device information
  Return:
    none
******************************************************************************/
void ZDO_NotifyChildUpdate(ZDO_Status_t status, ChildInfo_t *childInfo);

/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() with new network parameters.
  Parameters:
    status - update status
  Return:
    none
******************************************************************************/
void ZDO_NotifyNwkUpdate(ZDO_Status_t status);

/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() with conflict address.
  Parameters:
    status - update status
    shortAddr - conflicting address
  Return:
    none
******************************************************************************/
void ZDO_NotifyAddressConflict(ZDO_Status_t status, ShortAddr_t *shortAddr);

#if defined(_SECURITY_) && defined _LINK_SECURITY_ 
/******************************************************************************
  Call ZDO_MgmtNwkUpdateNotf() with information about security keys updates.
  Parameters:
    status - update status
    extAddr - extended address of updated device
  Return:
    none
******************************************************************************/
void ZDO_NotifySecurityUpdate(ZDO_Status_t status, ExtAddr_t *extAddr);

#ifdef _TC_PROMISCUOUS_MODE_
/**************************************************************************//**
\brief Notification about remote device bind have been staled

 The function should be defined by an application.
 It is called when the device was notificated about bounded device lost

\param[in] extAddr - pointer to extended address of lost bounded device
******************************************************************************/
void ZDO_NotifyBindLost(ExtAddr_t* extAddr);
#endif /* _TC_PROMISCUOUS_MODE_ */

#endif // defined(_SECURITY_) &&  defined _LINK_SECURITY_ 

#endif // _ZDONOTIFY_H

// eof zdoNotify.h
