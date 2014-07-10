/**************************************************************************//**
  \file  uid.h

  \brief The header file describes the UID interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    7/12/07 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _UID_H
#define _UID_H

/******************************************************************************
                   Includes section
******************************************************************************/
// \cond
#include <types.h>
// \endcond

/******************************************************************************
                   Prototypes section
******************************************************************************/
/************************************************************//**
\brief  UID discovery.
\param[in]
    id - UID buffer pointer. \n
    Attention! Memory size must equal <i> (Number of devices) * 8 bytes </i>
\return
    0 - if UID of DS2411 has been found successfully; \n
   -1 - if some error occured during UID discovery.
****************************************************************/
int HAL_ReadUid(uint64_t *id);

#endif /* _UID_H */

// eof uid.h
