/*****************************************************************************//**
\file  halDbg.h

\brief Declarations of hal , bsb mistake interface.

\author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

\internal
  History:
    09/11/07 A. Khromykh - Created
**********************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALDBG_H
#define _HALDBG_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <dbg.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
enum
{
  APPTIMER_MISTAKE               = 0x2000,
  INCORRECT_EEPROM_ADDRESS       = 0x2001,
  MEMORY_CANNOT_WRITE            = 0x2002,
  USARTC_HALUNKNOWNERRORREASON_0 = 0x2003,
  USARTC_HALUNKNOWNERRORREASON_1 = 0x2004,
  USARTC_HALSIGUSARTTRANSMISSIONCOMPLETE_0 = 0x2005,
  USARTC_HALSIGUSARTRECEPTIONCOMPLETE_0 = 0x2006,
  HSMCI_CLOCK_DIVIDER_ERROR      = 0x2007,
};

/******************************************************************************
                   Prototypes section
******************************************************************************/

#endif /* _HALDBG_H */

// eof halDbg.h
