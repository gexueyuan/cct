/**************************************************************************//**
  \file  halDiagnostic.h

  \brief Implementation of diagnostics defines.

  \author
      Atmel Corporation: http://www.atmel.com \n
      Support email: avr@atmel.com

    Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
    Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
      22/04/10 A. Khromykh - Created
 ******************************************************************************/

#ifndef _HALDIAGNOSTIC_H
#define _HALDIAGNOSTIC_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <halDbg.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#if defined (MEASURE)
    #define BEGIN_MEASURE
    #define END_MEASURE(code)
#else
    #define BEGIN_MEASURE
    #define END_MEASURE(code)
#endif


#endif /* _HALDIAGNOSTIC_H */

