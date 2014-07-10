/******************************************************************************
  \file machwdMem.h
  
  \brief MACHWD memory type.
  
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \brief
    History:     
      24/04/08 A. Mandychev - Created.
******************************************************************************/

#ifndef _MACHWDMEM_H
#define _MACHWDMEM_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <types1.h>
#include <macBuffers.h>

/******************************************************************************
                    Define(s) section
******************************************************************************/

/******************************************************************************
                    Types section
******************************************************************************/
typedef struct
{
  FrameRxBufferState_t rxBufferState;
} MachwdMem_t;

#endif /*_MACHWDMEM_H*/

// eof machwdMem.h
