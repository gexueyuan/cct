/******************************************************************************
  \file macMem.h
  
  \brief MAC memory type.
  
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).
  
  \internal
    History:     
    23/04/08 A. Mandychev - Created.
******************************************************************************/

#ifndef _MACMEM_H
#define _MACMEM_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <machwiMem.h>
#include <machwdMem.h>
#include <macenvMem.h>

/******************************************************************************
                    Define(s) section
******************************************************************************/
 
/******************************************************************************
                    Types section
******************************************************************************/
typedef struct
{
  MachwiMem_t hwi;
  MachwdMem_t hwd;
} MAC_Mem_t;

/******************************************************************************
                   Extern variables section
******************************************************************************/
extern MAC_Mem_t macMem;

/******************************************************************************
                    Prototypes section
******************************************************************************/

#endif /*_MACMEM_H*/

// eof macMem.h
