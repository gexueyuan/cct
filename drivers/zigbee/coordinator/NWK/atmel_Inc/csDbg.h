/******************************************************************************
  \file csDbg.h

  \brief
    Configuration Server debug info

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    22.10.10 A. Razinkov - Created.
******************************************************************************/

#ifndef _CSDBG_H_
#define _CSDBG_H_

/******************************************************************************
                    Includes section
******************************************************************************/
#include <dbg.h>

typedef enum
{
  CS_CSGETITEM0 = 0x6000
} CS_DbgCodeId_t;

#endif /* _CSDBG_H_ */
