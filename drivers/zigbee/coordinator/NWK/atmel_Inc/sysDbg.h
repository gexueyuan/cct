/*****************************************************************************
  \file  sysDbg.h

  \brief

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    13.10.2009 ivagulin - Created
******************************************************************************/


#ifndef _SYSDBG_H_
#define _SYSDBG_H_

#include <dbg.h>

typedef enum
{
  SYS_ASSERT_ID_DOUBLE_QUEUE_PUT             = 0x8000,
  SYS_ASSERT_ID_DRT_SIZE_TOO_BIG             = 0x8001,
  SYS_ASSERT_ID_DEVICE_WAS_NOT_ABLE_TO_SLEEP = 0x8002
} SysAssertId_t;

#endif /* _SYSDBG_H_ */
