/**************************************************************************//**
  \file  halSleep.h

  \brief Interface to control sleep mode.

  \author
      Atmel Corporation: http://www.atmel.com \n
      Support email: avr@atmel.com

    Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
    Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
      1/12/09 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALSLEEP_H
#define _HALSLEEP_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <sleep.h>
#include <sleepTimer.h>

/******************************************************************************
                   Defines section
******************************************************************************/
#define HAL_ACTIVE_MODE                    0
#define HAL_SLEEP_MODE                     1
#define HAL_SLEEP_TIMER_IS_STOPPED         0
#define HAL_SLEEP_TIMER_IS_STARTED         1
#define HAL_SLEEP_TIMER_IS_WAKEUP_SOURCE   0
#define HAL_EXT_IRQ_IS_WAKEUP_SOURCE       1

/******************************************************************************
                   Types section
******************************************************************************/
typedef struct
{
  HAL_WakeUpCallback_t callback;
  HAL_SleepTimer_t sleepTimer;
  uint8_t wakeupStation    : 1;
  uint8_t wakeupSource     : 1;
  uint8_t sleepTimerState  : 1;
} HalSleepControl_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Switch on system power.

\param[in]
  wakeupSource - wake up source
******************************************************************************/
void halPowerOn(const uint8_t wakeupSource);

/*******************************************************************************
  Shutdown system.
  NOTES:
  the application should be sure the poweroff will not be
  interrupted after the execution of the sleep().
*******************************************************************************/
void halPowerOff(void);

#endif /* _HALSLEEP_H */
// eof halSleep.h
