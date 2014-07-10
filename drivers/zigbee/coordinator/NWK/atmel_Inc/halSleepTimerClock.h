/**************************************************************************//**
  \file  halSleepTimerClock.h

  \brief Definition for count out requested sleep interval.

  \author
      Atmel Corporation: http://www.atmel.com \n
      Support email: avr@atmel.com

    Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
    Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
      29/06/07 E. Ivanov - Created
       7/04/09 A. Khromykh - Refactored
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALSLEEPTIMERCLOCK_H
#define _HALSLEEPTIMERCLOCK_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <halTaskManager.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
#define SLEEPTIMER_CLOCK                   32768ul
#define SLEEPTIMER_DIVIDER                  1024ul

/******************************************************************************
                   Prototypes section
******************************************************************************/
/******************************************************************************
Starts the sleep timer clock.
******************************************************************************/
void halStartSleepTimerClock(void);

/******************************************************************************
Sets interval.
Parameters:
  value - contains number of ticks which the timer must count out.
Returns:
  none.
******************************************************************************/
void halSetSleepTimerInterval(uint32_t value);

/******************************************************************************
Returns the sleep timer frequency in Hz.
Parameters:
  none.
Returns:
  the sleep timer frequency in Hz.
******************************************************************************/
uint32_t halSleepTimerFrequency(void);

/**************************************************************************//**
\brief Wake up procedure for all external interrupts
******************************************************************************/
void halWakeupFromIrq(void);

/**************************************************************************//**
\brief Get time of sleep timer.

\return
  time in ms.
******************************************************************************/
uint64_t halGetTimeOfSleepTimer(void);

/******************************************************************************
                   Inline static functions section
******************************************************************************/
/******************************************************************************
  Interrupt handler signal implementation
******************************************************************************/
INLINE void halInterruptSleepClock(void)
{
  halPostTask(HAL_ASYNC_TIMER);
}

#endif /* _HALSLEEPTIMERCLOCK_H */
// eof halSleepTimerClock.h
