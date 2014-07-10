/**************************************************************************//**
  \file  sleep.h

  \brief The header file describes the sleep interface.

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
#ifndef _SLEEP_H
#define _SLEEP_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>

/******************************************************************************
                   Types section
******************************************************************************/
typedef void (* HAL_WakeUpCallback_t)(void);

/** \brief fields of structure \n
    \brief uint32_t sleepTime - time of mcu+radio sleeping \n
    \brief HAL_WakeUpCallback_t callback - pointer to wake up callback function */
typedef struct
{
  uint32_t sleepTime;
  HAL_WakeUpCallback_t callback;
} HAL_Sleep_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Starts sleep timer and HAL sleep. When system is wake up send callback
\param[in]
    sleepParam - pointer to sleep structure.
\return
    -1 - bad parameters,   \n
    -2 - sleep timer busy, \n
    -3 - sleep system has been started.
     0 - success.
******************************************************************************/
int HAL_StartSystemSleep(HAL_Sleep_t *sleepParam);

/**************************************************************************//**
\brief Prepares mcu for power-save, power-down.
  Power-down the mode is possible only when internal RC is used
\return
  -1 - there is no possibility to sleep.
******************************************************************************/
int HAL_Sleep(void);

/***************************************************************************//**
\brief Makes MCU enter Idle mode.
*******************************************************************************/
void HAL_IdleMode(void);

#endif /* _SLEEP_H */
