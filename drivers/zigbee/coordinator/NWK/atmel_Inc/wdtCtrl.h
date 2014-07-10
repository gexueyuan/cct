/**************************************************************************//**
  \file  wdtCtrl.h

  \brief The header file describes the WDT interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    10/12/07 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _WDTCTRL_H
#define _WDTCTRL_H

/******************************************************************************
                   Includes section
******************************************************************************/
//#include <halWdt.h>

/******************************************************************************
                   Types section
******************************************************************************/
// \cond
// an interval before WDT will expire
#if defined(AT91SAM7X256)
  typedef enum
  {
    WDT_INTERVAL_4    = 0xFFE,   // 4 ms
    WDT_INTERVAL_16   = 0xFFB,   // 16 ms
    WDT_INTERVAL_32   = 0xFF9,   // 32 ms
    WDT_INTERVAL_64   = 0xFEF,   // 64 ms
    WDT_INTERVAL_125  = 0xFDF,   // 125 ms
    WDT_INTERVAL_250  = 0xFBF,   // 250 ms
    WDT_INTERVAL_500  = 0xF7F,   // 500 ms
    WDT_INTERVAL_1000 = 0xEFF,   // 1 second
    WDT_INTERVAL_2000 = 0xDFF,   // 2 seconds
    WDT_INTERVAL_4000 = 0xBFF,   // 4 seconds
    WDT_INTERVAL_8000 = 0x7FF    // 8 seconds
  } HAL_WdtInterval_t;
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  typedef enum
  {
    WDT_INTERVAL_16   = 0x00,    // 16 ms
    WDT_INTERVAL_32   = 0x01,    // 32 ms
    WDT_INTERVAL_64   = 0x02,    // 64 ms
    WDT_INTERVAL_125  = 0x03,    // 125 ms
    WDT_INTERVAL_250  = 0x04,    // 250 ms
    WDT_INTERVAL_500  = 0x05,    // 500 ms
    WDT_INTERVAL_1000 = 0x06,    // 1 second
    WDT_INTERVAL_2000 = 0x07,    // 2 seconds
    WDT_INTERVAL_4000 = 0x20,    // 4 seconds
    WDT_INTERVAL_8000 = 0x21     // 8 seconds
  }HAL_WdtInterval_t;
#endif
// \endcond

/******************************************************************************
                   Prototypes section
******************************************************************************/
/***************************************************************************//**
\brief Starts WDT within a given interval.

\param[in]
  interval - interval. Must be chosen from: \n
      WDT_INTERVAL_16      // 16 ms \n
      WDT_INTERVAL_32      // 32 ms \n
      WDT_INTERVAL_64      // 64 ms \n
      WDT_INTERVAL_125     // 125 ms \n
      WDT_INTERVAL_250     // 250 ms \n
      WDT_INTERVAL_500     // 500 ms \n
      WDT_INTERVAL_1000    // 1 second \n
      WDT_INTERVAL_2000    // 2 seconds \n
      WDT_INTERVAL_4000    // 4 seconds \n
      WDT_INTERVAL_8000    // 8 seconds \n
*******************************************************************************/
//void HAL_StartWdt(HAL_WdtInterval_t interval);

/***************************************************************************//**
\brief Registers WDT fired callback.

\param[in]
  wdtCallback - pointer to the callback function.
*******************************************************************************/
void HAL_RegisterWdtCallback(void (*wdtCallback)(void));

/******************************************************************************
                   Inline static functions section
******************************************************************************/
#if defined(AT91SAM7X256)
/**************************************************************************//**
\brief reset and reload wdt counter
******************************************************************************/
INLINE void HAL_ResetWdt(void)
{
  halResetWdt();
}

#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
/*******************************************************************//**
\brief stops the WDT
***********************************************************************/
#define HAL_StopWdt() wdt_disable()

/*******************************************************************//**
\brief resets the WDT
***********************************************************************/
#define HAL_ResetWdt() wdt_reset()
#endif

#endif /*_WDTCTRL_H*/

// eof wdtCtrl.h
