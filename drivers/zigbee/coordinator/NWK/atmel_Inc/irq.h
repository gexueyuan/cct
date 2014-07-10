/**************************************************************************//**
  \file  irq.h

  \brief The header file describes the IRQ interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    5/12/07 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _IRQ_H
#define _IRQ_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief The low level generates an interrupt request. */
#define  IRQ_LOW_LEVEL        0
/** \brief The high level generates an interrupt request (valid only for arm and avr32). */
#define  IRQ_HIGH_LEVEL       1
/** \brief Any edge generates an interrupt request (valid only for avr and xmega). */
#define  IRQ_ANY_EDGE         2
/** \brief Falling edge generates an interrupt request. */
#define  IRQ_FALLING_EDGE     3
/** \brief Rising edge generates an interrupt request. */
#define  IRQ_RISING_EDGE      4
/** \brief interrupt is disabled. (valid only for xmega)*/
#define  IRQ_IS_DISABLED      5
/** \brief interrupt is not changed previous state. (valid only for xmega)*/
#define  IRQ_IS_NOT_CHANGED   6

/******************************************************************************
                   Types section
******************************************************************************/
/** \brief numbers of possible interrupt. */
typedef enum
{
#if defined(AT91SAM7X256) || defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || \
    defined(AT90USB1287) || defined(ATMEGA128RFA1) || defined(AT32UC3A0512)
/** \brief number of valid interrupt for arm and avr32. */
  IRQ_0 = 0,
/** \brief number of valid interrupt for arm and avr32. */
  IRQ_1 = 1,
/** \brief number of valid interrupt for avr32. */
  IRQ_2 = 2,
/** \brief number of valid interrupt for avr32. */
  IRQ_3 = 3,
/** \brief number of valid interrupt for avr32. */
  IRQ_4 = 4,
/** \brief number of valid interrupt for avr(only rcb platform) and  avr32. */
  IRQ_5 = 5,
/** \brief number of valid interrupt for avr and avr32. */
  IRQ_6 = 6,
/** \brief number of valid interrupt for avr and avr32. */
  IRQ_7 = 7,
#endif
#if defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3)
    IRQ_A0,
    IRQ_A1,
    IRQ_B0,
    IRQ_B1,
    IRQ_C0,
    IRQ_C1,
    IRQ_D0,
    IRQ_D1,
    IRQ_E0,
    IRQ_E1,
    IRQ_F0,
    IRQ_F1,
#if defined(ATXMEGA128A1)
    IRQ_H0,
    IRQ_H1,
    IRQ_J0,
    IRQ_J1,
    IRQ_K0,
    IRQ_K1,
#endif
#endif
#if defined(AT91SAM3S4C)
  IRQ_PORT_A,
  IRQ_PORT_B,
  IRQ_PORT_C,
#endif
  IRQ_LIMIT
} HAL_IrqNumber_t;

#if defined(AT91SAM7X256) || defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || \
    defined(AT90USB1287) || defined(ATMEGA128RFA1) || defined(AT32UC3A0512)
/** \brief interrupt activation condition. */
typedef uint8_t HAL_IrqMode_t;
#endif
#if defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3)
/** \brief interrupt activation condition. */
typedef struct
{
  uint32_t pin0 : 3;
  uint32_t pin1 : 3;
  uint32_t pin2 : 3;
  uint32_t pin3 : 3;
  uint32_t pin4 : 3;
  uint32_t pin5 : 3;
  uint32_t pin6 : 3;
  uint32_t pin7 : 3;
} HAL_IrqMode_t;
#endif
#if defined(AT91SAM3S4C)
typedef uint8_t HAL_IrqMode_t[32];
#endif

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Registers the user's irqNumber external interrupt

\param[in]
   irqNumber - IRQ number. Must be chosen from: \n
            IRQ_0 (for arm and avr32) \n
            IRQ_1 (for arm and avr32) \n
            IRQ_2 (for avr32) \n
            IRQ_3 (for avr32) \n
            IRQ_4 (for avr32) \n
            IRQ_5 (for avr (only rcb platform) and avr32) \n
            IRQ_6 (for avr and avr32) \n
            IRQ_7 (for avr and avr32) \n
            IRQ_PC (for xmega. P - port name, C - interrupt number. For example: IRQ_D1) \n
            IRQ_PORT_x (for cortex m3, x stands for port name) \n
\param[in]
   irqMode - Controls the sort of interrupt. For avr and arm must be chosen from: \n
            IRQ_LOW_LEVEL         // The low level generates an interrupt request. \n
            IRQ_HIGH_LEVEL        // The high level generates an interrupt request (valid for arm, avr32, cortex m3). \n
            IRQ_ANY_EDGE          // Any edge generates an interrupt request (valid for avr and cortex m3).  \n
            IRQ_FALLING_EDGE      // Falling edge generates an interrupt request. \n
            IRQ_RISING_EDGE       // Rising edge generates an interrupt request. \n

            irqMode is bit field for xmega . Bit field includes members pinX (X - pin number of selected port). \n
            All members that bit field must have value:

            IRQ_LOW_LEVEL         // The low level generates an interrupt request. \n
            IRQ_ANY_EDGE          // Any edge generates an interrupt request.  \n
            IRQ_FALLING_EDGE      // Falling edge generates an interrupt request. \n
            IRQ_RISING_EDGE       // Rising edge generates an interrupt request. \n
            IRQ_IS_DISABLED       // Pin is disabled for interrupt request. \n
            IRQ_IS_NOT_CHANGED    // Pin is not changed previous state.

\param[in]
   f         - user's interrupt handler. Handler must be executed less than 100 us.
\return
  -1 - if irqNumber is out of range, \n
       not valid irq mode, \n
       such interrupt has been already registered. \n
   0 - otherwise.
******************************************************************************/
int HAL_RegisterIrq(HAL_IrqNumber_t irqNumber, HAL_IrqMode_t irqMode, void (*f)(void));

/**************************************************************************//**
\brief Enables the irqNumber interrupt.
\param[in]
   irqNumber - IRQ number
\return
   -1 - if irqNumber is out of range or has not been registered yet. \n
    0 - otherwise.
******************************************************************************/
int HAL_EnableIrq(HAL_IrqNumber_t irqNumber);

/**************************************************************************//**
\brief Disables the irqNumber interrupt.
\param[in]
   irqNumber - IRQ number
\return
   -1 - if irqNumber is out of range or has not been registered yet. \n
    0 - otherwise.
******************************************************************************/
int HAL_DisableIrq(HAL_IrqNumber_t irqNumber);

/**************************************************************************//**
\brief Unregisters the user's irqNumber interrupt
\param[in]
   irqNumber - IRQ number
\return
   -1 - if irqNumber is out of range or has not been registered yet. \n
    0 - otherwise.
******************************************************************************/
int HAL_UnregisterIrq(HAL_IrqNumber_t irqNumber);

#endif /* _IRQ_H */
//eof irq.h
