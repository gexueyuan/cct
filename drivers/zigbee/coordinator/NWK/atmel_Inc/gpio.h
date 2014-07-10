/**************************************************************************//**
\file  gpio.h

\brief This module contains a set of functions to manipulate GPIO pins.

\author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

\internal
  History:
    21/04/10 A. Khromykh - Created
******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/
#ifndef _GPIO_H
#define _GPIO_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <AT91SAM3S4.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/******************************************************************************
* void gpioX_set() sets GPIOX pin to logical 1 level.
* void gpioX_clr() clears GPIOX pin to logical 0 level.
* void gpioX_make_in makes GPIOX pin as input.
* void gpioX_make_in makes GPIOX pin as output.
* uint8_t gpioX_read() returns logical level GPIOX pin.
* uint8_t gpioX_state() returns configuration of GPIOX port.
*******************************************************************************/
#define HAL_ASSIGN_PIN(name, port, bit) \
INLINE void  GPIO_##name##_set()           {PIO##port->PIO_SODR = bit;} \
INLINE void  GPIO_##name##_clr()           {PIO##port->PIO_CODR = bit;} \
INLINE uint8_t  GPIO_##name##_read()       {return ((PIO##port->PIO_PDSR & bit) != 0l);} \
INLINE uint8_t  GPIO_##name##_state()      {return ((PIO##port->PIO_OSR & bit) != 0l);} \
INLINE void  GPIO_##name##_make_out()      {PIO##port->PIO_OER = bit;} \
INLINE void  GPIO_##name##_make_in()       {PIO##port->PIO_ODR = bit; \
                                            PIO##port->PIO_PUDR = bit; \
                                            PIO##port->PIO_PPDDR = bit;} \
INLINE void  GPIO_##name##_make_pullup()   {PIO##port->PIO_PPDDR = bit; \
                                            PIO##port->PIO_PUER = bit;}\
INLINE void  GPIO_##name##_make_pulldown() {PIO##port->PIO_PUDR = bit; \
                                            PIO##port->PIO_PPDER = bit;}\
INLINE void  GPIO_##name##_toggle()        {if (PIO##port->PIO_ODSR & bit) \
                                              PIO##port->PIO_CODR = bit; \
                                            else \
                                              PIO##port->PIO_SODR = bit;}

/******************************************************************************
                   Inline static functions section
******************************************************************************/
// the macros for the manipulation by PA19
HAL_ASSIGN_PIN(0, A, PIO_PA19);
// the macros for the manipulation by PA20
HAL_ASSIGN_PIN(1, A, PIO_PA20);
// the macros for the manipulation by PC20
HAL_ASSIGN_PIN(2, C, PIO_PC20);
// the macros for the manipulation by PA19
HAL_ASSIGN_PIN(A19, A, PIO_PA19);
// the macros for the manipulation by PA17
HAL_ASSIGN_PIN(A17, A, PIO_PA17);
// the macros for the manipulation by PA18
HAL_ASSIGN_PIN(A18, A, PIO_PA18);

// USART RTS manipulation macros
HAL_ASSIGN_PIN(USART_RTS, A, PIO_PA24);
// USART CTS manipulation macros
HAL_ASSIGN_PIN(USART_CTS, A, PIO_PA25);
// USART DTR manipulation macros
HAL_ASSIGN_PIN(USART_DTR, A, PIO_PA24);
// USART0------------------------------------
// USART0 TXD manipulation macros
HAL_ASSIGN_PIN(USART0_TXD, A, PIO_PA6);
// USART1------------------------------------
// USART1 TXD manipulation macros
HAL_ASSIGN_PIN(USART1_TXD, A, PIO_PA22);

// PA23 manipulation macros
HAL_ASSIGN_PIN(PA23, A, PIO_PA23);

#endif
//eof gpio.h
