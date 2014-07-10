/**************************************************************************//**
  \file  halInterrupt.h

  \brief Interface for interrupt registration.

  \author
      Atmel Corporation: http://www.atmel.com \n
      Support email: avr@atmel.com

    Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
    Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
      20/04/10 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _HALINTERRUPT_H
#define _HALINTERRUPT_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <core_cm3.h>

/******************************************************************************
                   Define(s) section
******************************************************************************/
/* interrupt priority for NVIC. 0 - highest priority .... 15 - lowest priority. */
#define INTERRUPT_PRIORITY_0            0
#define INTERRUPT_PRIORITY_1            1
#define INTERRUPT_PRIORITY_2            2
#define INTERRUPT_PRIORITY_3            3
#define INTERRUPT_PRIORITY_4            4
#define INTERRUPT_PRIORITY_5            5
#define INTERRUPT_PRIORITY_6            6
#define INTERRUPT_PRIORITY_7            7
#define INTERRUPT_PRIORITY_8            8
#define INTERRUPT_PRIORITY_9            9
#define INTERRUPT_PRIORITY_10           10
#define INTERRUPT_PRIORITY_11           11
#define INTERRUPT_PRIORITY_12           12
#define INTERRUPT_PRIORITY_13           13
#define INTERRUPT_PRIORITY_14           14
#define INTERRUPT_PRIORITY_15           15

/******************************************************************************
                   Types section
******************************************************************************/
//! Function prototype for exception table items (interrupt handler).
typedef void(* HalIntFunc_t)(void);

// Type of entry in vector table
typedef union
{
  HalIntFunc_t __fun;
  void *__ptr;
} HalIntVector_t;


/*****************************************************************************
                              Prototypes section
******************************************************************************/

/**************************************************************************//**
\brief Register interrupt handler in vector table

\param[in] num - number of irq vector starting from zeroth IRQ handler (IRQ0)
\param[in] handler - vector handler function. Pass NULL to unregister vector
\return true if succeed, false otherwise
******************************************************************************/
bool HAL_InstallInterruptVector(int32_t num, HalIntFunc_t handler);

/**************************************************************************//**
\brief Default NMI interrupt handler.
******************************************************************************/
void halNmiHandler(void);

/**************************************************************************//**
\brief Default HardFault interrupt handler.
******************************************************************************/
void halHardFaultHandler(void);

/**************************************************************************//**
\brief Default MemManage interrupt handler.
******************************************************************************/
void halMemManageHandler(void);

/**************************************************************************//**
\brief Default BusFault interrupt handler.
******************************************************************************/
void halBusFaultHandler(void);

/**************************************************************************//**
\brief Default UsageFault interrupt handler.
******************************************************************************/
void halUsageFaultHandler(void);

/**************************************************************************//**
\brief Default SVC interrupt handler.
******************************************************************************/
void halSvcHandler(void);

/**************************************************************************//**
\brief Default DebugMon interrupt handler.
******************************************************************************/
void halDebugMonHandler(void);

/**************************************************************************//**
\brief Default PendSV interrupt handler.
******************************************************************************/
void halPendsvHandler(void);

/**************************************************************************//**
\brief Default SysTick interrupt handler.
******************************************************************************/
void halSysTickHandler(void);

#endif

// eof halInterrupt.h
