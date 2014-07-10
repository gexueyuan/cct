/************************************************************************//**
  \file halUart.h

  \brief
    Header for hw UART module

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    17.11.2010 D. Loskutnikov - Created.
******************************************************************************/
#ifndef _HALUART_H_
#define _HALUART_H_

/******************************************************************************
                        Includes section
******************************************************************************/
#include <halUsart.h>
#include <usart.h>

/******************************************************************************
                        Defines section
******************************************************************************/
// Cast to UartChannel_t to UsartChannel_t for minimum changes in applications
#define UART_CHANNEL_0  ((UsartChannel_t)UART0)
#define UART_CHANNEL_1  ((UsartChannel_t)UART1)

/******************************************************************************
                             Types section
******************************************************************************/
typedef Uart* UartChannel_t;  // Pointer to uart hw registers
// UART functionality is a subset of USART one.
// Common descriptor is used to make API compatible.
typedef HAL_UsartDescriptor_t HAL_UartDescriptor_t;
typedef HalUsartService_t HalUartService_t;

/******************************************************************************
                    Functions prototypes section
******************************************************************************/

/**************************************************************************//**
\brief UART task handler
******************************************************************************/
void halSigUartHandler(void);

/**************************************************************************//**
\brief UART0 ISR
******************************************************************************/
void uart0Handler(void);

/**************************************************************************//**
\brief UART1 ISR
******************************************************************************/
void uart1Handler(void);

/**************************************************************************//**
\brief Check if descriptor is valid for available UARTs

\param[in] desc UART descriptor
\return 0 if not
******************************************************************************/
bool halIsUartValid(HAL_UartDescriptor_t *desc);

/**************************************************************************//**
\brief Check if specific UART is opened/registered

\param[in] desc UART descriptor
\return 0 if not
******************************************************************************/
bool halIsUartOpened(HAL_UartDescriptor_t *desc);

/**************************************************************************//**
\brief Register UART descriptor to make UART 'open', init pins/clocks/irqs

\param[in] desc UART descriptor
\return 0 if error
******************************************************************************/
bool halOpenUart(HAL_UartDescriptor_t *desc);

/**************************************************************************//**
\brief Close specific UART, stop clocks, free pins

\param[in] desc UART descriptor
\return 0 if error
******************************************************************************/
bool halCloseUart(HAL_UartDescriptor_t *desc);

#endif // _HALUART_H_
// eof halUart.h

