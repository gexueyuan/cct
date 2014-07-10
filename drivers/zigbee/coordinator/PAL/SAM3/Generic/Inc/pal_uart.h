/**
 * @file pal_uart.h
 *
 * @brief PAL uart internal functions prototypes
 *
 * This header contains the function prototypes for transmit,
 * receive functions and macros used in UART module.
 *
 * $Id: pal_uart.h 31128 2012-03-02 12:31:53Z mahendran.p $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef PAL_UART_H
#define PAL_UART_H

/* === Includes ============================================================= */

#include "app_config.h"
#include "bmm.h"
#ifdef UART_DMA_TX
typedef struct uart_dma_tx_queue_tag
{
    uint8_t   *buffer[20];
    uint8_t   length[20];
    buffer_t  *free[20];
    uint8_t   head;
    uint8_t   tail;
    uint8_t   count;
    uint8_t   isRuning;
} uart_dma_tx_queue_t;   

extern uart_dma_tx_queue_t uart_dma_tx_queue;
#endif   
/* === Types ================================================================ */

#if ((defined UART0) || (defined UART1))

/**
 * Structure containing the transmit and receive buffer
 * and also the buffer head, tail and count
 */
typedef struct uart_communication_buffer_tag
{
    /* Transmit buffer */
#ifndef UART_DMA_TX  
    uint8_t tx_buf[UART_MAX_TX_BUF_LENGTH];
#endif
    /* Receive buffer */
    uint8_t *rx_buf[20];

    /* Transmit buffer head */
    uint16_t tx_buf_head;

    /* Transmit buffer tail */
    uint16_t tx_buf_tail;

    /* Receive buffer head */
    uint8_t *rx_buf_head;

    /* Receive buffer tail */
    uint16_t rx_buf_tail;

    /* Number of bytes in transmit buffer */
    uint8_t tx_count;

    /* Number of bytes in receive buffer */
    uint16_t rx_count;

} uart_communication_buffer_t;

/* Irq handlers of UART */
typedef void (*uart_irq_handler_t) (void);

/* === Externals ============================================================ */
extern uint32_t timeOut;
extern uint8_t firstInt,packets;
extern uint8_t *head[];
extern uart_communication_buffer_t uart_0_buffer;
/* === Macros =============================================================== */

/* UART baud rate of in bits per second. */
#define UART_BAUD_115k2               (115200)//(115200)

#define DISABLE_ALL_INTERRUPTS       (0xFFFFFFFF)
/*
 * Value to be loaded in the SCBR register of UART to obtain a baud rate of
 * 9600 bps.
 * As given in datasheet Baud Rate = Selected Clock / 16 /CD
 * hence CD = Baud Rate / Selected Clock / 16
 */
#define UART_BAUD_DIVISOR           (uint16_t) ((F_CPU / UART_BAUD_115k2) / 16)

/* UART0 */
#ifdef UART0

/* UART Base */
#ifndef PA
  #define UART0_BASE                 (USART3)
  #define UART0_IRQ                  (USART3_IRQn)   
#else
    #ifdef STM32F10X_MD  
      #define UART0_BASE                 (USART2)
      #define UART0_IRQ                  (USART2_IRQn)     
    #elif defined STM32F051
      #define UART0_BASE                 (USART1)
      #define UART0_IRQ                  (USART1_IRQn)     
    #elif defined STM32F4XX
      #define UART0_BASE                 (USART1)   
      #define UART0_IRQ                  (USART1_IRQn)     
    #endif
#endif

/* Enables the RX interrupt of UART0 */
#define ENABLE_UART_0_RX_INT()      (USART_ITConfig(UART0_BASE, USART_IT_RXNE , ENABLE))

/* Enables the TX interrupt of UART0 */
#define ENABLE_UART_0_TX_INT()      ( USART_ITConfig(UART0_BASE, USART_IT_TXE, ENABLE))

/* Disables the RX interrupt of UART0 */
#define DISABLE_UART_0_RX_INT()     (USART_ITConfig(UART0_BASE, USART_IT_RXNE , DISABLE))

/* Disables the TX interrupt of UART0 */
#define DISABLE_UART_0_TX_INT()     (USART_ITConfig(UART0_BASE, USART_IT_TXE , DISABLE))

/* Writes data in the UART data register if there is no transmission ongoing */
#if defined (STM32F10X_MD)  || defined (STM32F4XX)
#define WRITE_TO_UART_0(tx_data)   do {                            \
        while (USART_GetFlagStatus(UART0_BASE, USART_FLAG_TXE) == 0);      \
        UART0_BASE->DR = (tx_data & (uint16_t)0x01FF);                          \
    } while (0);

/* Reads data from the UART data register if there is data in receiver */
#define READ_FROM_UART_0(rx_data)   do {                            \
        while (USART_GetFlagStatus(UART0_BASE, USART_FLAG_RXNE) == 0);          \
        rx_data = (uint16_t)(UART0_BASE->DR & (uint16_t)0x01FF);                               \
    } while (0);
#elif defined (STM32F051)
#define WRITE_TO_UART_0(tx_data)   do {                            \
        while (USART_GetFlagStatus(UART0_BASE, USART_FLAG_TXE) == 0);      \
        UART0_BASE->TDR = (tx_data & (uint16_t)0x01FF);                          \
    } while (0);

/* Reads data from the UART data register if there is data in receiver */
#define READ_FROM_UART_0(rx_data)   do {                            \
        while (USART_GetFlagStatus(UART0_BASE, USART_FLAG_RXNE) == 0);          \
        rx_data = (uint16_t)(UART0_BASE->RDR & (uint16_t)0x01FF);                               \
    } while (0);
#endif
/** Pins used for UART transfer */
#define PINS_UART0      PIN_UART0_TXD, PIN_UART0_RXD
#define PINS_UART0_FLOW  PIN_UART0_RTS, PIN_UART0_CTS
#endif  /* UART0 */

/* UART1 */
#ifdef UART1

/* UART Base */
#define UART1_BASE                 (UART_1)

/* Enables the RX interrupt of UART0 */
#define ENABLE_UART_1_RX_INT()      (UART1_BASE->UART_IER = UART_IER_RXRDY)

/* Enables the TX interrupt of UART0 */
#define ENABLE_UART_1_TX_INT()      (UART1_BASE->UART_IER = UART_IER_TXRDY)

/* Disables the RX interrupt of UART0 */
#define DISABLE_UART_1_RX_INT()     (UART1_BASE->UART_IDR = UART_IDR_RXRDY)

/* Disables the TX interrupt of UART0 */
#define DISABLE_UART_1_TX_INT()     (UART1_BASE->UART_IDR = UART_IDR_TXRDY)

/* Writes data in the UART data register if there is no transmission ongoing */
#define WRITE_TO_UART_1(tx_data)   do {                             \
        while ((UART1_BASE->UART_SR & UART_SR_TXEMPTY) == 0);       \
        UART1_BASE->UART_THR = tx_data;                             \
    } while (0);

/* Reads data from the UART data register if there is data in receiver */
#define READ_FROM_UART_1(rx_data)   do {                            \
        while ((UART1_BASE->UART_SR & UART_SR_RXRDY) == 0);         \
        rx_data = UART1_BASE->UART_RHR;                             \
    } while (0);

/** Pins used for UART transfer */
#define PINS_UART1      PIN_UART1_RXD, PIN_UART1_TXD

#endif  /* UART1 */

/* === Prototypes =========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

    void sio_uart_0_init(uint32_t baud_rate);
    void sio_uart_1_init(uint32_t baud_rate);
    uint8_t sio_uart_0_rx(uint8_t *data, uint16_t max_length);
    uint8_t sio_uart_1_rx(uint8_t *data, uint8_t max_length);
    uint8_t sio_uart_0_tx(uint8_t *data, uint8_t length);
    uint8_t sio_uart_1_tx(uint8_t *data, uint8_t length);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* #if ((defined UART0) || (defined UART1)) */

#endif /* PAL_UART_H */
/* EOF */
