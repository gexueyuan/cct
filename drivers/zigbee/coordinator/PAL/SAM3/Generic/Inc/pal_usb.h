/**
 * @file pal_usb.h
 *
 * @brief PAL usb internal functions prototypes
 *
 * This header contains the function prototypes for transmit,
 * receive functions and macros used in USB module.
 *
 * $Id: pal_usb.h 31133 2012-03-05 05:26:08Z mahendran.p $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */
#ifndef PAL_USB_H
#define PAL_USB_H

#include "pal.h"

#define USB_INT_BUF_MAX_SIZE          (250)


typedef struct usb_communication_buffer_tag
{
    /* Transmit buffer */
    uint8_t tx_buf[USB_TX_BUF_SIZE];

    /* Receive buffer */
    uint8_t rx_buf[USB_RX_BUF_SIZE];

    /* Transmit buffer head */
    uint8_t tx_buf_head;

    /* Transmit buffer tail */
    uint8_t tx_buf_tail;

    /* Receive buffer head */
    uint8_t rx_buf_head;

    /* Receive buffer tail */
    uint8_t rx_buf_tail;

    /* Number of bytes in transmit buffer */
    uint8_t tx_count;

    /* Number of bytes in receive buffer */
    uint8_t rx_count;

} usb_communication_buffer_t;

void sio_usb_0_init(void);
uint8_t sio_usb_0_tx(uint8_t *data, uint8_t length);
uint8_t sio_usb_0_rx(uint8_t *data, uint8_t max_length);
void usb0_tx_complete_handler(void *pArg , uint8_t   status,
                              uint32_t transferred, uint32_t remaining);
void usb0_rx_complete_handler(void *pArg , uint8_t   status,
                              uint32_t received, uint32_t remaining);

#endif /*PAL_USB_H*/
/* EOF */
