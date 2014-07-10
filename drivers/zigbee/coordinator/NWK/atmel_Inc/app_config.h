/**
 * @file app_config.h
 *
 * @brief These are application-specific resources which are used
 *        in the example application of the device in addition to the
 *        underlaying stack.
 *
 * $Id: app_config.h 28120 2011-08-18 06:07:25Z mahendran.p $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* === Includes ============================================================= */

#include "stack_config.h"
#include "nwk_config.h"

/* === Macros =============================================================== */

/** @brief This is the first timer identifier of the application.
 *
 *  The value of this identifier is an increment of the largest identifier
 *  value used by the MAC.
 */
#if (NUMBER_OF_TOTAL_STACK_TIMERS == 0)
#define APP_FIRST_TIMER_ID          (0)
#else
#define APP_FIRST_TIMER_ID          (LAST_STACK_TIMER_ID + 1)
#endif

/* === Types ================================================================ */

/** Timer ID's used by the Application */
typedef enum
{
    /* App Timers start from APP_FIRST_TIMER_ID */

    /** Application timer id used by the device to poll for pending data. */
    APP_TIMER_POLL_DATA = (APP_FIRST_TIMER_ID),

    /** Application timer id used to switch off LED. */
    APP_TIMER_LED_OFF = (APP_FIRST_TIMER_ID + 1),
    APP_TIMER_INDIRECT_DATA = (APP_FIRST_TIMER_ID + 2),
    APP_TIMER_REQUEST_DATA = (APP_FIRST_TIMER_ID + 3),
    APP_TIMER_UART_TIMEOUT = (APP_FIRST_TIMER_ID + 4),
    APP_TIMER_BC_DATA = (APP_FIRST_TIMER_ID + 5),
    APP_TIMER_BC_DATA1 = (APP_FIRST_TIMER_ID + 6),
    APP_TIMER_BC_DATA2 = (APP_FIRST_TIMER_ID + 7),
    APP_TIMER_BCN_PAYLOAD_UPDATE = (APP_FIRST_TIMER_ID + 8),
    APP_TIMER = (APP_FIRST_TIMER_ID + 9),
} SHORTENUM app_timer_t;

/** Defines the number of timers used by the application. */
#define NUMBER_OF_APP_TIMERS        (APP_TIMER-APP_TIMER_POLL_DATA+1)

/** Defines the total number of timers used by the application and the layers below. */
#define TOTAL_NUMBER_OF_TIMERS      (NUMBER_OF_APP_TIMERS + NUMBER_OF_TOTAL_STACK_TIMERS + NUMBER_OF_NWK_STACK_TIMERS)//����1��permitjoining�Ķ�ʱ��

/** Defines the number of additional large buffers used by the application */
#define NUMBER_OF_LARGE_APP_BUFS    (5)

/** Defines the number of additional small buffers used by the application */
#define NUMBER_OF_SMALL_APP_BUFS    (0)

/**
 *  Defines the total number of large buffers used by the application and the
 *  layers below.
 */
#define TOTAL_NUMBER_OF_LARGE_BUFS  (NUMBER_OF_LARGE_APP_BUFS + NUMBER_OF_LARGE_STACK_BUFS)

/**
 *  Defines the total number of small buffers used by the application and the
 *  layers below.
 */
#define TOTAL_NUMBER_OF_SMALL_BUFS  (NUMBER_OF_SMALL_APP_BUFS + NUMBER_OF_SMALL_STACK_BUFS)

#define TOTAL_NUMBER_OF_BUFS        (TOTAL_NUMBER_OF_LARGE_BUFS + TOTAL_NUMBER_OF_SMALL_BUFS)

/**
 * Defines the USB transmit buffer size
 */
#define USB_TX_BUF_SIZE             (10)

/**
 * Defines the USB receive buffer size
 */
#define USB_RX_BUF_SIZE             (10)

/*
 * USB-specific definitions
 */

/*
 * USB Vendor ID (16-bit number)
 */
#define USB_VID                 0x03EB /* Atmel's USB vendor ID */

/*
 * USB Product ID (16-bit number)
 */
#define USB_PID                 0x2018 /* RZ USB stick product ID */

/*
 * USB Release number (BCD format, two bytes)
 */
#define USB_RELEASE             { 0x00, 0x01 } /* 01.00 */

/*
 * Maximal number of UTF-16 characters used in any of the strings
 * below.  This is only used for compilers that cannot handle the
 * initialization of flexible array members within structs.
 */
#define USB_STRING_SIZE         10

/*
 * String representation for the USB vendor name.
 */
#define USB_VENDOR_NAME L"ATMEL"

/*
 * String representation for the USB product name.
 */
#define USB_PRODUCT_NAME L"RZUSBSTICK"

/**
 * Defines the UART transmit buffer size
 */
#define UART_MAX_TX_BUF_LENGTH      (1280)

/**
 * Defines the UART receive buffer size
 */
#define UART_MAX_RX_BUF_LENGTH      (1782)

/* Offset of IEEE address storage location within EEPROM */
#define EE_IEEE_ADDR                (0)

/* === Externals ============================================================ */


/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_CONFIG_H */
/* EOF */
