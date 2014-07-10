/**
 * @file sio_handler.h
 *
 * @brief This file contains macros and function prototypes for SIO handling.
 *
 * $Id: sio_handler.h 29087 2011-11-02 14:59:39Z yogesh.bellan $
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
#ifndef SIO_HANDLER_H
#define SIO_HANDLER_H

/* === Includes ============================================================= */


/* === Macros =============================================================== */
#ifdef USB0
#define SIO_CHANNEL  SIO_2
#elif (defined(UART0))
#define SIO_CHANNEL  SIO_0
#elif (defined(UART1))
#define SIO_CHANNEL  SIO_1
#elif (defined(UART2))
#define SIO_CHANNEL  SIO_3
#elif (defined(UART3))
#define SIO_CHANNEL  SIO_4
#endif

/* Function aliases allowing IAR and GCC functions use the same way */
#if ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__))
#define sio_putchar(data)       _sio_putchar(data)
#define sio_getchar()           _sio_getchar()
#define sio_getchar_nowait()    _sio_getchar_nowait()
#define fdevopen(a,b)           /* IAR does not use the fdevopen - the __write() (or __read()) must exist instead (here in file write.c) */
#else
#define sio_putchar(data)       _sio_putchar(data, NULL)
#define sio_getchar()           _sio_getchar(NULL)
#define sio_getchar_nowait()    _sio_getchar_nowait(NULL)
#if (PAL_GENERIC_TYPE == ARM7)
#define fdevopen(a,b)
#endif
#endif

/* === Types ================================================================ */

/* === Externals ============================================================ */


/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

    void init_sio(void);
    void sio_task(void);
    void sio_write(uint8_t *message);

#if ((defined __ICCAVR__) || (defined __ICCARM__) || (defined __ICCAVR32__))
int _sio_putchar(char data);
int _sio_getchar(void);
int _sio_getchar_nowait(void);
#else
int _sio_putchar(char data, FILE *dummy);
int _sio_getchar(FILE *dummy);
int _sio_getchar_nowait(FILE *dummy);
#endif
void sio_binarywrite(uint8_t *d, int16_t sz);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* SIO_HANDLER_H */

/* EOF */
