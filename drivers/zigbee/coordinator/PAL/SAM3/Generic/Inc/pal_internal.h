/**
 * @file pal_internal.h
 *
 * @brief PAL internal functions prototypes for SAM3 MCUs
 *
 * $Id: pal_internal.h 28470 2011-09-08 07:16:41Z mahendran.p $
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
#ifndef PAL_INTERNAL_H
#define PAL_INTERNAL_H

/* === Includes ============================================================= */
#include <stdint.h>
#include <stdbool.h>
#ifdef STM32F051
#include "stm32f0xx_gpio.h"
#elif defined STM32F10X_MD
#include "stm32f10x_gpio.h"
#elif defined STM32F4XX
#include "stm32f4xx_gpio.h"
#endif
/* === Types ================================================================ */
typedef struct
{

    /* Bitmask indicating which pin(s) to configure */
    uint16_t mask;
    /* Pointer to the PIO controller which has the pin(s).*/
    GPIO_TypeDef    *pio;
    /* Peripheral ID of the PIO controller which has the pin(s).*/
    uint8_t id;
    /* Pin type */
    uint8_t type;
    /* Pin attribute */
    GPIOMode_TypeDef attribute;
#ifdef STM32F10X_MD      
#elif defined (STM32F051) || defined (STM32F4XX) 
    GPIOOType_TypeDef outType;   
    GPIOPuPd_TypeDef pupdType;  
#endif    

} pin_t;

typedef struct
{

    /*Pointer to the source pin instance */
    const pin_t *pPin;
    /* Interrupt handler */
    void (*pio_handler)(const pin_t *);

} interruptSource_t;

/* === Externals ============================================================ */


/* === Macros ================================================================ */
/* The pin is controlled by the associated signal of peripheral A */
#define PIO_PERIPH_A                0
/* The pin is controlled by the associated signal of peripheral B */
#define PIO_PERIPH_B                1
/* The pin is controlled by the associated signal of peripheral C */
#define PIO_PERIPH_C                2
/* The pin is controlled by the associated signal of peripheral D */
#define PIO_PERIPH_D                3
/* The pin is an input */
#define PIO_INPUT                   4
/* The pin is an output and has a default level of 0 */
#define PIO_OUTPUT_0                5
/* The pin is an output and has a default level of 1 */
#define PIO_OUTPUT_1                6
/* Default pin configuration (no attribute) */
#define PIO_DEFAULT                 (0 << 0)
/* The internal pin pull-up is active */
#define PIO_PULLUP                  (1 << 0)
/* The internal glitch filter is active */
#define PIO_DEGLITCH                (1 << 1)
/* The pin is open-drain */
#define PIO_OPENDRAIN               (1 << 2)
/*  The internal debouncing filter is active */
#define PIO_DEBOUNCE                (1 << 3)
/* Enable additional interrupt modes */
#define PIO_IT_AIME                 (1 << 4)
/* Interrupt High Level/Rising Edge detection is active */
#define PIO_IT_RE_OR_HL             (1 << 5)
/* Interrupt Edge detection is active */
#define PIO_IT_EDGE                 (1 << 6)
/* Low level interrupt is active */
#define PIO_IT_LOW_LEVEL            (0 | 0 | PIO_IT_AIME)
/* High level interrupt is active */
#define PIO_IT_HIGH_LEVEL           (PIO_IT_RE_OR_HL | 0 | PIO_IT_AIME)
/* Falling edge interrupt is active */
#define PIO_IT_FALL_EDGE            (0 | PIO_IT_EDGE | PIO_IT_AIME)
/* Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE            (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)

#define PIO_LISTSIZE(pPins)    (sizeof(pPins) / sizeof(pin_t))

/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

    void gpio_init(void);
    void trx_interface_init(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

uint32_t pal_get_peripheral_clock_status(RCC_ClocksTypeDef  *RCC_Clocks);
void pal_peripheral_clock_enable(uint8_t RCC_APB2Periph);
void pal_peripheral_clock_disable(uint32_t RCC_APB2Periph);
bool pal_pio_configure(const pin_t *list, uint32_t size);
bool pal_pio_get(const pin_t *pin);
void pal_pio_initialize_interrupts(uint32_t priority);
void pal_pio_configure_interrupt(const pin_t *pPin, void (*handler)(const pin_t *));
void pal_pio_disable_interrupt(const pin_t *pPin);
void pal_pio_enable_interrupt(const pin_t *pPin);

#endif  /* PAL_INTERNAL_H */
/* EOF */
