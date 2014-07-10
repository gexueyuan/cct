/**
 * @file tal_config.h
 *
 * @brief File contains TAL configuration parameters.
 *
 * $Id: tal_config.h 29695 2011-12-07 16:44:22Z uwalter $
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
#ifndef TAL_CONFIG_H
#define TAL_CONFIG_H

/* === INCLUDES ============================================================ */

#include "at86rf231.h"

/* === EXTERNALS =========================================================== */

/* === MACROS ============================================================== */

#define TAL_RADIO_WAKEUP_TIME_SYM       (TAL_CONVERT_US_TO_SYMBOLS(SLEEP_TO_TRX_OFF_TYP_US))
#define TAL_FIRST_TIMER_ID              (0)

#ifndef ANTENNA_DEFAULT
#define ANTENNA_DEFAULT                 (ANT_CTRL_2)
#endif

#ifdef ENABLE_FTN_PLL_CALIBRATION
/*
 * PLL calibration and filter tuning timer timeout in minutes
 */
#define TAL_CALIBRATION_TIMEOUT_MIN         (5UL)

/*
 * PLL calibration and filter tuning timer timeout in us,
 */
#define TAL_CALIBRATION_TIMEOUT_US          ((TAL_CALIBRATION_TIMEOUT_MIN) * (60UL) * (1000UL) * (1000UL))
#endif  /* ENABLE_FTN_PLL_CALIBRATION */

/* === TYPES =============================================================== */

/* Timer ID's used by TAL */
#ifdef BEACON_SUPPORT
// Beacon Support
#ifdef ENABLE_FTN_PLL_CALIBRATION
typedef enum tal_timer_id_tag
{
    TAL_CSMA_CCA                    = (TAL_FIRST_TIMER_ID),
    TAL_CSMA_BEACON_LOSS_TIMER      = (TAL_FIRST_TIMER_ID + 1),
    TAL_CALIBRATION                 = (TAL_FIRST_TIMER_ID + 2)
#ifdef SW_CONTROLLED_CSMA
                                      ,
    TAL_T_BOFF                      = (TAL_FIRST_TIMER_ID + 3)
#endif
} SHORTENUM tal_timer_id_t;

#ifdef SW_CONTROLLED_CSMA
#define NUMBER_OF_TAL_TIMERS        (4)
#else
#define NUMBER_OF_TAL_TIMERS        (3)
#endif

#else
typedef enum tal_timer_id_tag
{
    TAL_CSMA_CCA                    = (TAL_FIRST_TIMER_ID),
    TAL_CSMA_BEACON_LOSS_TIMER      = (TAL_FIRST_TIMER_ID + 1)
#ifdef SW_CONTROLLED_CSMA
    ,
    TAL_T_BOFF                      = (TAL_FIRST_TIMER_ID + 2)
#endif
} SHORTENUM tal_timer_id_t;

#ifdef SW_CONTROLLED_CSMA
#define NUMBER_OF_TAL_TIMERS        (3)
#else
#define NUMBER_OF_TAL_TIMERS        (2)
#endif
#endif  /* ENABLE_FTN_PLL_CALIBRATION */
#else /* No BEACON_SUPPORT */
#ifdef ENABLE_FTN_PLL_CALIBRATION
typedef enum tal_timer_id_tag
{
    TAL_CALIBRATION                 = (TAL_FIRST_TIMER_ID)
#ifdef SW_CONTROLLED_CSMA
    ,
    TAL_T_BOFF                      = (TAL_FIRST_TIMER_ID + 1)
#endif
}SHORTENUM  tal_timer_id_t;

#ifdef SW_CONTROLLED_CSMA
#define NUMBER_OF_TAL_TIMERS        (2)
#else
#define NUMBER_OF_TAL_TIMERS        (1)
#endif
#else
#ifdef SW_CONTROLLED_CSMA
typedef enum tal_timer_id_tag
{
    TAL_T_BOFF                      = (TAL_FIRST_TIMER_ID)
} SHORTENUM tal_timer_id_t;
#endif
#ifdef SW_CONTROLLED_CSMA
#define NUMBER_OF_TAL_TIMERS        (1)
#else
#define NUMBER_OF_TAL_TIMERS        (0)
#endif
#endif  /* ENABLE_FTN_PLL_CALIBRATION */
#endif  /* BEACON_SUPPORT */

#if (NUMBER_OF_TAL_TIMERS > 0)
#define TAL_LAST_TIMER_ID    (TAL_FIRST_TIMER_ID + NUMBER_OF_TAL_TIMERS - 1) // -1: timer id starts with 0
#else
#define TAL_LAST_TIMER_ID    (TAL_FIRST_TIMER_ID)
#endif

#ifdef ENABLE_QUEUE_CAPACITY
#define TAL_INCOMING_FRAME_QUEUE_CAPACITY   (255)
#endif  /* ENABLE_QUEUE_CAPACITY */

/* === PROTOTYPES ========================================================== */


#endif /* TAL_CONFIG_H */
