/**
 * @file usr_mcps_data_conf.c
 *
 * @brief This file contains user call back function for MCPS-DATA.confirm.
 *
 * $Id: usr_mcps_data_conf.c 21621 2010-04-15 08:09:37Z sschneid $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === Includes ============================================================= */

#include <stdint.h>
#include <stdbool.h>
#include "mac_api.h"
#include <stdio.h>
#include <inttypes.h>

/* === Macros ============================================================== */


/* === Globals ============================================================= */


/* === Prototypes ========================================================== */


/* === Implementation ====================================================== */

#ifdef ENABLE_TSTAMP
void usr_mcps_data_conf(uint8_t msduHandle, uint8_t status, uint32_t Timestamp)
#else
void usr_mcps_data_conf(uint8_t msduHandle, uint8_t status)
#endif  /* ENABLE_TSTAMP */
{
#ifdef SIO_HUB
   // char sio_array[255];

   // sprintf(sio_array, "Result frame with handle %" PRIu8 ": ", msduHandle);
   // printf(sio_array);
#endif

    if (status == MAC_SUCCESS)
    {
#ifdef SIO_HUB
       // printf("Success\n");
#endif
    }
    else if (status == MAC_TRANSACTION_OVERFLOW)
    {
#ifdef SIO_HUB
        /* Frame could not be placed into the indirect queue. */
      //  printf("Transaction overflow\n");
#endif
    }
    else if (status == MAC_TRANSACTION_EXPIRED)
    {
#ifdef SIO_HUB
        /*
         * Frame could not be delivered to the target node within
         * the proper time.
         */
      //  printf("Transaction expired\n");
#endif
    }


    /* Keep compiler happy. */
    msduHandle = msduHandle;
#ifdef ENABLE_TSTAMP
    Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */
}
/* EOF */
