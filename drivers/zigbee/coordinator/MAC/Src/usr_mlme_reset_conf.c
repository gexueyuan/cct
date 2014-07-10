/**
 * @file usr_mlme_reset_conf.c
 *
 * @brief This file contains user call back function for MLME-RESET.confirm.
 *
 * $Id: usr_mlme_reset_conf.c 20399 2010-02-18 08:10:29Z sschneid $
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
#include <stdio.h>
#include "mac_api.h"
#include "nwkStateMachine.h"
#include "nwk.h"
#include "nwk_api.h"

/* === Macros ============================================================== */

/* === Globals ============================================================= */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

void usr_mlme_reset_conf(uint8_t status)
{
    if (NwkState != NWK_MODULE_ID_RESET)
      return;

    if (status == MAC_SUCCESS)
    {
#ifdef SIO_HUB
       if (gNwk_nib.deviceType == DEVICE_TYPE_ROUTER)
          printf("Searching network\n");
#endif
        nlme_reset_conf_t *nrc;
        nrc  = (nlme_reset_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)gNwk_conf_buf_ptr));    
   
        nrc->status  =  status;
        nrc->cmdcode =  NLME_RESET_CONFIRM;
    
        /* Append the mlme reset confirm to the MAC-NHLE queue */
        qmm_queue_append(&nwk_aps_q, (buffer_t *)gNwk_conf_buf_ptr);        
        
    }
    else
    {
        /* Something went wrong; restart. */
        wpan_mlme_reset_req(true);
    }
}

/* EOF */
