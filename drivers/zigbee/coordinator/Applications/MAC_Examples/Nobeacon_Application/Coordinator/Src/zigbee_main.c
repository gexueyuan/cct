/**
 * @file main.c
 *
 * @brief MAC Example Nobeacon Application - Coordinator
 *
 * This is the source code of a simple MAC example. It implements the
 * firmware for the coordinator of a network with star topology.
 *
 * The coordinator puts the data in the indirect queue periodically
 * and transmits data frames based on the periodic poll request from the device.
 *
 * The Device receives the data from the coordinator and redirects the data
 * back to the coordinator using direct transmission
 *
 * $Id: main.c 28120 2011-08-18 06:07:25Z mahendran.p $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === INCLUDES ============================================================ */

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include "pal.h"
#include "tal.h"
#include "sio_handler.h"
#include "mac_api.h"
#include "app_config.h"
#include "ieee_const.h"

#include "nwk_api.h"
#include "nwkIB.h"
#include "nlmeSetGet.h"

#include "mac_msg_types.h"
#include "nwk.h"
#include "nwkFormation.h"
#include "nwkStateMachine.h"

#include "nwk_msg_types.h"
#include "nwkFrame.h"

#include "mac_internal.h"
#include "nwkIB.h"
#include "main1.h"
#include "nwkNeighbor.h"
#include "nwkNeighborTable.h"
#include "nwk_config.h"
#include "platform_config.h"
#include "pal_uart.h"
/* === TYPES =============================================================== */


/* === MACROS ============================================================== */





/* === GLOBALS ============================================================= */

#ifndef ONLY_TRANSPARENT_MODE
extern void ProceBackGroundMessage(void);//处理后台指令
#endif


/* === PROTOTYPES ========================================================== */

#ifndef _ENDDEVICE_

extern uint8_t mac_rx_enable(void);

#endif


/* === IMPLEMENTATION ====================================================== */


uint8_t mac_rx_enable()
{
    uint8_t status;

    /* Wake up the radio first */
    mac_trx_wakeup();

    /* Turn the receiver on immediately. */
    status = tal_rx_enable(PHY_RX_ON);

    /* Rx is enabled */
    mac_rx_enabled = true;

    return (status);

} /* mac_rx_enable() */



#ifdef SYS_OS
void zigbee_main(void)
#else
int main(void)
#endif
{

    if (wpan_init() != MAC_SUCCESS)
    {
        /*
         * Stay here; we need a valid IEEE address.
         * Check kit documentation how to create an IEEE address
         * and to store it into the EEPROM.
         */
        pal_alert();
    }


    /* Initialize LEDs. */
    pal_led_init();
#ifndef PA
    pal_led(LED_START, LED_ON);         // indicating application is started
    pal_led(LED_NWK_SETUP, LED_OFF);    // indicating network is started
    pal_led(LED_DATA, LED_OFF);         // indicating data reception
#else
    //pal_led(LED_START, LED_OFF);         // indicating application is started
    pal_led(LED_NWK_SETUP, LED_ON);    // indicating network is started
   // pal_led(LED_DATA, LED_OFF);         // indicating data reception
#endif
    //EXTI0_Config();//interrupt中断0配置
    /*
     * The stack is initialized above, hence the global interrupts are enabled
     * here.
     */
    pal_global_irq_enable();

#ifdef SIO_HUB
    /* Initialize the serial interface used for communication with terminal program. */
#ifdef SPI_COMMUNICTION
    spi_Interface_Init(SLAVE);
#else
  
    if (pal_sio_init(SIO_CHANNEL) != MAC_SUCCESS)
    {
        /* Something went wrong during initialization. */
        pal_alert();
    }
	
#endif


#if ((!defined __ICCAVR__) && (!defined __ICCARM__) && (!defined __GNUARM__) && \
     (!defined __ICCAVR32__) && (!defined __GNUAVR32__))
    fdevopen(_sio_putchar, _sio_getchar);
#endif

    /* To make sure the Hyper Terminal Connected to the system*/
   // sio_getchar();
#ifndef PA
    printf("\nNobeacon_Application\n\n");
    printf("\nCoordinator\n\n");
#endif
#endif /* SIO_HUB */

    /*
     * Reset the MAC layer to the default values.
     * This request will cause a mlme reset confirm message ->
     * usr_mlme_reset_conf
     */
#ifdef NIBFromFlash
    pin_t button_B6 = PIN_PUSHBUTTON_1;
    if (ModuleCfgInfo.AtCfgInfo.byModuleCfgFlag != ModuleDefaultConfig.AtCfgInfo.byModuleCfgFlag || pal_pio_get(&button_B6) == 0)
    {
    	wpan_nlme_reset_req(true);
    }
    else
    {
        mac_state = MAC_PAN_COORD_STARTED;
        mac_rx_enable();
    }
#else
#ifdef NO_JOIN

    if (gNwk_nib.deviceType == DEVICE_TYPE_COORDINATOR)
    	mac_state = MAC_PAN_COORD_STARTED;
    else if (gNwk_nib.deviceType == DEVICE_TYPE_ROUTER)
    	mac_state = MAC_COORDINATOR;
    //else mac_state = MAC_ASSOCIATED;

#ifndef _ENDDEVICE_
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
	    	{
        		if (gNwk_nib.symLink == true)
        		{
        			nwkStartLinkStatusTimer();//v2v
                }
        		    mac_rx_enable();
        		
	    	}
#ifndef REDUCE_CHECK_PARMER1
        	else wpan_nlme_reset_req(true);
#endif
#endif
#else
    if (App_System_Para_Pointer->AtCfgInfo.isJoin == true)
    {
    	wpan_nlme_reset_req(true);
    }
    else
    {
        if (gNwk_nib.deviceType == DEVICE_TYPE_COORDINATOR)
        	mac_state = MAC_PAN_COORD_STARTED;
        else if (gNwk_nib.deviceType == DEVICE_TYPE_ROUTER)
        	mac_state = MAC_COORDINATOR;

		if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
		{
			nwkStartLinkStatusTimer();
			mac_rx_enable();
		}
		else wpan_nlme_reset_req(true);
    }
#endif
#endif
    /* Main loop */
    while (1)
    {
        wpan_task();

#ifndef ONLY_TRANSPARENT_MODE
        if (serialMode == USER_MIBEE_FRAME_MODE)
        	ProceBackGroundMessage();
#endif
    }
}


