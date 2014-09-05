/**
 * @file tal_pib.c
 *
 * @brief This file handles the TAL PIB attributes, set/get and initialization
 *
 * $Id: tal_pib.c 32115 2012-05-18 11:23:47Z yogesh.bellan $
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

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pal.h"
#include "return_val.h"
#include "tal.h"
#include "ieee_const.h"
#include "tal_constants.h"
#include "tal_pib.h"
#include "at86rf231.h"
#include "tal_internal.h"

#include "nwk.h"
#include "cv_cms_def.h"
/* === TYPES =============================================================== */


/* === MACROS ============================================================== */

#ifdef CUSTOM_PWR_TABLE
/*
 * If a customized powr table is used, the power table gets defined within the
 * pal_board.c file.
 */
extern FLASH_DECLARE(int8_t tx_pwr_table[16]);
#else
/*
 * Default translation table converting register values to power levels (dBm).
 */
FLASH_DECLARE(int8_t tx_pwr_table[16]) =
{
    3, /* 3.2 */
    2, /* 2.8 */
    2, /* 2.3 */
    1, /* 1.8 */
    1, /* 1.3 */
    0, /* 0.7 */
    0, /* 0.0 */
    -1,
    -2,
    -3,
    -4,
    -5,
    -7,
    -9,
    -12,
    -17,
};
#endif

/* === GLOBALS ============================================================= */


/* === PROTOTYPES ========================================================== */

static uint8_t limit_tx_pwr(uint8_t tal_pib_TransmitPower);
static uint8_t convert_phyTransmitPower_to_reg_value(uint8_t phyTransmitPower_value);
#ifdef HIGH_DATA_RATE_SUPPORT
static bool apply_channel_page_configuration(uint8_t ch_page);
#endif

/* === IMPLEMENTATION ====================================================== */

/**
 * @brief Initialize the TAL PIB
 *
 * This function initializes the TAL information base attributes
 * to their default values.
 */
void init_tal_pib(void)
{
    tal_pib.MaxCSMABackoffs = TAL_MAX_CSMA_BACKOFFS_DEFAULT;
    tal_pib.MinBE = TAL_MINBE_DEFAULT;
//#ifdef NO_JOIN
    	  tal_pib.CurrentChannel       = App_System_Para_Pointer->AtCfgInfo.byRfChannelID + 11;
		  tal_pib.ShortAddress       = p_cms_param->pid[3] | p_cms_param->pid[2]<<8;
		  tal_pib.PANId                = App_System_Para_Pointer->AtCfgInfo.wMacPanID;
#ifdef ATMEL_RF		  
		  switch (App_System_Para_Pointer->AtCfgInfo.byRfSendPower)
		  {
			  case 0:
				  tal_pib.TransmitPower = 0;
				  break;

			  case 1:
				  tal_pib.TransmitPower = 8;
				  break;

			  case 2:
				  tal_pib.TransmitPower = 10;
				  break;

			  case 3:
				  tal_pib.TransmitPower = 0x0B;
				  break;

			  case 4:
				  tal_pib.TransmitPower = 0x0C;
				  break;

			  case 5:
				  tal_pib.TransmitPower = 0x0D;
				  break;

			  case 6:
				  tal_pib.TransmitPower = 0x0E;
				  break;

		  	  case 7:
		  		  tal_pib.TransmitPower = 0x0F;
		  		  break;

		  	  default :
		  		  tal_pib.TransmitPower = 3;
		  		  break;

		  }
#else
		  tal_pib.TransmitPower      = App_System_Para_Pointer->AtCfgInfo.byRfSendPower;
#endif		  
//#else
//    tal_pib.PANId = TAL_PANID_BC_DEFAULT;
//    tal_pib.ShortAddress = TAL_SHORT_ADDRESS_DEFAULT;
//    tal_pib.CurrentChannel = TAL_CURRENT_CHANNEL_DEFAULT;
//    tal_pib.TransmitPower = limit_tx_pwr(TAL_TRANSMIT_POWER_DEFAULT);
//#endif
	  tal_pib.SupportedChannels = TRX_SUPPORTED_CHANNELS;
	  switch (App_System_Para_Pointer->AtCfgInfo.byRfBaudRate)
	  {
		  case 0:
			  tal_pib.CurrentPage = 0;
			  break;

		  case 1:
			  tal_pib.CurrentPage = 2;
			  break;

		  case 2:
			  tal_pib.CurrentPage = 16;
			  break;

		  case 3:
			  tal_pib.CurrentPage = 17;
			  break;

	  	  default :
	  		  tal_pib.CurrentPage = 0;
	  		  break;
	  }

    //tal_pib.CurrentPage = TAL_CURRENT_PAGE_DEFAULT;
    tal_pib.MaxFrameDuration = TAL_MAX_FRAME_DURATION_DEFAULT;
    tal_pib.SHRDuration = TAL_SHR_DURATION_DEFAULT;
    tal_pib.SymbolsPerOctet = TAL_SYMBOLS_PER_OCTET_DEFAULT;
    tal_pib.MaxBE = TAL_MAXBE_DEFAULT;
    tal_pib.MaxFrameRetries = TAL_MAXFRAMERETRIES_DEFAULT;
    tal_pib.CCAMode = TAL_CCA_MODE_DEFAULT;
    tal_pib.PrivatePanCoordinator = ((App_System_Para_Pointer->AtCfgInfo.byNodeRouterType == DEVICE_TYPE_COORDINATOR) ? true : false);
#ifdef BEACON_SUPPORT
    tal_pib.BattLifeExt = TAL_BATTERY_LIFE_EXTENSION_DEFAULT;
    tal_pib.BeaconOrder = TAL_BEACON_ORDER_DEFAULT;
    tal_pib.SuperFrameOrder = TAL_SUPERFRAME_ORDER_DEFAULT;
#endif  /* BEACON_SUPPORT */

#ifdef PROMISCUOUS_MODE
    tal_pib.PromiscuousMode = TAL_PIB_PROMISCUOUS_MODE_DEFAULT;
#endif

#if (defined SW_CONTROLLED_CSMA) && (defined TX_OCTET_COUNTER)
    tal_tx_octet_cnt = 0;
#endif
}


void copy_tal_pib(void)
{
	if (gPowerUpFlag == true)
	{
    tal_pib.MaxCSMABackoffs = App_System_Para_Pointer->talInfo.MaxCSMABackoffs;
    tal_pib.MinBE = App_System_Para_Pointer->talInfo.MinBE;
    tal_pib.PANId = App_System_Para_Pointer->talInfo.PANId;
    tal_pib.ShortAddress = App_System_Para_Pointer->talInfo.ShortAddress;
    tal_pib.CurrentChannel = App_System_Para_Pointer->talInfo.CurrentChannel;
    tal_pib.SupportedChannels = App_System_Para_Pointer->talInfo.SupportedChannels;
    tal_pib.CurrentPage = App_System_Para_Pointer->talInfo.CurrentPage;
    tal_pib.MaxFrameDuration = App_System_Para_Pointer->talInfo.MaxFrameDuration;
    tal_pib.SHRDuration = App_System_Para_Pointer->talInfo.SHRDuration;
    tal_pib.SymbolsPerOctet = App_System_Para_Pointer->talInfo.SymbolsPerOctet;
    tal_pib.MaxBE = App_System_Para_Pointer->talInfo.MaxBE;
    tal_pib.MaxFrameRetries = App_System_Para_Pointer->talInfo.MaxFrameRetries;
    tal_pib.TransmitPower = limit_tx_pwr(App_System_Para_Pointer->talInfo.TransmitPower);
    tal_pib.CCAMode = App_System_Para_Pointer->talInfo.CCAMode;
    tal_pib.PrivatePanCoordinator = App_System_Para_Pointer->talInfo.PrivatePanCoordinator;
#ifdef BEACON_SUPPORT
    tal_pib.BattLifeExt = App_System_Para_Pointer->talInfo.BattLifeExt;
    tal_pib.BeaconOrder = App_System_Para_Pointer->talInfo.BeaconOrder;
    tal_pib.SuperFrameOrder = App_System_Para_Pointer->talInfo.SuperFrameOrder;
#endif  /* BEACON_SUPPORT */

#ifdef PROMISCUOUS_MODE
    tal_pib.PromiscuousMode = App_System_Para_Pointer->talInfo.PromiscuousMode;
#endif

#if (defined SW_CONTROLLED_CSMA) && (defined TX_OCTET_COUNTER)
    tal_tx_octet_cnt = 0;
#endif
	}
}

/**
 * @brief Write all shadow PIB variables to the transceiver
 *
 * This function writes all shadow PIB variables to the transceiver.
 * It is assumed that the radio does not sleep.
 */
void write_all_tal_pib_to_trx(void)
{
    uint8_t *ptr_to_reg;

    ptr_to_reg = (uint8_t *)&tal_pib.PANId;
    for (uint8_t i = 0; i < 2; i++)
    {
        pal_trx_reg_write((RG_PAN_ID_0 + i), *ptr_to_reg);
        ptr_to_reg++;
    }

    ptr_to_reg = (uint8_t *)&tal_pib.IeeeAddress;
    for (uint8_t i = 0; i < 8; i++)
    {
        pal_trx_reg_write((RG_IEEE_ADDR_0 + i), *ptr_to_reg);
        ptr_to_reg++;
    }

    ptr_to_reg = (uint8_t *)&tal_pib.ShortAddress;
    for (uint8_t i = 0; i < 2; i++)
    {
        pal_trx_reg_write((RG_SHORT_ADDR_0 + i), *ptr_to_reg);
        ptr_to_reg++;
    }

    /* Configure TX_ARET; CSMA and CCA */
    pal_trx_bit_write(SR_CCA_MODE, tal_pib.CCAMode);

#ifdef SW_CONTROLLED_CSMA
    /*
     * If receiver is enabled during backoff periods,
     * CSMA and frame re-transmissions are handled by software.
     * Setup trx for immediate transmission.
     */
    pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
    pal_trx_bit_write(SR_MAX_CSMA_RETRIES, 7);
#else
    pal_trx_bit_write(SR_MIN_BE, tal_pib.MinBE);
    pal_trx_bit_write(SR_MAX_BE, tal_pib.MaxBE);
#endif

    //pal_trx_bit_write(SR_AACK_I_AM_COORD, tal_pib.PrivatePanCoordinator);

    /* set phy parameter */

#ifdef HIGH_DATA_RATE_SUPPORT
    apply_channel_page_configuration(tal_pib.CurrentPage);
#endif

    pal_trx_bit_write(SR_CHANNEL, tal_pib.CurrentChannel);
    {
        //uint8_t reg_value;

       // reg_value = convert_phyTransmitPower_to_reg_value(tal_pib.TransmitPower);
        pal_trx_bit_write(SR_TX_PWR, tal_pib.TransmitPower);
    }

#ifdef PROMISCUOUS_MODE
    if (tal_pib.PromiscuousMode)
    {
        set_trx_state(CMD_RX_ON);
    }
#endif
}


/**
 * @brief Gets a TAL PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * @param[in] attribute TAL infobase attribute ID
 * @param[out] value TAL infobase attribute value
 *
 * @return MAC_UNSUPPORTED_ATTRIBUTE if the TAL infobase attribute is not found
 *         MAC_SUCCESS otherwise
 */
#if (HIGHEST_STACK_LAYER == TAL)
retval_t tal_pib_get(uint8_t attribute, uint8_t *value)
{
    switch (attribute)
    {
        case macMaxCSMABackoffs:
            *value = tal_pib.MaxCSMABackoffs;
            break;

        case macMinBE:
            *value = tal_pib.MinBE;
            break;

        case macPANId:
            *(uint16_t *)value = tal_pib.PANId;
            break;
#ifdef PROMISCUOUS_MODE
        case macPromiscuousMode:
            *(uint16_t *)value = tal_pib.PromiscuousMode;
            break;
#endif
        case macShortAddress:
            *(uint16_t *)value = tal_pib.ShortAddress;
            break;

        case phyCurrentChannel:
            *value = tal_pib.CurrentChannel;
            break;

        case phyChannelsSupported:
            *(uint32_t *)value = tal_pib.SupportedChannels;
            break;

        case phyTransmitPower:
            *value = tal_pib.TransmitPower;
            break;

        case phyCCAMode:
            *value = tal_pib.CCAMode;
            break;

        case phyCurrentPage:
            *value = tal_pib.CurrentPage;
            break;

        case phyMaxFrameDuration:
            *(uint16_t *)value = tal_pib.MaxFrameDuration;
            break;

        case phySymbolsPerOctet:
            *value = tal_pib.SymbolsPerOctet;
            break;

        case phySHRDuration:
            *value = tal_pib.SHRDuration;
            break;

        case macMaxBE:
            *value = tal_pib.MaxBE;
            break;

        case macMaxFrameRetries:
            *value = tal_pib.MaxFrameRetries;
            break;

        case macIeeeAddress:
            *(uint64_t *)value = tal_pib.IeeeAddress;
            break;
#ifdef BEACON_SUPPORT
        case macBattLifeExt:
            *(bool *)value = tal_pib.BattLifeExt;
            break;

        case macBeaconOrder:
            *value = tal_pib.BeaconOrder;
            break;

        case macSuperframeOrder:
            *value = tal_pib.SuperFrameOrder;
            break;

        case macBeaconTxTime:
            *(uint32_t *)value = tal_pib.BeaconTxTime;
            break;
#endif  /* BEACON_SUPPORT */
        case mac_i_pan_coordinator:
            *(bool *)value = tal_pib.PrivatePanCoordinator;
            break;

        case macAckWaitDuration:
            /*
             * AT86RF231 does not support changing this value w.r.t.
             * compliance operation.
             */
            return MAC_UNSUPPORTED_ATTRIBUTE;

        default:
            /* Invalid attribute id */
            return MAC_UNSUPPORTED_ATTRIBUTE;
    }

    return MAC_SUCCESS;
} /* tal_pib_get() */
#endif  /* (HIGHEST_STACK_LAYER != MAC) */


/**
 * @brief Sets a TAL PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * @param attribute TAL infobase attribute ID
 * @param value TAL infobase attribute value to be set
 *
 * @return MAC_UNSUPPORTED_ATTRIBUTE if the TAL info base attribute is not found
 *         TAL_BUSY if the TAL is not in TAL_IDLE state. An exception is
 *         macBeaconTxTime which can be accepted by TAL even if TAL is not
 *         in TAL_IDLE state.
 *         MAC_SUCCESS if the attempt to set the PIB attribute was successful
 *         TAL_TRX_ASLEEP if trx is in SLEEP mode and access to trx is required
 */
retval_t tal_pib_set(uint8_t attribute, pib_value_t *value)
{
    /*
     * Do not allow any changes while ED or TX is done.
     * We allow changes during RX, but it's on the user's own risk.
     */
#if (MAC_SCAN_ED_REQUEST_CONFIRM == 1)
    if (tal_state == TAL_ED_RUNNING)
    {
        ASSERT("TAL is busy" == 0);
        return TAL_BUSY;
    }
#endif /* (MAC_SCAN_ED_REQUEST_CONFIRM == 1) */

    /*
     * Distinguish between PIBs that need to be changed in trx directly
     * and those that are simple variable udpates.
     * Ensure that the transceiver is not in SLEEP.
     * If it is in SLEEP, change it to TRX_OFF.
     */

    switch (attribute)
    {
        case macMaxFrameRetries:
            /*
             * The new PIB value is not immediately written to the
             * transceiver. This is done on a frame-by-frame base.
             */
            tal_pib.MaxFrameRetries = value->pib_value_8bit;
            break;

        case macMaxCSMABackoffs:
            /*
             * The new PIB value is not immediately written to the
             * transceiver. This is done on a frame-by-frame base.
             */
            tal_pib.MaxCSMABackoffs = value->pib_value_8bit;
            break;

#ifdef BEACON_SUPPORT
        case macBattLifeExt:
            tal_pib.BattLifeExt = value->pib_value_bool;
            break;

        case macBeaconOrder:
            tal_pib.BeaconOrder = value->pib_value_8bit;
            break;

        case macSuperframeOrder:
            tal_pib.SuperFrameOrder = value->pib_value_8bit;
            break;

        case macBeaconTxTime:
            tal_pib.BeaconTxTime = value->pib_value_32bit;
            break;

#endif  /* BEACON_SUPPORT */
#ifdef PROMISCUOUS_MODE
        case macPromiscuousMode:
            tal_pib.PromiscuousMode = value->pib_value_8bit;
            if (tal_pib.PromiscuousMode)
            {
                tal_trx_wakeup();

                /* Check if receive buffer is available or queue is not full. */
                if (NULL == tal_rx_buffer)
                {
                    set_trx_state(CMD_PLL_ON);
                    tal_rx_on_required = true;
                }
                else
                {
                    set_trx_state(CMD_RX_ON);
                }
            }
            else
            {
                set_trx_state(CMD_TRX_OFF);
                tal_rx_on_required = false;
            }
            break;
#endif

        default:
            /*
             * Following PIBs require access to trx.
             * Therefore trx must be at least in TRX_OFF.
             */

            if (tal_trx_status == TRX_SLEEP)
            {
                /* While trx is in SLEEP, register cannot be accessed. */
                return TAL_TRX_ASLEEP;
            }

            switch (attribute)
            {
                case macMinBE:
                    tal_pib.MinBE = value->pib_value_8bit;
#ifndef REDUCED_PARAM_CHECK
                    /*
                     * macMinBE must not be larger than macMaxBE or calculation
                     * of macMaxFrameWaitTotalTime will fail.
                     */
                    if (tal_pib.MinBE > tal_pib.MaxBE)
                    {
                        tal_pib.MinBE = tal_pib.MaxBE;
                    }
#endif  /* REDUCED_PARAM_CHECK */
#ifndef SW_CONTROLLED_CSMA
                    pal_trx_bit_write(SR_MIN_BE, tal_pib.MinBE);
#endif
                    break;

                case macPANId:
                    tal_pib.PANId = value->pib_value_16bit;
                    uint8_t *ptr_pan;
                    ptr_pan = (uint8_t *)&tal_pib.PANId;
                    for (uint8_t i = 0; i < 2; i++)
                    {
                        pal_trx_reg_write((RG_PAN_ID_0 + i), *ptr_pan);
                        ptr_pan++;
                    }
                    gNwk_nib.panId = tal_pib.PANId;
                    break;

                case macShortAddress:
                    tal_pib.ShortAddress = value->pib_value_16bit;
                    uint8_t *ptr_shrt;
                    ptr_shrt = (uint8_t *)&tal_pib.ShortAddress;
                    for (uint8_t i = 0; i < 2; i++)
                    {
                        pal_trx_reg_write((RG_SHORT_ADDR_0 + i), *ptr_shrt);
                        ptr_shrt++;
                    }
                    gNwk_nib.networkAddress = tal_pib.ShortAddress;
                    break;

                case phyCurrentChannel:
                    if (tal_state != TAL_IDLE)
                    {
                        return TAL_BUSY;
                    }
                    if ((uint32_t)TRX_SUPPORTED_CHANNELS & ((uint32_t)0x01 << value->pib_value_8bit))
                    {
                        tal_trx_status_t previous_trx_status = TRX_OFF;
                        /*
                         * Set trx to "soft" off avoiding that ongoing
                         * transaction (e.g. ACK) are interrupted.
                         */
                        if (tal_trx_status != TRX_OFF)
                        {
                            previous_trx_status = RX_AACK_ON;   /* any other than TRX_OFF state */
                            do
                            {
                                /* set TRX_OFF until it could be set;
                                 * trx might be busy */
                            }
                            while (set_trx_state(CMD_TRX_OFF) != TRX_OFF);
                        }
                        tal_pib.CurrentChannel = value->pib_value_8bit;
                        pal_trx_bit_write(SR_CHANNEL, tal_pib.CurrentChannel);
                        /* Re-store previous trx state */
                        if (previous_trx_status != TRX_OFF)
                        {
                            /* Set to default state */
                            set_trx_state(CMD_RX_AACK_ON);
                        }
                        gNwk_nib.logicalChannel = tal_pib.CurrentChannel;
                    }
                    else
                    {
                        return MAC_INVALID_PARAMETER;
                    }
                    break;

                case phyCurrentPage:
#ifdef HIGH_DATA_RATE_SUPPORT
                    if (tal_state != TAL_IDLE)
                    {
                        return TAL_BUSY;
                    }
                    else
                    {
                        uint8_t page;
                        tal_trx_status_t previous_trx_status = TRX_OFF;
                        bool ret_val;

                        /*
                         * Changing the channel, channel page or modulation
                         * requires that TRX is in TRX_OFF.
                         * Store current trx state and return to default state
                         * after channel page has been set.
                         */
                        if (tal_trx_status != TRX_OFF)
                        {
                            previous_trx_status = RX_AACK_ON;   /* any other than TRX_OFF state */
                            do
                            {
                                /* set TRX_OFF until it could be set;
                                 * trx might be busy */
                            }
                            while (set_trx_state(CMD_TRX_OFF) != TRX_OFF);
                        }

                        page = value->pib_value_8bit;

                        ret_val = apply_channel_page_configuration(page);

                        if (previous_trx_status != TRX_OFF)
                        {
                            /* Set to default state */
                            set_trx_state(CMD_RX_AACK_ON);
                        }

                        if (ret_val)
                        {
                            tal_pib.CurrentPage = page;
                            gNwk_nib.channelPage = tal_pib.CurrentPage;
                        }
                        else
                        {
                            return MAC_INVALID_PARAMETER;
                        }
                    }
#else
                    if (tal_state != TAL_IDLE)
                    {
                        return TAL_BUSY;
                    }
                    else
                    {
                        uint8_t page;

                        page = value->pib_value_8bit;
                        if (page != 0)
                        {
                            return MAC_INVALID_PARAMETER;
                        }
                    }
#endif  /* #ifdef HIGH_DATA_RATE_SUPPORT */
                    break;

                case macMaxBE:
                    tal_pib.MaxBE = value->pib_value_8bit;
#ifndef REDUCED_PARAM_CHECK
                    /*
                     * macMinBE must not be larger than macMaxBE or calculation
                     * of macMaxFrameWaitTotalTime will fail.
                     */
                    if (tal_pib.MaxBE < tal_pib.MinBE)
                    {
                        tal_pib.MinBE = tal_pib.MaxBE;
                    }
#endif  /* REDUCED_PARAM_CHECK */
#ifndef SW_CONTROLLED_CSMA
                    pal_trx_bit_write(SR_MAX_BE, tal_pib.MaxBE);
#endif
                    break;

                case phyTransmitPower:
                    {
                        uint8_t reg_value;

                        tal_pib.TransmitPower = value->pib_value_8bit;

                        /* Limit tal_pib.TransmitPower to max/min trx values */
                        tal_pib.TransmitPower = limit_tx_pwr(tal_pib.TransmitPower);
                        reg_value = convert_phyTransmitPower_to_reg_value(tal_pib.TransmitPower);
                        pal_trx_bit_write(SR_TX_PWR, reg_value);
                    }
                    break;

                case phyCCAMode:
                    tal_pib.CCAMode = value->pib_value_8bit;
                    pal_trx_bit_write(SR_CCA_MODE, tal_pib.CCAMode);
                    break;

                case macIeeeAddress:
                    {
                        uint8_t *ptr;
                        tal_pib.IeeeAddress = value->pib_value_64bit;
                        ptr = (uint8_t *)&tal_pib.IeeeAddress;
                        for (uint8_t i = 0; i < 8; i++)
                        {
                            pal_trx_reg_write((RG_IEEE_ADDR_0 + i), *ptr);
                            ptr++;
                        }
                    }
                    break;

                case mac_i_pan_coordinator:
                    tal_pib.PrivatePanCoordinator = value->pib_value_bool;
                    pal_trx_bit_write(SR_AACK_I_AM_COORD, tal_pib.PrivatePanCoordinator);
                    break;

                case macAckWaitDuration:
                    /*
                     * AT86RF231 does not support changing this value w.r.t.
                     * compliance operation.
                     * The ACK timing can be reduced to 2 symbols using TFA function.
                     */
                    return MAC_UNSUPPORTED_ATTRIBUTE;

                default:
                    return MAC_UNSUPPORTED_ATTRIBUTE;
            }

            break; /* end of 'default' from 'switch (attribute)' */
    }
    return MAC_SUCCESS;
} /* tal_pib_set() */


/**
 * @brief Limit the phyTransmitPower to the trx limits
 *
 * @param phyTransmitPower phyTransmitPower value
 *
 * @return limited tal_pib_TransmitPower
 */
static uint8_t limit_tx_pwr(uint8_t tal_pib_TransmitPower)
{
    uint8_t ret_val = tal_pib_TransmitPower;
    int8_t dbm_value;

    dbm_value = CONV_phyTransmitPower_TO_DBM(tal_pib_TransmitPower);
    if (dbm_value > (int8_t)PGM_READ_BYTE(&tx_pwr_table[0]))
    {
        dbm_value = (int8_t)PGM_READ_BYTE(&tx_pwr_table[0]);
        ret_val = CONV_DBM_TO_phyTransmitPower(dbm_value);

    }
    else if (dbm_value < (int8_t)PGM_READ_BYTE(&tx_pwr_table[sizeof(tx_pwr_table) - 1]))
    {
        dbm_value = (int8_t)PGM_READ_BYTE(&tx_pwr_table[sizeof(tx_pwr_table) - 1]);
        ret_val = CONV_DBM_TO_phyTransmitPower(dbm_value);
    }

    return (ret_val | TX_PWR_TOLERANCE);
}


/**
 * @brief Converts a phyTransmitPower value to a register value
 *
 * @param phyTransmitPower_value phyTransmitPower value
 *
 * @return register value
 */
static uint8_t convert_phyTransmitPower_to_reg_value(uint8_t phyTransmitPower_value)
{
    int8_t dbm_value;
    int8_t i;
    int8_t trx_tx_level;

    dbm_value = CONV_phyTransmitPower_TO_DBM(phyTransmitPower_value);

    /* Compare to the register value to identify the value that matches. */
    for (i = sizeof(tx_pwr_table) - 1; i >= 0 ; i-- )
    {
        trx_tx_level = (int8_t)PGM_READ_BYTE(&tx_pwr_table[i]);
        if (trx_tx_level >= dbm_value)
        {
            if (trx_tx_level > dbm_value)
            {
                return (i + 1);
            }
            return i;
        }
    }

    /* This code should never be reached. */
    return 0;
}



#ifdef HIGH_DATA_RATE_SUPPORT
/**
 * @brief Apply channel page configuartion to transceiver
 *
 * @param ch_page Channel page
 *
 * @return true if changes could be applied else false
 */
static bool apply_channel_page_configuration(uint8_t ch_page)
{
    /*
     * Before updating the transceiver a number of TAL PIB attributes need
     * to be updated depending on the channel page.
     */
    tal_pib.MaxFrameDuration = MAX_FRAME_DURATION;
    tal_pib.SHRDuration = NO_OF_SYMBOLS_PREAMBLE_SFD;
    tal_pib.SymbolsPerOctet = SYMBOLS_PER_OCTET;

    switch (ch_page)
    {
        case 0: /* compliant O-QPSK */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_250KBPS);
            // Apply compliant ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_12_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 2: /* non-compliant OQPSK mode 1 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_500KBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 16:    /* non-compliant OQPSK mode 2 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_1MBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 17:    /* non-compliant OQPSK mode 3 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_2MBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use reduced sensitivity for 2Mbit mode
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x01);
            break;

        default:
            return false;
    }

    return true;
}
#endif

/* EOF */
