/**
 * @file mac_mcps_data.c
 *
 * @brief Handles MCPS related primitives and frames
 *
 * This file handles MCPS-DATA requests from the upper layer,
 * generates data frames and initiates its transmission, and
 * processes received data frames.
 *
 * $Id: mac_mcps_data.c 30017 2012-01-04 03:05:10Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* === Includes ============================================================ */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "pal.h"
#include "return_val.h"
#include "bmm.h"
#include "qmm.h"
#include "tal.h"
#include "ieee_const.h"
#include "mac_msg_const.h"
#include "mac_api.h"
#include "mac_msg_types.h"
#include "mac_data_structures.h"
#include "stack_config.h"
#include "mac_internal.h"
#include "mac.h"
#include "mac_build_config.h"
#ifdef MAC_SECURITY_ZIP
#include "mac_security.h"
#endif

#include "nwk.h"
#include "nwkStateMachine.h"
#include "nwk_api.h"
/* === Macros =============================================================== */


/* === Globals ============================================================= */


/* === Prototypes ========================================================== */
//extern bool wpan_nlde_data_req(NWK_DstAddrMode_t dstAddrMode,
//        ShortAddr_t dstAddr,
//        uint8_t nsduLength,
//        uint8_t *nsdu,
//        uint8_t *nsduHandle,
//        uint8_t radius,
//        uint8_t nonMemberRadius,
//        bool    discoverRoute,
//        uint8_t    secutityEnable,
//        app_route_method_t routeType);
 retval_t build_data_frame(mcps_data_req_t *pmdr,
                                 frame_info_t *frame, uint8_t fcfs);
#if (MAC_INDIRECT_DATA_FFD == 1)
static void handle_persistence_time_decrement(void);
static void handle_exp_persistence_timer(buffer_t *buf_ptr);
static void mac_t_persistence_cb(void *callback_parameter);
static uint8_t decrement_persistence_time(void *buf_ptr, void *handle);
static uint8_t check_persistence_time_zero(void *buf_ptr, void *handle);
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */

/* MAC-internal Buffer functions */
#if (MAC_PURGE_REQUEST_CONFIRM == 1)
static uint8_t check_msdu_handle_cb(void *buf, void *handle);
static bool mac_buffer_purge(uint8_t msdu_handle);
#endif /* (MAC_PURGE_REQUEST_CONFIRM == 1) */

/* === Implementation ====================================================== */

/*
 * @brief Initiates mcps data confirm message
 *
 * This function creates the mcps data confirm structure,
 * and appends it into internal event queue.
 *
 * @param buf Buffer for mcps data confirmation.
 * @param status Data transmission status.
 * @param handle MSDU handle.
 * @param timestamp Time in symbols at which the data were transmitted
 *        (only if timestampting is enabled).
 */
#ifdef ENABLE_TSTAMP
void mac_gen_mcps_data_conf(buffer_t *buf, uint8_t status, uint8_t handle, uint32_t timestamp)
#else
void mac_gen_mcps_data_conf(buffer_t *buf, uint8_t status, uint8_t handle)
#endif  /* ENABLE_TSTAMP */
{
    mcps_data_conf_t *mdc = (mcps_data_conf_t *)BMM_BUFFER_POINTER(buf);

    mdc->cmdcode = MCPS_DATA_CONFIRM;
    mdc->msduHandle = handle;
    mdc->status = status;
#ifdef ENABLE_TSTAMP
    mdc->Timestamp = timestamp;
#endif  /* ENABLE_TSTAMP */

    qmm_queue_append(&mac_nhle_q, buf);
}



/**
 * @brief Builds the data frame for transmission
 *
 * This function builds the data frame for transmission.
 * The NWK layer has supplied the parameters.
 * The frame_info_t data type is constructed and filled in.
 * Also the FCF is constructed based on the parameters passed.
 *
 * @param msg Pointer to the MCPS-DATA.request parameter
 */
void mcps_data_request(uint8_t *msg)
{
    retval_t status = FAILURE;
    mcps_data_req_t mdr;

    memcpy(&mdr, BMM_BUFFER_POINTER((buffer_t *)msg), sizeof(mcps_data_req_t));

    if ((mdr.TxOptions & WPAN_TXOPT_INDIRECT) == 0)
    {
        /*
         * Data Requests for a coordinator using direct transmission are
         * accepted in all non-transient states (no polling and no scanning
         * is ongoing).
         */
        if ((MAC_POLL_IDLE != mac_poll_state) ||
            (MAC_SCAN_IDLE != mac_scan_state)
           )
        {
            mac_gen_mcps_data_conf((buffer_t *)msg,
                                   (uint8_t)MAC_CHANNEL_ACCESS_FAILURE,
#ifdef ENABLE_TSTAMP
                                   mdr.msduHandle,
                                   0);
#else
                                   mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
            return;
        }
    }




#ifndef REDUCED_PARAM_CHECK
    /*To check the broadcst address*/
    uint16_t broadcast_check;
    ADDR_COPY_DST_SRC_16(broadcast_check, mdr.DstAddr);
    /* Check whether somebody requests an ACK of broadcast frames */
    if ((mdr.TxOptions & WPAN_TXOPT_ACK) &&
        (FCF_SHORT_ADDR == mdr.DstAddrMode) &&
         (BROADCAST == broadcast_check))
    {
        mac_gen_mcps_data_conf((buffer_t *)msg,
                           (uint8_t)MAC_INVALID_PARAMETER,
#ifdef ENABLE_TSTAMP
                           mdr.msduHandle,
                           0);
#else
                           mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
        return;
    }

    /* Check whether both Src and Dst Address are not present */
    if ((FCF_NO_ADDR == mdr.SrcAddrMode) &&
        (FCF_NO_ADDR == mdr.DstAddrMode))
    {
        mac_gen_mcps_data_conf((buffer_t *)msg,
                               (uint8_t)MAC_INVALID_ADDRESS,
#ifdef ENABLE_TSTAMP
                               mdr.msduHandle,
                               0);
#else
                               mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
        return;
    }

    /* Check whether Src or Dst Address indicate reserved values */
    if ((FCF_RESERVED_ADDR == mdr.SrcAddrMode) ||
        (FCF_RESERVED_ADDR == mdr.DstAddrMode))
    {
        mac_gen_mcps_data_conf((buffer_t *)msg,
                               (uint8_t)MAC_INVALID_PARAMETER,
#ifdef ENABLE_TSTAMP
                               mdr.msduHandle,
                               0);
#else
                               mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
        return;
    }
#endif  /* REDUCED_PARAM_CHECK */

    /* Now all is fine, continue... */
    frame_info_t *transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER((buffer_t *)msg);

    /* Store the message type */
    transmit_frame->msg_type = MCPS_MESSAGE;
    transmit_frame->msduHandle = mdr.msduHandle;

#if (MAC_INDIRECT_DATA_FFD == 1)
    /* Indirect transmission not ongoing yet. */
    transmit_frame->indirect_in_transit = false;
#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */

    status = build_data_frame(&mdr, transmit_frame, 2);

    if (MAC_SUCCESS != status)
    {
        /* The frame is too long. */
        mac_gen_mcps_data_conf((buffer_t *)msg,
                               (uint8_t)status,
#ifdef ENABLE_TSTAMP
                               mdr.msduHandle,
                               0);
#else
                               mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
        return;
    }

    /*
     * Broadcast transmission in a beacon-enabled network intiated by a
     * PAN Coordinator or Coordinator is put into the broadcast queue..
     */
#if (MAC_START_REQUEST_CONFIRM == 1)
#ifdef BEACON_SUPPORT
    /*To check the broadcst address*/
    uint16_t broadcast;
    ADDR_COPY_DST_SRC_16(broadcast, mdr.DstAddr);
    if (
        ((MAC_PAN_COORD_STARTED == mac_state) || (MAC_COORDINATOR == mac_state)) &&
        (tal_pib.BeaconOrder < NON_BEACON_NWK) &&
        (FCF_SHORT_ADDR == mdr.DstAddrMode) &&
        (BROADCAST == broadcast)
       )
    {
        /* Append the MCPS data request into the broadcast queue */
#ifdef ENABLE_QUEUE_CAPACITY
        if (QUEUE_FULL == qmm_queue_append(&broadcast_q, (buffer_t *)msg))
        {
            mac_gen_mcps_data_conf((buffer_t *)msg,
                                   (uint8_t)MAC_CHANNEL_ACCESS_FAILURE,
#ifdef ENABLE_TSTAMP
                                   mdr.msduHandle,
                                   0);
#else
                                   mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
            return;
        }
#endif   /* ENABLE_QUEUE_CAPACITY */
        qmm_queue_append(&broadcast_q, (buffer_t *)msg);
        return;
    }
#endif  /* BEACON_SUPPORT */
#endif /* (MAC_START_REQUEST_CONFIRM == 1) */

    /*
     * Indirect transmission is only allowed if we are
     * a PAN coordinator or coordinator.
     */
#if (MAC_INDIRECT_DATA_FFD == 1)
    if (
        (mdr.TxOptions & WPAN_TXOPT_INDIRECT) &&
        ((MAC_PAN_COORD_STARTED == mac_state) || (MAC_COORDINATOR == mac_state))
       )
    {
        /* Append the MCPS data request into the indirect data queue */
#ifdef ENABLE_QUEUE_CAPACITY
        if (QUEUE_FULL == qmm_queue_append(&indirect_data_q, (buffer_t *)msg))
        {
            mac_gen_mcps_data_conf((buffer_t *)msg,
                                   (uint8_t)MAC_TRANSACTION_OVERFLOW,
#ifdef ENABLE_TSTAMP
                                   mdr.msduHandle,
                                   0);
#else
                                   mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */
            return;
        }
#else
        qmm_queue_append(&indirect_data_q, (buffer_t *)msg);
#endif  /* ENABLE_QUEUE_CAPACITY */

        /*
         * If an FFD does have pending data,
         * the MAC persistence timer needs to be started.
         */
        transmit_frame->persistence_time = mac_pib.mac_TransactionPersistenceTime;
        mac_check_persistence_timer();
    }
    else
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */

    /*
     * We are NOT indirect, so we need to transmit using
     * CSMA_CA in the CAP (for beacon enabled) or immediately (for
     * a non-beacon enabled).
     */
    {
        mac_trx_wakeup();

        transmit_frame->buffer_header = (buffer_t *)msg;

        /* Transmission should be done with CSMA-CA and with frame retries. */
#ifdef BEACON_SUPPORT
        csma_mode_t cur_csma_mode;

        if (NON_BEACON_NWK == tal_pib.BeaconOrder)
        {
            /* In Nonbeacon network the frame is sent with unslotted CSMA-CA. */
            cur_csma_mode = CSMA_UNSLOTTED;
        }
        else
        {
            /* In Beacon network the frame is sent with slotted CSMA-CA. */
            cur_csma_mode = CSMA_SLOTTED;
        }

        status = tal_tx_frame(transmit_frame, cur_csma_mode, true);
#else   /* No BEACON_SUPPORT */
        /* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
        status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);
#endif  /* BEACON_SUPPORT / No BEACON_SUPPORT */

        if (MAC_SUCCESS == status)
        {
            MAKE_MAC_BUSY();
        }
        else
        {
            /* Transmission to TAL failed, generate confirmation message. */
            mac_gen_mcps_data_conf((buffer_t *)msg,
                                   (uint8_t)MAC_CHANNEL_ACCESS_FAILURE,
#ifdef ENABLE_TSTAMP
                                   mdr.msduHandle,
                                   0);
#else
                                   mdr.msduHandle);
#endif  /* ENABLE_TSTAMP */

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
        }
    }
} /* mcps_data_request() */



/**
 * @brief Processes data frames
 *
 * This function processes the data frames received and sends
 * mcps_data_indication to the NHLE.
 *
 * @param buf_ptr Pointer to receive buffer of the data frame
 */
void mac_process_data_frame(buffer_t *buf_ptr)
{
#ifndef _ENDDEVICE_  
    retval_t status = FAILURE;
#endif    
    mcps_data_ind_t *mdi =
        (mcps_data_ind_t *)BMM_BUFFER_POINTER(buf_ptr);

    if (mac_parse_data.mac_payload_length == 0)
    {
        /*
         * A null frame is neither indicated to the higher layer
         * nor checked for for frame pending bit set, since
         * null data frames with frame pending bit set are nonsense.
         */
        /* Since no indication is generated, the frame buffer is released. */
        bmm_buffer_free(buf_ptr);

        /* Set radio to sleep if allowed */
        mac_sleep_trans();
    }
    else
    {
        /* Build the MLME_Data_indication parameters. */
        mdi->DSN = mac_parse_data.sequence_number;
#ifdef ENABLE_TSTAMP
        mdi->Timestamp = mac_parse_data.time_stamp;
#endif /* ENABLE_TSTAMP */

        /* Source address info */
        mdi->SrcAddrMode = mac_parse_data.src_addr_mode;
        mdi->SrcPANId = mac_parse_data.src_panid;

        if (FCF_LONG_ADDR == mdi->SrcAddrMode ||
            FCF_SHORT_ADDR == mdi->SrcAddrMode)
        {
            mdi->SrcAddr = 0;
            if(FCF_SHORT_ADDR == mdi->SrcAddrMode)
            {
                ADDR_COPY_DST_SRC_16(mdi->SrcAddr, mac_parse_data.src_addr.short_address);
            }
            else if (FCF_LONG_ADDR == mdi->SrcAddrMode)
            {
                ADDR_COPY_DST_SRC_64(mdi->SrcAddr, mac_parse_data.src_addr.long_address);
            }
        }
        else
        {
            /*
             * Even if the Source address mode is zero, and the source address
             * informationis 韘 not present, the values are cleared to prevent
             * the providing of trash information.
             */
            mdi->SrcPANId = 0;
            mdi->SrcAddr = 0;
        }


        /* Start of duplicate detection. */
        if ((mdi->DSN == mac_last_dsn) &&
            (mdi->SrcAddr == mac_last_src_addr)
           )
        {
            /*
             * This is a duplicated frame.
             * It will not be indicated to the next higher layer,
             * but nevetheless the frame pending bit needs to be
             * checked and acted upon.
             */
            /* Since no indication is generated, the frame buffer is released. */
            bmm_buffer_free(buf_ptr);
        }
        else
        {
#ifndef _ENDDEVICE_
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
        	{
            mcps_data_req_t mdr;
            gNwkFrameHeader_p = (NwkFrameHeader_t *)mac_parse_data.mac_payload_data.data.payload;
            if (gNwkFrameHeader_p->dstAddr != gNwk_nib.networkAddress && gNwkFrameHeader_p->dstAddr < BROADCAST_ADDR_ROUTERS)
            {
                gNwkFrameHeader_p->radius--;
                if (gNwkFrameHeader_p->radius > 0)  
                {  
                    if (gNwkFrameHeader_p->frameControl.frameType == NWK_FRAMETYPE_COMMAND && gNwkFrameHeader_p->field.src.payload[0] == NWK_CMD_ROUTE_RECORD)
                        goto toUplayer;
                  //  NwkState = NWK_MODULE_ID_DATA_IND_FORWARD; 
                    uint16_t N;

#ifndef  REDUCE_CHECK_PARMER3
                    AppFrameHeader_t *AppFrameHeader;//enddevice 的父亲回应master一个ack


                    if (gNwkFrameHeader_p->frameControl.frameType == NWK_FRAMETYPE_DATA)
                    {
                        AppFrameHeader = (AppFrameHeader_t *)(gNwkFrameHeader_p->field.payload+(gNwkFrameHeader_p->frameControl.sourceRoute ? gNwkFrameHeader_p->field.payload[0]*2+2 : 0));
                        mac_parse_data.mac_payload_data.data.payload[mac_parse_data.mac_payload_length-1] ^= *((uint8_t *)AppFrameHeader +1);
                        AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(mdi->RSSI);       
                        mac_parse_data.mac_payload_data.data.payload[mac_parse_data.mac_payload_length-1] ^= *((uint8_t *)AppFrameHeader +1);;
                    }
#endif

					NwkRoutingTableEntry_t *tempRoutingTableEntry;
#ifdef END_DEVICE_SUPPORT
					NwkNeighbor_t *tempNwkNeighbor;
				    uint8_t src_addr_mode;
				    wpan_addr_spec_t dst_addr;



					tempNwkNeighbor = NWK_FindNeighborByShortAddr(gNwkFrameHeader_p->dstAddr);
					if (tempNwkNeighbor->deviceType == DEVICE_TYPE_END_DEVICE && tempNwkNeighbor != NULL && tempNwkNeighbor->relationship == RELATIONSHIP_CHILD && tempNwkNeighbor->rxOnWhenIdle == false)
					{
				        src_addr_mode = WPAN_ADDRMODE_SHORT;
				        dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
				        dst_addr.PANId = tal_pib.PANId;
				        dst_addr.Addr.short_address = gNwkFrameHeader_p->dstAddr;

				        if (!wpan_mcps_data_req(src_addr_mode,////enddevice 的父亲间接传输给孩子
				                                &dst_addr,
				                                mac_parse_data.mac_payload_length,  // One octet
				                                mac_parse_data.mac_payload_data.data.payload,
				                                mac_pib.mac_DSN++,
				                                WPAN_TXOPT_INDIRECT_ACK))
				        {
				            /*
				             * Data could not be queued into the indirect queue.
				             * Add error handling if required.
				             */
				        }

				        AppFrameHeader_t *AppFrameHeader = (AppFrameHeader_t *)gNwkFrameHeader_p->field.payload;

                        if (AppFrameHeader->frameControl.state == APP_REQUEST &&  AppFrameHeader->frameControl.ackReq == true)
                        {
							AppFrameHeader->frameControl.state = APP_RESPONSE;
							AppFrameHeader->frameControl.sendSuccess = true;
							AppFrameHeader->frameControl.ackReq = false;
							AppFrameHeader->frameControl.tries = 0;
#ifndef  REDUCE_CHECK_PARMER3
							AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(mdi->RSSI);
							pal_get_current_time(&currentTime);
							AppFrameHeader->travelTime = currentTime>>8;
							AppFrameHeader->hops = NWK_MAX_HOPS-gNwkFrameHeader_p->radius+1;
#endif
							if (AppFrameHeader->frameControl.frameType == APP_DATA)
							{
								AppFrameHeader->length = 0;
								AppFrameHeader->AppPayload[0] = XORSUM(gNwkFrameHeader_p->field.payload,7);
							}
							else
							{
								AppFrameHeader->length = 1;
								AppFrameHeader->AppPayload[1] = XORSUM(gNwkFrameHeader_p->field.payload,8);
							}

							//if (gResIndex > 207)
							//	gResIndex = 0;
							//memcpy(&gResponseData[gResIndex],gNwkFrameHeader_p->field.payload,APP_HEAD_LENGTH + AppFrameHeader->length);

							wpan_nlde_data_req(NWK_DSTADDRMODE_RESPONSE,
											  gNwkFrameHeader_p->srcAddr,
											  APP_HEAD_LENGTH + AppFrameHeader->length,
											  gNwkFrameHeader_p->field.payload,
											  gNwkFrameHeader_p->field.payload,
											  NWK_MAX_HOPS,
											  0,
											  false,
											  false,
											  AppFrameHeader->frameControl.routeType);

							//gResIndex += APP_HEAD_LENGTH + AppFrameHeader->length;

                        }

						bmm_buffer_free(buf_ptr);
						mac_sleep_trans();

						return;
					}
#endif
					if (gNwkFrameHeader_p->frameControl.sourceRoute == true)
					{
					    NwkSourceRouteSubframe_t *NwkSourceRouteSubframe = (NwkSourceRouteSubframe_t *)gNwkFrameHeader_p->field.payload;
					    if (NwkSourceRouteSubframe->relayIndex == 0)
					    	N = gNwkFrameHeader_p->dstAddr;
					    else
					    	N = NwkSourceRouteSubframe->relayList[NwkSourceRouteSubframe->relayIndex-1];
						NwkSourceRouteSubframe->relayIndex -= 1;
					}
#ifdef noBufferData_andUseTreeRoute
					else if (gNwkFrameHeader_p->frameControl.discoverRoute == true)
#else
					else
#endif
					{
						tempRoutingTableEntry = nwkFindRoutingEntry(gNwkFrameHeader_p->dstAddr, false);

						if (tempRoutingTableEntry != NULL && tempRoutingTableEntry->status == ACTIVE)
						{
							N = tempRoutingTableEntry->nextHopAddr;
						}
						else if (App_System_Para_Pointer->AtCfgInfo.isJoin == true)
						{
							N = Find_NextHopAddress_InTree(gNwkFrameHeader_p->dstAddr);
#ifdef noBufferData_andUseTreeRoute
							gNwkFrameHeader_p->frameControl.discoverRoute = false;
#endif
						}
						else
						{
	                        bmm_buffer_free(buf_ptr);
	                        return;
						}
					}
#ifdef noBufferData_andUseTreeRoute
					else	N = Find_NextHopAddress_InTree(gNwkFrameHeader_p->dstAddr);
#endif
                    mdr.SrcAddrMode = WPAN_ADDRMODE_SHORT;
                    mdr.DstAddrMode = WPAN_ADDRMODE_SHORT;
                    mdr.DstPANId    = gNwk_nib.panId;
                    mdr.DstAddr     = N;
                    mdr.msduHandle  = 0xFF;
                    mdr.TxOptions   = WPAN_TXOPT_ACK;
                    mdr.msduLength  = mac_parse_data.mac_payload_length;
                    mdr.msdu        = mac_parse_data.mac_payload_data.data.payload;
                
                    /* Now all is fine, continue... */
                    frame_info_t *transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER(buf_ptr);
                
                    /* Store the message type */
                    transmit_frame->msg_type = NWK_DATA_IND_FORWARD_N;
                    transmit_frame->msduHandle = mdr.msduHandle;
                    transmit_frame->prevHopAddr = mdi->SrcAddr;
                    transmit_frame->NwkFrameHeader = gNwkFrameHeader_p;
                    transmit_frame->AppFrameHeader = (AppFrameHeader_t *)(gNwkFrameHeader_p->field.payload+(gNwkFrameHeader_p->frameControl.sourceRoute ? gNwkFrameHeader_p->field.payload[0]*2+2 : 0));
                #if (MAC_INDIRECT_DATA_FFD == 1)
                    /* Indirect transmission not ongoing yet. */
                    transmit_frame->indirect_in_transit = false;
                #endif  /* (MAC_INDIRECT_DATA_FFD == 1) */
                
                    status = build_data_frame(&mdr, transmit_frame, 4);
                
                    if (MAC_SUCCESS != status)
                    {
                        bmm_buffer_free(buf_ptr);
                        return;
                    }

					mac_trx_wakeup();
#ifdef BEACON_SUPPORT         
                /**beacon 打开时的转发处理**/    
                uint8_t src_addr_mode;
                wpan_addr_spec_t dst_addr;
                
                src_addr_mode = WPAN_ADDRMODE_SHORT;
	        dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
	        dst_addr.PANId = tal_pib.PANId;
	        dst_addr.Addr.short_address = N;
                
                if (N == gNwk_nib.parentNetworkAddress)
                  wpan_mcps_data_req( src_addr_mode,////在enable——beacon如果是目的地址是父亲就缓存在to——parent——q队列里
                    &dst_addr,
                    mac_parse_data.mac_payload_length,  // One octet
                    mac_parse_data.mac_payload_data.data.payload,
                    mac_pib.mac_DSN,
                    WPAN_TXOPT_ACK);
                else
                  wpan_mcps_data_req( src_addr_mode,////在enable——beacon如果是目的地址是孩子就缓存在indirect—q队列里
                                      &dst_addr,
                                      mac_parse_data.mac_payload_length,  // One octet
                                      mac_parse_data.mac_payload_data.data.payload,
                                      mac_pib.mac_DSN,
                                      WPAN_TXOPT_INDIRECT_ACK);
                
                bmm_buffer_free(buf_ptr);
                goto label_pend;//当时转发帧时，也要判断是不是还有数据pending在父亲的beacon里

#else                

					transmit_frame->buffer_header = buf_ptr;


					/* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
					status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

					if (MAC_SUCCESS == status)
					{
					  MAKE_MAC_BUSY();
					}
					else
					{
						bmm_buffer_free(buf_ptr);
						mac_sleep_trans();
					}
                                        return;
#endif
                   
                } 
                else
                {
                    bmm_buffer_free(buf_ptr);
                    mac_sleep_trans();             
                    return;
                }
            }
            else if (gNwkFrameHeader_p->dstAddr >= BROADCAST_ADDR_ROUTERS && gNwkFrameHeader_p->frameControl.frameType == NWK_FRAMETYPE_DATA)
            {
#ifdef _NWK_PASSIVE_ACK_
            	uint8_t index = nwkFindPassiveAck(gNwkFrameHeader_p->srcAddr, gNwkFrameHeader_p->sequenceNumber);
            	if (index == 0xFF)
            	{
								if (gNwkFrameHeader_p->radius > 1)  
								{
									wpan_nlde_data_req(NWK_DSTADDRMODE_NOADDR,
														gNwkFrameHeader_p->srcAddr,//把源网络地址重新填回发送中
														mac_parse_data.mac_payload_length-8,
														gNwkFrameHeader_p->field.payload,
														gNwkFrameHeader_p->field.payload,
														gNwkFrameHeader_p->radius-1,
														gNwkFrameHeader_p->sequenceNumber,
														false,
														false,
														APP_ROUTE_BROADCAST);
								}
            	}
            	else
				{
#ifndef VANET
            		NwkNeighbor_t *tempNeighbor;
            		tempNeighbor = NWK_FindNeighborByShortAddr(mac_parse_data.src_addr.short_address);
            		if (tempNeighbor != NULL && tempNeighbor->isBroadRelay[index] == false)
            		{
						gNwkPassiveAckTable.table[index].realyNeighborNum++;
						tempNeighbor->isBroadRelay[index] = true;
            		}
#endif


#ifdef BROAD_BASE_GPS_OR_RSSI
        			index = nwkFindBroadDelay(gNwkFrameHeader_p->srcAddr, gNwkFrameHeader_p->sequenceNumber);
        			if (index != 0xFF)
        			{
						broadDelayBitmap &= ~(1<<index);
						nwkBroadDelay[index].delayCount = 0xFFFFFFFF;
						broadDelayCount--;
	                    nwkBroadDelay[index].transmit_frame = NULL;

	        			if (broadDelayCount == 0)
	        			{
	        				pal_timer_stop(APP_TIMER_UART_TIMEOUT);
	        			}
	        			else
	        			{
	        				if (bubble_sort_broad(index) == true)
	        				{
                                pal_timer_stop(APP_TIMER_UART_TIMEOUT);
	        					do
	        					{
	        						status =
	        							pal_timer_start(APP_TIMER_UART_TIMEOUT,
	        											nwkBroadDelay[minBroadDelayIndex].delayCount,
	        											TIMEOUT_ABSOLUTE,
	        											(FUNC_PTR)bc_delay_cb,
	        											NULL);
	        						if (MAC_SUCCESS != status)
	        						{
	        							broadDelayBitmap &= ~(1<<minBroadDelayIndex);
	        							nwkBroadDelay[minBroadDelayIndex].transmit_frame = NULL;
	        							nwkBroadDelay[minBroadDelayIndex].delayCount = 0xFFFFFFFF;
	        							broadDelayCount--;
	        	                        minBroadDelayIndex++;
	        						}

	        					} while (MAC_SUCCESS != status && nwkBroadDelay[minBroadDelayIndex].transmit_frame != NULL);
                                if (broadDelayCount == 0)
                                  minBroadDelayIndex = 0;
	        	//				if (MAC_SUCCESS != timer_status)//TODO 会有问题，后续的时钟也开起不了
	        	//				{
	        	//					NwkState = NWK_MODULE_NONE;
	        	//					//bmm_buffer_free(transmit_frame->buffer_header);
	        	//					mac_sleep_trans();
	        	//					return;
	        	//				}
	        				}
	        			}
        			}

#endif


            		bmm_buffer_free(buf_ptr);
                    mac_sleep_trans();
                    return;
            	}
#endif

            }
        }
#endif          
#ifndef _ENDDEVICE_
toUplayer:
#endif
            /* Generate data indication to next higher layer. */

            /* Store required information for perform subsequent
             * duplicate detections.
             */
            mac_last_dsn = mdi->DSN;
            mac_last_src_addr = mdi->SrcAddr;

            /* Destination address info */
            mdi->DstAddrMode = mac_parse_data.dest_addr_mode;
            /*
             * Setting the address to zero is required for a short address
             * and in case no address is included. Therefore the address
             * is first always set to zero to reduce code size.
             */
            mdi->DstAddr = 0;
            /*
             * Setting the PAN-Id to the Destiantion PAN-Id is required
             * for a both short and long address, but not in case no address
             * is included. Therefore the PAN-ID is first always set to
             * the Destination PAN-IDto reduce code size.
             */
            mdi->DstPANId = mac_parse_data.dest_panid;
            if (FCF_LONG_ADDR == mdi->DstAddrMode)
            {
                ADDR_COPY_DST_SRC_64(mdi->DstAddr, mac_parse_data.dest_addr.long_address);
            }
            else if (FCF_SHORT_ADDR == mdi->DstAddrMode)
            {
                ADDR_COPY_DST_SRC_16(mdi->DstAddr, mac_parse_data.dest_addr.short_address);
            }
            else
            {
                /*
                 * Even if the Destination address mode is zero, and the destination
                 * address information is 韘 not present, the values are cleared to
                 * prevent the providing of trash information.
                 * The Desintation address was already cleared above.
                 */
                mdi->DstPANId = 0;
            }

            mdi->mpduLinkQuality = mac_parse_data.ppdu_link_quality;
            mdi->RSSI          = mac_parse_data.ppdu_RSSI;

#ifdef MAC_SECURITY_ZIP
            mdi->SecurityLevel = mac_parse_data.sec_ctrl.sec_level;
            mdi->KeyIdMode = mac_parse_data.sec_ctrl.key_id_mode;
            mdi->KeyIndex = mac_parse_data.key_id[0];
#endif  /* MAC_SECURITY_ZIP */

            mdi->msduLength = mac_parse_data.mac_payload_length;

            /* Set pointer to data frame payload. */
            mdi->msdu = mac_parse_data.mac_payload_data.data.payload;

            mdi->cmdcode = MCPS_DATA_INDICATION;

            /* Append MCPS data indication to MAC-NHLE queue */
            qmm_queue_append(&mac_nhle_q, buf_ptr);

        }   /* End of duplicate detection. */


        /* Continue with checking the frame pending bit in the received
         * data frame.
         */

#if (MAC_INDIRECT_DATA_BASIC == 1)
label_pend:        
        if (mac_parse_data.fcf & FCF_FRAME_PENDING)
        {
#if (MAC_START_REQUEST_CONFIRM == 1)
            /* An node that is not PAN coordinator may poll for pending data. */
            if (MAC_PAN_COORD_STARTED != mac_state)
#endif  /* (MAC_START_REQUEST_CONFIRM == 1) */
            {
                 address_field_t src_addr;

                /* Build command frame due to implicit poll request */
                /*
                 * No explicit destination address attached, so use current
                 * values of PIB attributes macCoordShortAddress or
                 * macCoordExtendedAddress.
                 */
                /*
                 * This implicit poll (i.e. corresponding data request
                 * frame) is to be sent to the same node that we have received
                 * this data frame. Therefore the source address information
                 * from this data frame needs to be extracted, and used for the
                 * data request frame appropriately.
                 * Use this as destination address expclitily and
                 * feed this to the function mac_build_and_tx_data_req
                 */
                if (FCF_SHORT_ADDR == mac_parse_data.src_addr_mode)
                {

                ADDR_COPY_DST_SRC_16(src_addr.short_address, mac_parse_data.src_addr.short_address);

                mac_build_and_tx_data_req(false,
                                              false,
                                              FCF_SHORT_ADDR,
                                              (address_field_t *)&(src_addr),
                                              mac_parse_data.src_panid);
                }
                else if (FCF_LONG_ADDR == mac_parse_data.src_addr_mode)
                {

                    ADDR_COPY_DST_SRC_64(src_addr.long_address, mac_parse_data.src_addr.long_address);

                    mac_build_and_tx_data_req(false,
                                              false,
                                              FCF_LONG_ADDR,
                                              (address_field_t *)&(src_addr),
                                              mac_parse_data.src_panid);
                }
                else
                {
                    mac_build_and_tx_data_req(false, false, 0, NULL, 0);
                }
            }
        }
        else
#endif /* (MAC_INDIRECT_DATA_BASIC == 1) */
        {
            /* Frame pending but was not set, so no further action required. */
            /* Set radio to sleep if allowed */
            mac_sleep_trans();
        }   /* if (mac_parse_data.fcf & FCF_FRAME_PENDING) */
    }   /* (mac_parse_data.payload_length == 0) */
} /* mac_process_data_frame() */



/*
 * @brief Builds MCPS data frame
 *
 * This function builds the data frame.
 *
 * @param pmdr Request parameters
 * @param buffer Pointer to transmission frame
 * @param indirect Transmission is direct or indirect
 *
 * @return Status of the attempt to build the data frame
 */
 retval_t build_data_frame(mcps_data_req_t *pmdr,
                                 frame_info_t *frame,
                                 uint8_t fcfs)
{
    uint8_t frame_len;
    uint8_t *frame_ptr;
    uint16_t fcf = 0;

    frame_len = pmdr->msduLength +
                FCS_LENGTH + // Add 2 octets for FCS
                3;  // 3 octets DSN and FCF

    /*
     * Payload pointer points to data, which was already been copied
     * into buffer
     */
    if (fcfs == 2)
    frame_ptr = (uint8_t *)frame +
                LARGE_BUFFER_SIZE -
                  pmdr->msduLength - fcfs; /* Add 2 octets for FCS. */ //TODO pmdr->msduLength
    else/**转发时候由于接收到的帧是减去127存储的数据，所以这里也要减去相应的字节才能找到合适的数据位置**/
          frame_ptr = (uint8_t *)frame +
                LARGE_BUFFER_SIZE -
                  116 - fcfs; /* Add 2 octets for FCS. */ //TODO pmdr->msduLength

#ifdef MAC_SECURITY_ZIP
    uint8_t *mac_payload_ptr = frame_ptr;
    /*
     * Note: The value of the payload_length parameter will be updated
     *       if security needs to be applied.
     */
    if (pmdr->SecurityLevel > 0)
    {
        retval_t build_sec = mac_build_aux_sec_header(&frame_ptr, pmdr, &frame_len);
        if (MAC_SUCCESS != build_sec)
    {
            return (build_sec);
    }
    }
#endif  /* MAC_SECURITY_ZIP */

    /*
     * Set Source Address.
     */
    if (FCF_SHORT_ADDR == pmdr->SrcAddrMode)
    {
        frame_ptr -= 2;
        frame_len += 2;
        convert_16_bit_to_byte_array(tal_pib.ShortAddress, frame_ptr);
    }
    else if (FCF_LONG_ADDR == pmdr->SrcAddrMode)
    {
        frame_ptr -= 8;
        frame_len += 8;
        convert_64_bit_to_byte_array(tal_pib.IeeeAddress, frame_ptr);
    }

    /* Shall the Intra-PAN bit set? */
    if ((tal_pib.PANId == pmdr->DstPANId) &&
        (FCF_NO_ADDR != pmdr->SrcAddrMode) &&
        (FCF_NO_ADDR != pmdr->DstAddrMode))
    {
        /*
         * Both address are present and both PAN-Ids are identical.
         * Set intra-PAN bit.
         */
        fcf |= FCF_PAN_ID_COMPRESSION;
    }
    else if (FCF_NO_ADDR != pmdr->SrcAddrMode)
    {
        /* Set Source PAN-Id. */
        frame_ptr -= 2;
        frame_len += 2;
        convert_16_bit_to_byte_array(tal_pib.PANId, frame_ptr);
    }

    /* Set the Destination Addressing fields. */
    if (FCF_NO_ADDR != pmdr->DstAddrMode)
    {
        if (FCF_SHORT_ADDR == pmdr->DstAddrMode)
        {
            frame_ptr -= 2;
            frame_len += 2;
            convert_16_bit_to_byte_address(pmdr->DstAddr, frame_ptr);
        }
        else
        {
            frame_ptr -= 8;
            frame_len += 8;
            convert_64_bit_to_byte_array(pmdr->DstAddr, frame_ptr);
        }

        frame_ptr -= 2;
        frame_len += 2;
        convert_16_bit_to_byte_array(pmdr->DstPANId, frame_ptr);
    }

    /* Set DSN. */
    frame_ptr--;
    *frame_ptr = mac_pib.mac_DSN++;

    /* Set the FCF. */
#ifdef TEST_HARNESS
        /*
     * When performing tests this PIB attribute defaults to 1
     * (i.e. a standard data frame). If not set to 1, it is used as a
     * (supposedly illegal) frame type to fill into the frame type
     * field of the data frame's FCF. In effect, valid (illegal)
     * values range from 4 through 7.
         */
    if (mac_pib.privateIllegalFrameType != 1)
    {
        fcf |= FCF_SET_FRAMETYPE(mac_pib.privateIllegalFrameType);
    }
    else
#endif /* TEST_HARNESS */
    {
        fcf |= FCF_SET_FRAMETYPE(FCF_FRAMETYPE_DATA);
    }

#ifdef MAC_SECURITY_ZIP
    if (pmdr->SecurityLevel > 0)
    {
        fcf |= FCF_SECURITY_ENABLED | FCF_FRAME_VERSION_2006;
    }
#endif

    if (pmdr->TxOptions & WPAN_TXOPT_ACK)
    {
        fcf |= FCF_ACK_REQUEST;
    }

    /*
     * 802.15.4-2006 section 7.1.1.1.3:
     *
     * If the msduLength parameter is greater than aMaxMACSafePayloadSize,
     * the MAC sublayer will set the Frame Version subfield of the
     * Frame Control field to one.
     */
    if (pmdr->msduLength > aMaxMACSafePayloadSize)
    {
        fcf |= FCF_FRAME_VERSION_2006;
    }

    /* Set FCFs address mode */
    fcf |= FCF_SET_SOURCE_ADDR_MODE(pmdr->SrcAddrMode);
    fcf |= FCF_SET_DEST_ADDR_MODE(pmdr->DstAddrMode);

    frame_ptr -= 2;
    convert_spec_16_bit_to_byte_array(fcf, frame_ptr);

    /*
     * In case the frame gets too large, return error.
     */
    if (frame_len > aMaxPHYPacketSize)
    {
       return MAC_FRAME_TOO_LONG;
    }

    /* First element shall be length of PHY frame. */
    frame_ptr--;
    *frame_ptr = frame_len;

    /* Finished building of frame. */
    frame->mpdu = frame_ptr;

#ifdef MAC_SECURITY_ZIP
    if (pmdr->SecurityLevel > 0)
    {
        retval_t build_sec = mac_secure(frame, mac_payload_ptr, pmdr);
        if (MAC_SUCCESS != build_sec)
        {
            return (build_sec);
        }
    }
#endif  /* MAC_SECURITY_ZIP */

    return MAC_SUCCESS;
} /* build_data_frame() */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Start the Persistence timer for indirect data
 *
 * This function starts the persistence timer for handling of indirect
 * data.
 */
void mac_start_persistence_timer(void)
{
    retval_t status = FAILURE;
    /* Interval of indirect data persistence timer */
    uint32_t persistence_int_us;

#ifdef BEACON_SUPPORT
    /*
     * This is a beacon build.
     */
    uint8_t bo_for_persistence_tmr;

    if (tal_pib.BeaconOrder == NON_BEACON_NWK)
    {
        /*
         * The timeout interval for the indirect data persistence timer is
         * based on the define below and is the same as for a nonbeacon build.
         */
        bo_for_persistence_tmr = BO_USED_FOR_MAC_PERS_TIME;
    }
    else
    {
        /*
         * The timeout interval for the indirect data persistence timer is
         * based on the current beacon order.
         */
        bo_for_persistence_tmr = tal_pib.BeaconOrder;
    }

    persistence_int_us =
        TAL_CONVERT_SYMBOLS_TO_US(TAL_GET_BEACON_INTERVAL_TIME(bo_for_persistence_tmr));
#else   /* No BEACON_SUPPORT */
    /*
     * This is a nonbeacon build. The timeout interval for the indirect data
     * persistence timer is based on the define below.
     */

    persistence_int_us =
        TAL_CONVERT_SYMBOLS_TO_US(TAL_GET_BEACON_INTERVAL_TIME(BO_USED_FOR_MAC_PERS_TIME));

#endif  /* BEACON_SUPPORT / No BEACON_SUPPORT */

    /* Start the indirect data persistence timer now. */
    status = pal_timer_start(T_Data_Persistence,
                             persistence_int_us,
                             TIMEOUT_RELATIVE,
                             (FUNC_PTR)mac_t_persistence_cb,
                             NULL);

    if (MAC_SUCCESS != status)
    {
        /* Got to the persistence timer callback function immediately. */
        mac_t_persistence_cb(NULL);
#if (DEBUG > 0)
        ASSERT("Indirect data persistence timer start failed" == 0);
#endif
    }
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Handles timeout of indirect data persistence timer
 *
 * This function is a callback function of the timer started for checking
 * the mac persistence time of indirect data in the queue.
 *
 * @param callback_parameter Callback parameter
 */
static void mac_t_persistence_cb(void *callback_parameter)
{
    /* Decrement the persistence time for indirect data. */
    handle_persistence_time_decrement();

    if (indirect_data_q.size > 0)
    {
        /* Restart persistence timer. */
        mac_start_persistence_timer();
    }

    callback_parameter = callback_parameter; /* Keep compiler happy. */
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Handles the decrement of the persistence time
 *
 * Handles the decrement of the persistance time of each indirect data frame
 * in the indirect queue.
 * If the persistance time of any indirect data reduces to zero, the
 * frame is removed from the indirect queue and
 * a confirmation for that indirect data is sent with the status
 * transaction expired.
 */
static void handle_persistence_time_decrement(void)
{
    search_t find_buf;

    /*
     * This callback function traverses through the indirect queue and
     * decrements the persistence time for each data frame.
     */
    find_buf.criteria_func = decrement_persistence_time;

    /*
     * At the end of this function call (qmm_queue_read), the indirect data
     * will be updated with the decremented persistence time.
     */
    qmm_queue_read(&indirect_data_q, &find_buf);

    /*
     * Once we have updated the persistence timer, any frame with a persistence
     * time of zero needs to be removed from the indirect queue.
     */
    buffer_t *buffer_persistent_zero = NULL;

    /*
     * This callback function traverses through the indirect queue and
     * searches for a data frame with persistence time equal to zero.
     */
    find_buf.criteria_func = check_persistence_time_zero;

    do
    {
        buffer_persistent_zero = qmm_queue_remove(&indirect_data_q, &find_buf);

        if (NULL != buffer_persistent_zero)
        {
            handle_exp_persistence_timer(buffer_persistent_zero);
        }
    }
    while (NULL != buffer_persistent_zero);
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Decrements the persistence time of an indirect data frame
 *
 * @param buf_ptr Pointer to the indirect data in the indirect queue
 * @param handle Callback parameter
 *
 * @return 0 to traverse through the full indirect queue
 *
 */
static uint8_t decrement_persistence_time(void *buf_ptr, void *handle)
{
    frame_info_t *frame = (frame_info_t *)buf_ptr;

    /*
     * In case the frame is currently in the process of being transmitted,
     * the persistence time is not decremented, to avoid the expiration of
     * the persistence timer during transmission.
     * Once the transmission is done (and was not successful),
     * the frame will still be in the indirect queue and the persistence
     * timer will be decremented again.
     */
    if (!frame->indirect_in_transit)
    {
        /* Decrement the persistence time for this indirect data frame. */
        frame->persistence_time--;
    }

    handle = handle;    /* Keep compiler happy. */

    return 0;
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Checks for indirect data with persistence time zero
 *
 * This callback function checks whether the persistence time
 * of the indirect data is set to zero.
 *
 * @param buf Pointer to indirect data buffer
 *
 * @return 1 if extended address passed matches with the destination
 * address of the indirect frame , 0 otherwise
 */
static uint8_t check_persistence_time_zero(void *buf_ptr, void *handle)
{
    frame_info_t *frame = (frame_info_t *)buf_ptr;

    /* Frame shall not be in transmission. */
    if (!frame->indirect_in_transit)
    {
        if (frame->persistence_time == 0)
        {
            return 1;
        }
    }

    handle = handle;    /* Keep compiler happy. */

    return 0;
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/*
 * @brief Generates notification for expired transaction
 *
 * This function generates the confirmation for those indirect data buffers
 * whose persistence time has reduced to zero.
 *
 * @param buf_ptr Pointer to buffer of indirect data whose persistance time
 * has reduced to zero
 */
static void handle_exp_persistence_timer(buffer_t *buf_ptr)
{
    frame_info_t *trans_frame = (frame_info_t *)BMM_BUFFER_POINTER(buf_ptr);

    /*
     * The frame should never be in transmission while this function
     * is called.
     */
    ASSERT(trans_frame->indirect_in_transit == false);

    switch (trans_frame->msg_type)
    {
#if (MAC_ASSOCIATION_INDICATION_RESPONSE == 1)
        case ASSOCIATIONRESPONSE:
            {
             // gMlmeCommStatus_FrameType = ASSOCIATIONRESPONSE;  
              mac_mlme_comm_status(MAC_TRANSACTION_EXPIRED,
                                     buf_ptr);
            }
            break;
#endif  /* (MAC_ASSOCIATION_INDICATION_RESPONSE == 1) */

#if (MAC_DISASSOCIATION_BASIC_SUPPORT == 1)
        case DISASSOCIATIONNOTIFICATION:
            /*
             * Prepare disassociation confirm message after transmission of
             * the disassociation notification frame.
             */
            mac_prep_disassoc_conf((buffer_t *)buf_ptr,
                                   MAC_TRANSACTION_EXPIRED);
            break;
#endif  /* (MAC_DISASSOCIATION_BASIC_SUPPORT == 1) */

        case MCPS_MESSAGE:
            {
                mac_gen_mcps_data_conf((buffer_t *)buf_ptr,
                                       (uint8_t)MAC_TRANSACTION_EXPIRED,
#ifdef ENABLE_TSTAMP
                                        trans_frame->msduHandle,
                                        0);
#else
                                        trans_frame->msduHandle);
#endif  /* ENABLE_TSTAMP */
            }
            break;

        default:
            ASSERT("Unknown buf_ptr type" == 0);
            /* Nothing to be done here. */
            break;
    }
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if ((MAC_PURGE_REQUEST_CONFIRM == 1) && (MAC_INDIRECT_DATA_FFD == 1))
/*
 * @brief Purges a buffer corresponding to a MSDU handle
 *
 * This function tries to purge a given msdu by finding its msdu handle.
 * If the handle is found, that buffer is freed up for further use.
 * This routine will typically be called from the mlme_purge_request routine.
 *
 * @param msdu_handle MSDU handle
 *
 * @return True if the MSDU handle is found in the indirect queue
 *         and removed successfully, false otherwise.
 */
static bool mac_buffer_purge(uint8_t msdu_handle)
{
    uint8_t *buf_ptr;
    search_t find_buf;
    uint8_t handle = msdu_handle;

    /*
     * Callback function  for searching the data having MSDU handle
     * given by purge request
     */
    find_buf.criteria_func = check_msdu_handle_cb;

    /* Update the MSDU handle to be searched */
    find_buf.handle = &handle;

    /* Remove from indirect queue if the short address matches */
    buf_ptr = (uint8_t *)qmm_queue_remove(&indirect_data_q, &find_buf);

    if (NULL != buf_ptr)
    {
        /* Free the buffer allocated, after purging */
        bmm_buffer_free((buffer_t *)buf_ptr);

        return true;
    }

    /* No data available in the indirect queue with the MSDU handle. */
    return false;
}



/**
 * @brief Processes a MCPS-PURGE.request primitive
 *
 * This functions processes a MCPS-PURGE.request from the NHLE.
 * The MCPS-PURGE.request primitive allows the next higher layer
 * to purge an MSDU from the transaction queue.
 * On receipt of the MCPS-PURGE.request primitive, the MAC sublayer
 * attempts to find in its transaction queue the MSDU indicated by the
 * msduHandle parameter. If an MSDU matching the given handle is found,
 * the MSDU is discarded from the transaction queue, and the MAC
 * sublayer issues the MCPSPURGE. confirm primitive with a status of
 * MAC_SUCCESS. If an MSDU matching the given handle is not found, the MAC
 * sublayer issues the MCPS-PURGE.confirm primitive with a status of
 * INVALID_HANDLE.
 *
 * @param msg Pointer to the MCPS-PURGE.request parameter
 */
void mcps_purge_request(uint8_t *msg)
{
    mcps_purge_req_t *mpr =
        (mcps_purge_req_t *)BMM_BUFFER_POINTER(((buffer_t *)msg));

    mcps_purge_conf_t *mpc = (mcps_purge_conf_t *)mpr;

    uint8_t purge_handle = mpr->msduHandle;

    /* Update the purge confirm structure */
    mpc->cmdcode = MCPS_PURGE_CONFIRM;
    mpc->msduHandle = purge_handle;

    if (mac_buffer_purge(purge_handle))
    {
        mpc->status = MAC_SUCCESS;
    }
    else
    {
        mpc->status = MAC_INVALID_HANDLE;
    }

    qmm_queue_append(&mac_nhle_q, (buffer_t *)msg);
}



/*
 * @brief Checks whether MSDU handle matches
 *
 * @param buf Pointer to indirect data buffer
 * @param handle MSDU handle to be searched
 *
 * @return 1 if MSDU handle matches with the indirect data, 0 otherwise
 */
static uint8_t check_msdu_handle_cb(void *buf, void *handle)
{
    frame_info_t *frame = (frame_info_t *)buf;
    uint8_t msdu;

    msdu = *((uint8_t *)handle);

    /* Compare the MSDU handle */
    if (frame->msduHandle == msdu)
    {
        return 1;
    }
    return 0;
}
#endif /* ((MAC_PURGE_REQUEST_CONFIRM == 1) && (MAC_INDIRECT_DATA_FFD == 1)) */
/* EOF */
