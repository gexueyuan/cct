/**
 * @file mac_process_tal_tx_frame_status.c
 *
 * @brief Processes the TAL tx frame status received on the frame transmission.
 *
 * $Id: mac_process_tal_tx_frame_status.c 30017 2012-01-04 03:05:10Z yogesh.bellan $
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
#include "mac_config.h"
#include "mac_build_config.h"


#include "nwk.h"
#include "nwk_api.h"
#include "nwkStateMachine.h"
#include "app_uart.h"
#include "main1.h"
/* === Macros =============================================================== */


/* === Globals ============================================================= */
uint32_t ledTxCounter=0,ledRxCounter=0;
//int ooo=0;
/* === Prototypes ========================================================== */

static void mac_process_tal_tx_status(retval_t tx_status,  frame_info_t *frame);

#if (MAC_INDIRECT_DATA_FFD == 1)
static uint8_t find_buffer_cb(void *buf, void *address);
static void remove_frame_from_indirect_q(frame_info_t *f_ptr);
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */

/* === Implementation ====================================================== */

/*
 * @brief Process tal_tx_frame_done_cb status
 *
 * This function is called, if an ACK is requested in the last transmitted frame.
 * According to the frame type that has previously been sent, the
 * corresponding actions are taken and the MAC returns to its standard state.
 *
 * @param tx_status Status of transmission
 * @param frame Pointer to the transmitted frame
 */
static void mac_process_tal_tx_status(retval_t tx_status,  frame_info_t *frame)
{
    switch (frame->msg_type)
    {
        case MCPS_MESSAGE:
#if (MAC_INDIRECT_DATA_FFD == 1)
            if (frame->indirect_in_transit)
            {
                // Clear indirect in transmit flag
                frame->indirect_in_transit = false;
                if ((tx_status == MAC_SUCCESS) || (tx_status == TAL_FRAME_PENDING))
                {
                    remove_frame_from_indirect_q(frame);

                    buffer_t *mcps_buf = frame->buffer_header;

                    /* Create the MCPS DATA confirmation message */
                    mac_gen_mcps_data_conf((buffer_t *)mcps_buf,
                                           (uint8_t)tx_status,
#ifdef ENABLE_TSTAMP
                                           frame->msduHandle,
                                           frame->time_stamp);
#else
                                           frame->msduHandle);
#endif  /* ENABLE_TSTAMP */
                }
            }
            else    // Not indirect
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */
            {
                buffer_t *mcps_buf = frame->buffer_header;

                /* Create the MCPS DATA confirmation message */
                mac_gen_mcps_data_conf((buffer_t *)mcps_buf,
                                       (uint8_t)tx_status,
#ifdef ENABLE_TSTAMP
                                       frame->msduHandle,
                                       frame->time_stamp);
#else
                                       frame->msduHandle);
#endif  /* ENABLE_TSTAMP */
            }
            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;


#if (MAC_INDIRECT_DATA_FFD == 1)
        case NULL_FRAME:
            /* Free the buffer allocated for the Null data frame */
            bmm_buffer_free(frame->buffer_header);

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */


#if (MAC_DISASSOCIATION_BASIC_SUPPORT == 1)
        case DISASSOCIATIONNOTIFICATION:
#if (MAC_DISASSOCIATION_FFD_SUPPORT == 1)
            if (frame->indirect_in_transit)
            {
                // Clear indirect in transmit flag
                frame->indirect_in_transit = false;
                if ((tx_status == MAC_SUCCESS) || (tx_status == TAL_FRAME_PENDING))
                {
                    remove_frame_from_indirect_q(frame);

                    /* Create the MLME DISASSOCIATION confirmation message */
                    /*
                     * Prepare disassociation confirm message after transmission of
                     * the disassociation notification frame.
                     */
                    mac_prep_disassoc_conf(frame->buffer_header,
                                           tx_status);
                }
            }
            else    // Not indirect
#endif  /* (MAC_DISASSOCIATION_FFD_SUPPORT == 1) */
            {
                /* Create the MLME DISASSOCIATION confirmation message */
                /*
                 * Prepare disassociation confirm message after transmission of
                 * the disassociation notification frame.
                 */
                mac_prep_disassoc_conf(frame->buffer_header,
                                       tx_status);
            }

            /*
             * Only an associated device should go to idle on transmission of a
             * disassociation frame.
             */
            if (MAC_ASSOCIATED == mac_state)
            {
                /*
                 * Entering sleep mode is already done implicitly in
                 * mac_idle_trans().
                 */
                mac_idle_trans();
            }

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif  /* (MAC_DISASSOCIATION_BASIC_SUPPORT == 1) */


#if (MAC_ASSOCIATION_REQUEST_CONFIRM == 1)
        case ASSOCIATIONREQUEST:
            if ((MAC_NO_ACK == tx_status) || (MAC_CHANNEL_ACCESS_FAILURE == tx_status))
            {
                /*
                 * Association Request frame could not be sent.
                 *
                 * The broadcast short address will only be used if the
                 * association attempt was unsuccessful.
                 * On a successful association, the
                 * actual value will be filled in based on the response to the
                 * data request obtained from the coordinator.
                 */
                mac_gen_mlme_associate_conf(frame->buffer_header, tx_status, BROADCAST);

                /* Set radio to sleep if allowed */
                mac_sleep_trans();
            }
            else
            {
                /* Successful transmission of Association Request. */
#ifdef TEST_HARNESS
                /*
                 * For purely testing purposes the coordinator may be set
                 * into a state where it will never respond with an
                 * Assoc Response frame
                 */
                if (mac_pib.privateNoDataAfterAssocReq >= 1)
                {
                    bmm_buffer_free(frame->buffer_header);

                    break;
                }
#endif /* TEST_HARNESS */

                mac_poll_state = MAC_AWAIT_ASSOC_RESPONSE;

                {
                    uint8_t status;
                    uint32_t response_timer;

                    /* Start the response wait timer */
                    response_timer = mac_pib.mac_ResponseWaitTime;
                    response_timer = TAL_CONVERT_SYMBOLS_TO_US(response_timer);

                    status = pal_timer_start(T_Poll_Wait_Time,
                                             response_timer, TIMEOUT_RELATIVE,
                                             (FUNC_PTR)mac_t_response_wait_cb,
                                             NULL);

#if (DEBUG > 0)
                    ASSERT(MAC_SUCCESS == status);
#endif
                    if (MAC_SUCCESS != status)
                    {
                        mac_t_response_wait_cb(NULL);
                    }
                }
            }
            break;
#endif /* (MAC_ASSOCIATION_REQUEST_CONFIRM == 1) */

#if (MAC_INDIRECT_DATA_BASIC == 1)
        case DATAREQUEST:           /* Explicit poll caused by MLME-POLL.request */
        case DATAREQUEST_IMPL_POLL: /* Implicit poll without MLME-POLL.request */
            {
                /* Free the data_request buffer */
                bmm_buffer_free(frame->buffer_header);

                /*
                 * In case we are in the middle of an association procedure,
                 * we nothing here, but keep in the same state.
                 * Also the poll timer is NOT canceled here, because it acts
                 * as a timer for the entire association procedure.
                 */
                if (MAC_AWAIT_ASSOC_RESPONSE != mac_poll_state)
                {
                    if (DATAREQUEST == frame->msg_type)
                    {
                         /* Explicit poll caused by MLME-POLL.request */
                        if (TAL_FRAME_PENDING != tx_status)
                        {
                            /* Reuse the poll request buffer for poll confirmation */
                            mlme_poll_conf_t *mpc = (mlme_poll_conf_t *)BMM_BUFFER_POINTER
                                                    ((buffer_t *)mac_conf_buf_ptr);

                            mpc->cmdcode = MLME_POLL_CONFIRM;
                            mpc->status = tx_status;
                            qmm_queue_append(&mac_nhle_q,
                                             (buffer_t *)mac_conf_buf_ptr);

                            /* Set radio to sleep if allowed */
                            mac_sleep_trans();
                            return;
                        }

                        /* Wait for data reception */
                        mac_poll_state = MAC_POLL_EXPLICIT;
                    }
                    else
                    {
                        if ((MAC_NO_ACK != tx_status) && (MAC_CHANNEL_ACCESS_FAILURE != tx_status))
                        {
                            /*
                             * Sucessful transmission of Data Request frame due to
                             * implicit poll without explicit poll request.
                             */
                        	mac_poll_state = MAC_POLL_IMPLICIT;
                        }
                        else
                        {
                            /* Data request for implicit poll could not be sent. */
                            /* Set radio to sleep if allowed */
                            mac_sleep_trans();
                            return;
                        }
                    }

                    {
                        uint8_t status;
                        uint32_t response_timer;
                        /*
                         * Start T_Poll_Wait_Time only if we are not in the
                         * middle of an association.
                         */
                        response_timer = mac_pib.mac_MaxFrameTotalWaitTime;
                        response_timer = TAL_CONVERT_SYMBOLS_TO_US(response_timer);

                        status = pal_timer_start(T_Poll_Wait_Time,
                                                 response_timer,
                                                 TIMEOUT_RELATIVE,
                                                 (FUNC_PTR)mac_t_poll_wait_time_cb,
                                                 NULL);

                        /*
                         * If we are waiting for a pending frame,
                         * the MAC needs to remain busy.
                         */
                        MAKE_MAC_BUSY();

                        if (MAC_SUCCESS != status)
                        {
                            mac_t_poll_wait_time_cb(NULL);
                        }
                    }
                }   /* (MAC_AWAIT_ASSOC_RESPONSE != mac_poll_state) */
            }
            break;
#endif /* (MAC_INDIRECT_DATA_BASIC == 1) */

#if (MAC_ASSOCIATION_INDICATION_RESPONSE == 1)
        case ASSOCIATIONRESPONSE:
            /*
             * Association Response is ALWAYS indirect, so not further check for
             * indirect_in_transit required, but clear the flag.
             */
            frame->indirect_in_transit = false;
            if ((tx_status == MAC_SUCCESS) || (tx_status == TAL_FRAME_PENDING))
            {
                /* Association resonse frames are always sent indirectly. */
                /*
                 * The removal of this frame from the indirect queue needs
                 * to be done BEFORE the subsequent call of function
                 * mac_mlme_comm_status(), since the variable frame is reused
                 * for the comm status indication buffer and would be invalid
                 * afterwards.
                 */
                remove_frame_from_indirect_q(frame);
               // gMlmeCommStatus_FrameType = ASSOCIATIONRESPONSE;
               // memcpy(&gMlmeCommStatus_DestAddr, frame->mpdu+PL_POS_DST_ADDR_START, sizeof(uint64_t));
                mac_mlme_comm_status(tx_status, frame->buffer_header);//û���ж�noack��ʧ��״̬������Ϊ���ӻ��п��ܻ���������ַ�ģ�����Ҫ������Ӷ��������Ϣ
            }
            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif  /* (MAC_ASSOCIATION_INDICATION_RESPONSE == 1) */

#if (MAC_ORPHAN_INDICATION_RESPONSE == 1)
        case ORPHANREALIGNMENT:
            mac_mlme_comm_status(tx_status,
                                     frame->buffer_header);

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif  /* (MAC_ORPHAN_INDICATION_RESPONSE == 1) */

#if (MAC_PAN_ID_CONFLICT_NON_PC == 1)
        case PANIDCONFLICTNOTIFICAION:
            /* Free the buffer allocated for the Pan-Id conflict notification frame */
            bmm_buffer_free(frame->buffer_header);

            /* Generate a sync loss to the higher layer. */
            mac_sync_loss(MAC_PAN_ID_CONFLICT);

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif  /* (MAC_PAN_ID_CONFLICT_NON_PC == 1) */

#if (MAC_START_REQUEST_CONFIRM == 1)
        case COORDINATORREALIGNMENT:
            /*
             * The coordinator realignment command has been sent out
             * successfully. Hence the MAC should be updated with the new
             * parameters given in the MLME.START_request with
             * coordinator realignment command
             */
            mac_coord_realignment_command_tx_success(tx_status,
                                                     frame->buffer_header);

            /* Set radio to sleep if allowed */
            mac_sleep_trans();
            break;
#endif /* (MAC_START_REQUEST_CONFIRM == 1) */

#if (MAC_START_REQUEST_CONFIRM == 1)
        case BEACON_MESSAGE:
#ifndef BEACON_SUPPORT
            /*
             * The code for freeing up the buffer is only included for a
             * build without beacon support, since in this case the static
             * beacon buffer is NOT used, but a standard buffer is used
             * instead, which needs to be freed up.
             */
            bmm_buffer_free(frame->buffer_header);
#endif  /* No BEACON_SUPPORT */
            /* Set radio to sleep if allowed */
            mac_sleep_trans();

            break;
#endif /* (MAC_START_REQUEST_CONFIRM == 1) */



























//#ifndef _ENDDEVICE_
//        case NWK_CMD_LEAVE_N:
//#ifndef _ENDDEVICE_
//        	if (frame->NwkFrameHeader->frameControl.dstExtAddr == true)//�ñ����뿪����
//                nwk_gen_nlme_leave_conf(frame->buffer_header, tx_status, frame->NwkFrameHeader->field.dst_src.dstExt.value);
//            else
//#endif
//                nwk_gen_nlme_leave_conf(frame->buffer_header, tx_status, NULL);//���Լ��뿪����
//            mac_sleep_trans();
//
//             break;
             
//        case NWK_CMD_LEAVE_IND_FORWARD_N://����Ҫ�Ƴ��Լ������к��ӣ����Բ������뿪ת��֡
//#ifndef REDUCE_CHECK_PARMER1
//        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
//#endif
//            {
//             nwk_gen_nlme_leave_ind_forward(frame->buffer_header, frame->NwkFrameHeader->field.src.payload[1] & NWK_LEAVE_REJOIN);
//             mac_sleep_trans();
//            }
//             break;
//#endif
        case NWK_DATA_REQUEST_N:
            if (tx_status == MAC_SUCCESS)
            {
#ifdef _ENDDEVICE_

            	nwk_gen_nlde_data_conf(frame->buffer_header,
                                       (uint8_t)tx_status,
                                       (void *)frame->AppFrameHeader,0,0);
                mac_sleep_trans();
#endif
#ifndef _ENDDEVICE_
#ifndef REDUCE_CHECK_PARMER1
            	if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE)
            	{
            	nwk_gen_nlde_data_conf(frame->buffer_header,
                                       (uint8_t)tx_status,
                                       (void *)frame->AppFrameHeader,0,0);
                mac_sleep_trans();  
            	}
				else
				{
#endif
#ifdef dataReqContinue
            	if (pal_is_timer_running(APP_TIMER_REQUEST_DATA) == false)
#endif
            	{
					if (frame->AppFrameHeader->frameControl.ackReq == true)
					{
						retval_t timer_status;
						frame->errorStatus = ERROR_SEND_TIMEOUT;
						timer_status =
							pal_timer_start(APP_TIMER_REQUEST_DATA,
											frame->AppFrameHeader->frameControl.timeOut*100000 + NWK_RETRY_TIMEOUT,//�����ack��С��ʱ500ms
											TIMEOUT_RELATIVE,
											(FUNC_PTR)app_t_DataRequest_cb,
											frame);

					 //   ASSERT(MAC_SUCCESS == timer_status);
						if (MAC_SUCCESS != timer_status)
						{
							nwk_gen_nlde_data_conf(frame->buffer_header,
												   (uint8_t)timer_status,
												   (void *)frame->AppFrameHeader,0,0);
							mac_sleep_trans();
						}
						else
						{
							//ooo++;
#ifdef dataReqContinue
							 qmm_queue_append(&wait_nwk_ack_q, frame->buffer_header);
							 NwkState = NWK_MODULE_NONE;
#endif
						}
					}
					else
					{
			            NwkState = NWK_MODULE_NONE;
			            bmm_buffer_free(frame->buffer_header);
			            mac_sleep_trans();
					}
					if (ledTxCounter == 0)
					{
						pal_led(LED_RF_TX, LED_ON);
						ledTxCounter = 48000;
					}


				}
#ifdef dataReqContinue
            	else
            	{
						 qmm_queue_append(&wait_nwk_ack_q, frame->buffer_header);
						 NwkState = NWK_MODULE_NONE;
            	}
#endif
#ifndef REDUCE_CHECK_PARMER1
				}
#endif
#endif
            }
            else
            {
            	gNwk_nib.transmitFailureCounter ++;
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
#ifndef _ENDDEVICE_
            {
            	frame->errorStatus = tx_status;
				pal_timer_start(APP_TIMER_REQUEST_DATA,
								frame->AppFrameHeader->frameControl.timeOut*100000 + NWK_RETRY_TIMEOUT,//�����ack��С��ʱ500ms
								TIMEOUT_RELATIVE,
								(FUNC_PTR)app_t_DataRequest_cb,
								frame);
            }
#endif
#ifndef REDUCE_CHECK_PARMER1
        	else
        	{
            	nwk_gen_nlde_data_conf(frame->buffer_header,
                                       (uint8_t)tx_status,
                                       (void *)frame->AppFrameHeader,0,0);
                mac_sleep_trans();                                  

            	//app_t_DataRequest_cb(frame);
            }
#endif
#ifdef    _ENDDEVICE_
        	nwk_gen_nlde_data_conf(frame->buffer_header,
                                   (uint8_t)tx_status,
                                   (void *)frame->AppFrameHeader,0,0);
            mac_sleep_trans();
#endif
            }
             break;
            
#ifndef _ENDDEVICE_             
        case NWK_DATA_IND_FORWARD_N://����ת��֡
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
#ifndef REDUCE_CHECK_PARMER5
            if ((MAC_NO_ACK == tx_status) || (MAC_CHANNEL_ACCESS_FAILURE == tx_status))
            {
            	if (frame->AppFrameHeader != NULL &&
					frame->AppFrameHeader->frameControl.state == APP_REQUEST &&
					frame->AppFrameHeader->frameControl.ackReq == true && (
					frame->AppFrameHeader->frameControl.frameType == APP_DATA ||
					frame->AppFrameHeader->AppPayload[0] != APP_CMD_REPORT_MY_ADDRESS))
            	wpan_nlme_NwkStatus_CommandFrame_req(NWK_NON_TREE_LINK_FAILURE,
            										frame->AppFrameHeader->DstAddress,
            										frame->NwkFrameHeader->srcAddr,
            										*(uint16_t *)&frame->mpdu[6],
            										frame->prevHopAddr);

            }
#endif
            }
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();

             break;             
#endif             
        case NWK_DATA_RESPONSE_N://Ӧ�ò����ݻظ�֡
        	if (frame->AppFrameHeader->frameControl.frameType == APP_CMD)
			{
        		switch (frame->AppFrameHeader->AppPayload[0])
        		{
        			case APP_CMD_HARD_REBOOT:
        				BackgroundCmdProce.byCmd = APP_CMD_HARD_REBOOT;
					break;

        			case APP_CMD_SLEEP:
        				BackgroundCmdProce.byCmd = APP_CMD_SLEEP;
					break;

        			case APP_CMD_POWER_DOWN:
        				BackgroundCmdProce.byCmd = APP_CMD_POWER_DOWN;
					break;

        			default:
					break;
        		}
			}
#ifdef SPI_COMMUNICTION
        	else //TODO ����spi���ͽ�����������
        	{
        		extern uint8_t spiTxLen;
                SPI_I2S_SendData(SPI2,spiTxLen);
                PIN_CLEAR(DR_485, PIO_B);  //485����
                for(int i=0;i<1;i++)
                {
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                asm  ("nop");
                }
                PIN_SET(DR_485, PIO_B);   //485����
        	}
#endif
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();           
          
          break;
 #ifndef _ENDDEVICE_        
        
        case NWK_CMD_LINK_STATUS_N://��·״̬֡
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
             NwkState = NWK_MODULE_NONE;
             NwkLinkStatusState = NWK_LINK_STATUS_WAIT_STATE;             
             bmm_buffer_free(frame->buffer_header);
             mac_sleep_trans();
            }
             break;  

        case NWK_CMD_ROUTE_REQUEST_N:
//            if (tx_status == MAC_SUCCESS)
            {
//                if (!pal_is_timer_running(T_NWK_ROUTE_DISCOVERY_TIME) && gNwkRouteDiscoveryTable.isTimerStarted == false)
//                {
//					retval_t timer_status;
//					timer_status =
//						pal_timer_start(T_NWK_ROUTE_DISCOVERY_TIME,
//										NWKC_ROUTE_DISCOVERY_TIME*1000,
//										TIMEOUT_RELATIVE,
//										(FUNC_PTR)nwk_t_RouteDiscoveryExpired_cb,
//										NULL);
//
//				 //   ASSERT(MAC_SUCCESS == timer_status);
//					if (MAC_SUCCESS != timer_status)
//					{
//	//                    nwk_gen_nlde_data_conf(frame->buffer_header,
//	//                                           (uint8_t)timer_status,
//	//                                           (void *)frame->AppFrameHeader,0,0);
//	//                    mac_sleep_trans();
//					}
//					else
//					{
//						gNwkRouteDiscoveryTable.isTimerStarted = true;
//					}
//                }
            }
//            else if (tx_status == MAC_CHANNEL_ACCESS_FAILURE)
            {
//                nwk_gen_nlde_data_conf(frame->buffer_header,
//                                       (uint8_t)tx_status,
//                                       (void *)frame->AppFrameHeader,0,0);
//                mac_sleep_trans();
            }

        	break;

        case NWK_CMD_ROUTE_REQUEST_RELAY_N:
            if (gRouteRelayRetries >= NWKC_RELAY_RREQ_RETRIES)
            {
            	gRouteRelayRetries = 0;
                NwkState = NWK_MODULE_NONE;//Ҫ����·�ɷ���confirm
                bmm_buffer_free(frame->buffer_header);
                mac_sleep_trans();

            }
          //  if (tx_status == MAC_SUCCESS)
 //           {
//                if (!pal_is_timer_running(T_NWK_ROUTE_DISCOVERY_TIME) && gNwkRouteDiscoveryTable.isTimerStarted == false)
//                {
//					retval_t timer_status;
//					timer_status =
//						pal_timer_start(T_NWK_ROUTE_DISCOVERY_TIME,
//										NWKC_ROUTE_DISCOVERY_TIME*1000,
//										TIMEOUT_RELATIVE,
//										(FUNC_PTR)nwk_t_RouteDiscoveryExpired_cb,
//										NULL);
//
//				 //   ASSERT(MAC_SUCCESS == timer_status);
//					if (MAC_SUCCESS != timer_status)
//					{
//	//                    nwk_gen_nlde_data_conf(frame->buffer_header,
//	//                                           (uint8_t)timer_status,
//	//                                           (void *)frame->AppFrameHeader,0,0);
//	//                    mac_sleep_trans();
//					}
//					else
//					{
//						gNwkRouteDiscoveryTable.isTimerStarted = true;
//					}
//                }
//            }
   //         else if (tx_status == MAC_CHANNEL_ACCESS_FAILURE)
 //           {
//                nwk_gen_nlde_data_conf(frame->buffer_header,
//                                       (uint8_t)tx_status,
//                                       (void *)frame->AppFrameHeader,0,0);
//                mac_sleep_trans();
 //           }

        	break;

        case NWK_CMD_ROUTE_REPLY_N:
//            if (!pal_is_timer_running(T_NWK_ROUTE_DISCOVERY_TIME) && gNwkRouteDiscoveryTable.isTimerStarted == false)
//            {
//				retval_t timer_status;
//				timer_status =
//					pal_timer_start(T_NWK_ROUTE_DISCOVERY_TIME,
//									NWKC_ROUTE_DISCOVERY_TIME*1000,
//									TIMEOUT_RELATIVE,
//									(FUNC_PTR)nwk_t_RouteDiscoveryExpired_cb,
//									NULL);
//
//			 //   ASSERT(MAC_SUCCESS == timer_status);
//				if (MAC_SUCCESS != timer_status)
//				{
////                    nwk_gen_nlde_data_conf(frame->buffer_header,
////                                           (uint8_t)timer_status,
////                                           (void *)frame->AppFrameHeader,0,0);
////                    mac_sleep_trans();
//				}
//				else
//				{
//					gNwkRouteDiscoveryTable.isTimerStarted = true;
//				}
//            }
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();
        	break;

        case NWK_CMD_ROUTE_REPLY_RELAY_N:
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();
        	break;
#ifndef _ENDDEVICE_

        case NWK_CMD_ROUTE_RECORD_N:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
//            NwkState = NWK_MODULE_NONE;
//            bmm_buffer_free(frame->buffer_header);
//            mac_sleep_trans();
            if (tx_status == MAC_SUCCESS)
            {
					//if (frame->AppFrameHeader->frameControl.timeOut == 0)
					//	frame->AppFrameHeader->frameControl.timeOut = 1;
            		retval_t timer_status;
            		frame->errorStatus = ERROR_SEND_TIMEOUT;
					timer_status =
						pal_timer_start(APP_TIMER_REQUEST_DATA,
										frame->AppFrameHeader->frameControl.timeOut*100000 + NWK_RETRY_TIMEOUT,//�����ack��С��ʱ500ms
										TIMEOUT_RELATIVE,
										(FUNC_PTR)app_t_DataRequest_cb,
										frame);

				 //   ASSERT(MAC_SUCCESS == timer_status);
					if (MAC_SUCCESS != timer_status)
					{
						nwk_gen_nlde_data_conf(frame->buffer_header,
											   (uint8_t)timer_status,
											   (void *)frame->AppFrameHeader,0,0);
						mac_sleep_trans();
					}
					else
					{

					}
            }
            else
            {
            	gNwk_nib.transmitFailureCounter ++;
            	frame->errorStatus = tx_status;
            	app_t_DataRequest_cb(frame);
            }
            }
        	break;
#endif                
        case NWK_CMD_ROUTE_RECORD_RELAY_N:
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();
            break;

        case NWK_CMD_NETWORK_STATUS_N:
        	NwkStatusState = NWK_STATUS_SENDER_LAST_STATE;
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();
            break;

#endif

        case NWK_DATA_BROADCAST_N:
           // if (tx_status == MAC_SUCCESS)
            {

				NwkState = NWK_MODULE_NONE;
				//bmm_buffer_free(frame->buffer_header);
				//mac_sleep_trans();

					//pal_led(LED_0, LED_ON);
            }
            //else
            {
            	//gNwk_nib.transmitFailureCounter ++;

//        	if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE)
//
//
//        	nwk_gen_nlde_data_conf(frame->buffer_header,
//                                   (uint8_t)tx_status,
//                                   (void *)frame->AppFrameHeader,0,0);
//            mac_sleep_trans();

            }
            break;

        case NWK_DATA_BROADCAST_RELAY_N:

            break;

        default:
#if (DEBUG > 0)
            ASSERT("Unknown message type" == 0);
#endif
            /* Set radio to sleep if allowed */
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(frame->buffer_header);
            mac_sleep_trans();
            break;
    }
} /* mac_process_tal_tx_status() */



/**
 * @brief Callback function from TAL after the frame is transmitted
 *
 * This is a callback function from the TAL. It is used when an attempt
 * to transmit a frame is finished.
 *
 * @param status Status of transmission
 * @param frame Specifies pointer to the transmitted frame
 */
void tal_tx_frame_done_cb(retval_t status, frame_info_t *frame)
{
    /* Frame transmission completed, set dispatcher to not busy */
    MAKE_MAC_NOT_BUSY();

#if ((MAC_SCAN_ACTIVE_REQUEST_CONFIRM == 1) || (MAC_SCAN_ORPHAN_REQUEST_CONFIRM == 1))
    /* If ack requested and ack not received, or ack not requested */
    if (
        ((MAC_SCAN_ACTIVE == mac_scan_state) && (frame->msg_type == BEACONREQUEST)) ||
        ((MAC_SCAN_ORPHAN == mac_scan_state) && (frame->msg_type == ORPHANNOTIFICATION))
       )
    {
        mac_scan_send_complete(status);
    }
    else
#endif /* ((MAC_SCAN_ACTIVE_REQUEST_CONFIRM == 1) || (MAC_SCAN_ORPHAN_REQUEST_CONFIRM == 1)) */
    {
        mac_process_tal_tx_status(status, frame);
    }
}



#if (MAC_INDIRECT_DATA_FFD == 1)
/**
 * @brief Helper function to remove transmitted indirect data from the queue
 *
 * @param f_ptr Pointer to frame_info_t structure of previously transmitted frame
 */
static void remove_frame_from_indirect_q(frame_info_t *f_ptr)
{
    search_t find_buf;

    find_buf.criteria_func = find_buffer_cb;

    /* Update the address to be searched */
    find_buf.handle = (void *)f_ptr->buffer_header;

    qmm_queue_remove(&indirect_data_q, &find_buf);
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */



#if (MAC_INDIRECT_DATA_FFD == 1)
/**
 * @brief Checks whether the indirect data frame address matches
 * with the address passed.
 *
 * @param buf Pointer to indirect data buffer
 * @param buffer Pointer to the buffer to be searched
 *
 * @return 1 if address matches, 0 otherwise
 */

static uint8_t find_buffer_cb(void *buf, void *buffer)
{
    uint8_t *buf_body = BMM_BUFFER_POINTER((buffer_t *)buffer);
    if (buf == buf_body)
    {
        return 1;
    }
    return 0;
}
#endif /* (MAC_INDIRECT_DATA_FFD == 1) */

/* EOF */
