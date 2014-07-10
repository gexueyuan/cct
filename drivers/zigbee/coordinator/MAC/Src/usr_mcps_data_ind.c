/**
 * @file usr_mcps_data_ind.c
 *
 * @brief This file contains user call back function for MCPS-DATA.indication.
 *
 * $Id: usr_mcps_data_ind.c 26610 2011-05-11 08:47:45Z sschneid $
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
#include "main1.h"
#include <stdio.h>
#include <inttypes.h>

#include "nwkLeave.h"
#include "nwk.h"
#include "nwkLinkStatus.h"
#include "nwkStateMachine.h"
#include "mac_internal.h"
/* === Macros ============================================================== */


/* === Globals ============================================================= */

/* === Prototypes ========================================================== */
extern bool wpan_nlde_data_req(NWK_DstAddrMode_t dstAddrMode,
                        ShortAddr_t dstAddr,
                        uint8_t nsduLength,
                        uint8_t *nsdu,
                        uint8_t *nsduHandle,
                        uint8_t radius,
                        uint8_t nonMemberRadius,
                        bool    discoverRoute,
                        uint8_t    secutityEnable,
                        app_route_method_t routeType
                        );

/* === Implementation ====================================================== */

//int sss=0,ttt=0;
/* EOF */
void usr_mcps_data_ind(wpan_addr_spec_t *SrcAddrSpec,
                       wpan_addr_spec_t *DstAddrSpec,
                       uint8_t msduLength,
                       uint8_t *msdu,
                       uint8_t mpduLinkQuality,
                       uint8_t RSSI,
#ifdef ENABLE_TSTAMP
                       uint8_t DSN,
                       uint32_t Timestamp)
#else
                       uint8_t DSN,
                       buffer_t *m)
#endif  /* ENABLE_TSTAMP */
{
     uint8_t length = 8,i,findFlag = 0,insertPosition = 0xFF;
    
     gNwkFrameHeader_p = (NwkFrameHeader_t *)msdu;
     //pal_sio_tx(SIO_0, msdu, msduLength);//v2v
     for (i=0; i<30; i++)
     {
    	 if (gNwkSequence[i].srcAddress != 0xFFFF)
    	 {
			 if (gNwkFrameHeader_p->srcAddr == gNwkSequence[i].srcAddress)
			 {
				 if (gNwkFrameHeader_p->sequenceNumber == gNwkSequence[i].sequence)
				 {
					 findFlag = 2;//重复的出现过这个序列号
					 break;
				 }
				 else
				 {
					 findFlag = 1;//有这个地址，这个序列号作为新的更新一下
					 gNwkSequence[i].sequence = gNwkFrameHeader_p->sequenceNumber;
					 break;
				 }
			 }
    	 }
    	 else if (insertPosition == 0xFF)
    		 insertPosition = i;
     }

     if (findFlag == 0)//没有这个地址，在此加入进来
     {
    	 gNwkSequence[insertPosition].srcAddress = gNwkFrameHeader_p->srcAddr;
    	 gNwkSequence[insertPosition].sequence = gNwkFrameHeader_p->sequenceNumber;
     }
     
       if (gNwkFrameHeader_p->frameControl.dstExtAddr  == true)
       { 
           gNwkParse.dstExt = (NwkFrameExtAddr_t *)(msdu + length);
           length += 8;
       }
    
       if (gNwkFrameHeader_p->frameControl.srcExtAddr == true)
       { 
           gNwkParse.srcExt = (NwkFrameExtAddr_t *)(msdu + length);
           length += 8;
       }
  
       if (gNwkFrameHeader_p->frameControl.multicastFlag == true)
       { 
           gNwkParse.multicast = (NwkFrameMulticastField_t *)(msdu + length);
           length += 1;
       }
  
       if (gNwkFrameHeader_p->frameControl.sourceRoute == true)
       {
           gNwkParse.sourceRouteSubframe = (NwkSourceRouteSubframe_t *)(msdu + length);
           length += gNwkParse.sourceRouteSubframe->relayCount*2 + 2;
       }
       
       gNwkParse.payload = msdu + length;
       gNwkParse.headerSize = length;
       gNwkParse.payloadSize = msduLength - length;
       gNwkParse.macSrcAddr = SrcAddrSpec->Addr.short_address;
       gNwkParse.lqi = mpduLinkQuality;
       gNwkParse.rssi = RSSI;



       if (findFlag == 2)
       {
    	   if (gNwkFrameHeader_p->dstAddr < BROADCAST_ADDR_ROUTERS)
    	   {
#ifndef _ENDDEVICE_
#ifndef REDUCE_CHECK_PARMER1
    	   if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
    	   {
    		    if (XORSUM(gNwkParse.payload , gNwkParse.payloadSize) != 0)
    		    {
    		    	bmm_buffer_free(m);
    		    	return;
    		    }
     	        AppFrameHeader_t *AppFrameHeader;
     	        AppFrameHeader = ( AppFrameHeader_t *)gNwkParse.payload;

				if (AppFrameHeader->frameControl.frameType == APP_DATA)
				{
					AppFrameHeader->frameControl.state = APP_RESPONSE;
					AppFrameHeader->frameControl.sendSuccess = true;
					AppFrameHeader->frameControl.ackReq = false;
					AppFrameHeader->frameControl.tries = 0;
					AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
		#ifndef  REDUCE_CHECK_PARMER3
					AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(RSSI);
					pal_get_current_time(&currentTime);
					AppFrameHeader->travelTime = currentTime>>8;
					AppFrameHeader->hops = NWK_MAX_HOPS-gNwkFrameHeader_p->radius+1;
		#endif
					AppFrameHeader->length = 0;
					AppFrameHeader->AppPayload[0] = XORSUM(gNwkParse.payload,7);
					wpan_nlde_data_req(NWK_DSTADDRMODE_RESPONSE,//TODO 如果数据帧里有采集命令，比如得到温湿度和tag表，要跳过这块，把采集信息返回回去
									  gNwkFrameHeader_p->srcAddr,
									  APP_HEAD_LENGTH + AppFrameHeader->length,
									  gNwkParse.payload,
									  gNwkParse.payload,
									  NWK_MAX_HOPS,
									  0,
									  (AppFrameHeader->frameControl.routeType == APP_ROUTE_TREE) ? false :true,
									  false,
									  AppFrameHeader->frameControl.routeType);
		            bmm_buffer_free(m);
		            return;
				}
//				else
//				{
//					AppFrameHeader->length = 1;
//					AppFrameHeader->AppPayload[1] = XORSUM(gNwkParse.payload,8);
//				}


    	   }
#endif

    	   }
           else//重复的广播帧，不返回ack，直接丢掉
           {
               bmm_buffer_free(m);
               return;
           }
       }


       switch (gNwkFrameHeader_p->frameControl.frameType)
       {
          case NWK_FRAMETYPE_DATA:
              nwkParseHeader(gNwkFrameHeader_p, &gNwkParse, m);
            break;
         
          case NWK_FRAMETYPE_COMMAND:
            
              switch (gNwkParse.payload[0])
              {
//                  case NWK_CMD_LEAVE:
//                      nwkLeaveFrameInd(&gNwkParse.payload[1], gNwkFrameHeader_p, &gNwkParse);
//                    break;
#ifndef _ENDDEVICE_                     
                  case NWK_CMD_LINK_STATUS:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
                      if (DstAddrSpec->Addr.short_address == 0xFFFF)
                          nwkLinkStatusInd(gNwkParse.payload, gNwkFrameHeader_p, &gNwkParse);  
                      else
                      {
                          if (NwkState == NWK_MODULE_ID_DATA_REQ && gNwkFrameHeader_p->srcAddr == nwk_frame_ptr->AppFrameHeader->DstAddress )
                          {
                             if (pal_is_timer_running(APP_TIMER_REQUEST_DATA) == true)
                             {
                                  retval_t timer_status;
                                  nwk_frame_ptr->AppFrameHeader->frameControl.state = APP_RESPONSE;
                                  nwk_frame_ptr->AppFrameHeader->frameControl.sendSuccess = true;
                                  nwk_frame_ptr->AppFrameHeader->length = msduLength-16;
                                  memcpy(msdu+9,nwk_frame_ptr->AppFrameHeader,APP_HEAD_LENGTH);
                                  msdu[msduLength] = XORSUM(msdu+9,msduLength-17+APP_HEAD_LENGTH);
                            	  pal_sio_tx(0, msdu+9, msduLength-16+APP_HEAD_LENGTH);

                                  timer_status = pal_timer_stop(APP_TIMER_REQUEST_DATA);
                              
                                  nwk_gen_nlde_data_conf((buffer_t *)gNwk_conf_buf_ptr,//收到一个远程节点的链路状态帧，这个帧是用本地的数据请求来产生的，所以要conf数据请求
                                                         timer_status,
                                                         (void *)nwk_frame_ptr->AppFrameHeader,0,NWK_MAX_HOPS-msdu[6]);
                                  mac_sleep_trans();
                             }
                          }
                      }
            }
                    break;

                  case NWK_CMD_ROUTE_REQUEST:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
                	  nwkRouteRequestFrameInd(gNwkParse.payload, gNwkFrameHeader_p, &gNwkParse);
            }
                	  break;

                  case NWK_CMD_ROUTE_REPLY:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
                	  nwkRouteReplyFrameInd(gNwkParse.payload, gNwkFrameHeader_p, &gNwkParse);
            }
                	  break;

                  case NWK_CMD_ROUTE_RECORD:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
                	  nwkRouteRecordFrameInd(gNwkParse.payload, gNwkFrameHeader_p, &gNwkParse);
            }
                	  break;

                  case NWK_CMD_NETWORK_STATUS:
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
#ifndef REDUCE_CHECK_PARMER5
                	  nwkStatusFrameInd(gNwkParse.payload, gNwkFrameHeader_p, &gNwkParse);
#endif
            }
                	  break;
#endif
                  default :
                	  break;
              }
              bmm_buffer_free(m);
              break;

         default:
           	  break;
       }
#ifdef ENABLE_TSTAMP
    Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */

}
