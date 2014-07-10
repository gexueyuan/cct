/* === Includes ============================================================= */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nwk_api.h"


#include "nwkIB.h"
#include "nlmeSetGet.h"
#include "main1.h"
#include "ieee_const.h"

#include "mac_api.h"
#include "nwk_api.h"
#include "mac_internal.h"

//#include "nlmeNetworkFormation.h"
//#include "nwkFormation.h"
#include "nwkStateMachine.h"
#include "nwk.h"
#include "nwk_config.h"
#include "tal_internal.h"
#include "app_uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Globals variables ---------------------------------------------------------*/
extern uint32_t ledRxCounter;
#ifdef WENSHIDU
extern unsigned char wenduANDshidu[];				 	//用于记录湿度

extern AppFrameHeader_t *App_W_AND_S_Header;
#endif
//int ccc=0,aaa=0;;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Global functions ---------------------------------------------------------*/
#ifdef dataReqContinue
static uint8_t check_ack_response_app_data(void *buf_ptr, void *handle)
{
    frame_info_t *frame = (frame_info_t *)buf_ptr;

    /* Frame shall not be in transmission. */
    if (frame->AppFrameHeader->AppSequenceNumber == *(uint16_t *)handle &&
    	frame->AppFrameHeader->DstAddress == *((uint16_t *)handle+1))
    {
            return 1;
    }

   // handle = handle;    /* Keep compiler happy. */

    return 0;
}
#endif
 
void usr_nlde_data_ind(NWK_DstAddrMode_t dstAddrMode, 
                       ShortAddr_t dstAddr,
                       ShortAddr_t srcAddr,
                       ShortAddr_t prevHopAddr,
                       NwkLength_t nsduLength,                      
                       uint8_t *nsdu,
                       Lqi_t linkQuality,                      
                       Rssi_t rssi,
                       uint32_t rxTime,                      
                       bool securityUse,
                       uint8_t radius
                       )
{
    AppFrameHeader_t *AppFrameHeader;    
    uint8_t length;
#ifndef _ENDDEVICE_    
    uint64_t extAddr;
#endif    
    if (XORSUM(nsdu, nsduLength) != 0)
    {
    	return;
    }
    AppFrameHeader = ( AppFrameHeader_t *)nsdu;
#ifdef dataReqContinue
    if (AppFrameHeader->frameControl.state == APP_RESPONSE)
#else
    if (AppFrameHeader->frameControl.state == APP_RESPONSE && NwkState == NWK_MODULE_ID_DATA_REQ)
#endif
    {
#ifdef dataReqContinue
    	uint16_t handle[2];
        search_t find_buf;
        buffer_t *buffer_ack_frame = NULL;

        handle[0] = AppFrameHeader->AppSequenceNumber;
        handle[1] = srcAddr;
        find_buf.criteria_func = check_ack_response_app_data;
        find_buf.handle = handle;

      //  do
      //  {
        	buffer_ack_frame = qmm_queue_remove(&wait_nwk_ack_q, &find_buf);

            if (NULL != buffer_ack_frame)
            {
                if (wait_nwk_ack_q.size == 0 && pal_is_timer_running(APP_TIMER_REQUEST_DATA) == true)//有问题
                {
                     //retval_t timer_status;
                     //timer_status =
                         pal_timer_stop(APP_TIMER_REQUEST_DATA);

                  //   ASSERT(MAC_SUCCESS == timer_status);

                }
                frame_info_t *localnwk_frame_ptr = (frame_info_t *)BMM_BUFFER_POINTER(buffer_ack_frame);

                nwk_gen_nlde_data_conf(buffer_ack_frame,
                						MAC_SUCCESS,
                                       (void *)localnwk_frame_ptr->AppFrameHeader,0,NWK_MAX_HOPS-radius);
                mac_sleep_trans();
            }
      //  }
    //    while (NULL != buffer_ack_frame);
#else
      if (AppFrameHeader->AppSequenceNumber == nwk_frame_ptr->AppFrameHeader->AppSequenceNumber &&
            (srcAddr == nwk_frame_ptr->AppFrameHeader->DstAddress || AppFrameHeader->DstAddress == nwk_frame_ptr->AppFrameHeader->DstAddress))//区分enddeviec被上级代替转发的
        {
    	      // aaa++;
    	       if (pal_is_timer_running(APP_TIMER_REQUEST_DATA) == true)
               {
                    retval_t timer_status;
#ifndef  REDUCE_CHECK_PARMER3
					AppFrameHeader->AppPayload[AppFrameHeader->length] ^= AppFrameHeader->travelTime;
					AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[1];

                    pal_get_current_time(&currentTime);
                    AppFrameHeader->travelTime = ((currentTime>>8)>nwk_frame_ptr->AppFrameHeader->travelTime) ?
                  		  	  	  	  	  	   (currentTime>>8)-nwk_frame_ptr->AppFrameHeader->travelTime :
                  		  	  	  	  	  	   nwk_frame_ptr->AppFrameHeader->travelTime - (currentTime>>8);
                    AppFrameHeader->frameControl.routeType = (app_route_method_t)(NWK_MAX_HOPS - radius + 1);

                    AppFrameHeader->AppPayload[AppFrameHeader->length] ^= AppFrameHeader->travelTime;
                    AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[1];
#endif
                    if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
                    	pal_sio_tx(SIO_0, nsdu, nsduLength);

                    timer_status = pal_timer_stop(APP_TIMER_REQUEST_DATA);
                
                 //   ASSERT(MAC_SUCCESS == timer_status);
                   nwk_gen_nlde_data_conf((buffer_t *)gNwk_conf_buf_ptr,
                                           timer_status,
                                           (void *)nwk_frame_ptr->AppFrameHeader,0,NWK_MAX_HOPS-radius+1);
                   mac_sleep_trans(); 
                  // ccc++;
               }
        }
#endif
      //  return;
    }
    else if (AppFrameHeader->frameControl.state == APP_REQUEST )
    {
        if (AppFrameHeader->frameControl.frameType == APP_CMD)//命令&& AppFrameHeader->frameControl.ackReq == true)
        {
            switch (AppFrameHeader->AppPayload[0])
            {
#ifndef _ENDDEVICE_
                case APP_CMD_REPORT_MY_ADDRESS://汇报自己的地址
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE)
        		return;
#endif
                  memcpy(&extAddr,&nsdu[8],8);
                  NwkAddress_Map_Add(extAddr,srcAddr);
                  goto label_3;
				//break;//statement is unreachable

                case APP_CMD_REPORT_NEIGHBOR://获取邻居表
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE)
        		return;
#endif
                  wpan_nlme_NwkLinkStatus_CommandFrame_req(srcAddr);
                  break;
#endif
                case APP_CMD_HARD_REBOOT:
                	if (AppFrameHeader->frameControl.ackReq == false)
                		BackgroundCmdProce.byCmd = APP_CMD_HARD_REBOOT;
                	goto label_3;
				//break;//statement is unreachable

                case APP_CMD_SLEEP:
                	if (AppFrameHeader->frameControl.ackReq == false)
                	{
                		BackgroundCmdProce.byCmd = APP_CMD_SLEEP;
                	}
            		BackgroundCmdProce.Data.dwData = AppFrameHeader->AppPayload[1];
                	goto label_3;
				//break;//statement is unreachable

                case APP_CMD_POWER_DOWN:
                	if (AppFrameHeader->frameControl.ackReq == false)
                		BackgroundCmdProce.byCmd = APP_CMD_POWER_DOWN;
                	goto label_3;
                	//break;

                case APP_CMD_GET_TAG_TABLE:
                	length = GetTagNeighborTableInfo(AppFrameHeader->AppPayload+1)+1;
                	AppFrameHeader->frameControl.state = APP_RESPONSE;
                    AppFrameHeader->frameControl.sendSuccess = true;
                    AppFrameHeader->frameControl.ackReq = false;
                    AppFrameHeader->frameControl.tries = 0;
                    AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
                    AppFrameHeader->length = length;
                    AppFrameHeader->AppPayload[length] = XORSUM(nsdu,length + APP_HEAD_LENGTH-1);
                    wpan_nlde_data_req(NWK_DSTADDRMODE_RESPONSE,
                                      srcAddr,
                                      APP_HEAD_LENGTH + length,
                                      nsdu,
                                      nsdu,
                                      NWK_MAX_HOPS,
                                      false,
                                      true,
                                      false,
                                      APP_ROUTE_MESH);
                	break;

                case APP_CMD_SEND_TAG_INFO:
                	UpdateTagneighborTable((TAG_NEIGHBOE_TABLE_ITEM *)(AppFrameHeader->AppPayload+1));
                	break;
#ifdef WENSHIDU
                case APP_CMD_GET_WEN_SHI_DU:
                	App_W_AND_S_Header->DstAddress = gNwk_nib.networkAddress;
                	App_W_AND_S_Header->AppSequenceNumber = AppFrameHeader->AppSequenceNumber;
    				wenduANDshidu[16] = XORSUM(wenduANDshidu,16);
    				wpan_nlde_data_req( NWK_DSTADDRMODE_RESPONSE,
    									srcAddr,
    									17,
    									wenduANDshidu,
    									wenduANDshidu,
    									NWK_MAX_HOPS,
    									false,
    									true,
    									false,
    									APP_ROUTE_MESH);
                	break;
#endif
                default:
    		        AppFrameHeader->frameControl.state = APP_RESPONSE;
    		        AppFrameHeader->frameControl.sendSuccess = false;
    		        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
    				AppFrameHeader->length = 2;
    				AppFrameHeader->AppPayload[1] = ERROR_CMD_NO_SUPPORT;
    				AppFrameHeader->AppPayload[2] = XORSUM(nsdu, 8);
    				pal_sio_tx(SIO_0, nsdu, 10);
    				return;
				//break;//statement is unreachable
            }
        }
        else

        {

label_3:

			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[4];
			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[5];
			AppFrameHeader->DstAddress = srcAddr;
			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[4];
			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[5];
#ifndef  REDUCE_CHECK_PARMER3
			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[2];
			AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
			AppFrameHeader->AppPayload[AppFrameHeader->length] ^= nsdu[2];
#endif
            if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
            	pal_sio_tx(SIO_0, nsdu, nsduLength);
            else pal_sio_tx_vanet(srcAddr, rssi, radius, AppFrameHeader->AppPayload, AppFrameHeader->length);

			if (ledRxCounter == 0)
			{
				pal_led(LED_RF_RX, LED_ON);
				ledRxCounter = 48000;
			}

            if (dstAddr >= BROADCAST_ADDR_ROUTERS)
            	return;

        	if (AppFrameHeader->frameControl.ackReq == false)
        		return;

#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE)
        		return;
#endif

#ifdef UART_DMA_TX
        	AppFrameHeader_t *AppFrameHeaderACK;
        	AppFrameHeaderACK = (AppFrameHeader_t *)gResponseData;
        	AppFrameHeaderACK->frameControl.frameType = AppFrameHeader->frameControl.frameType;
        	AppFrameHeaderACK->frameControl.state = APP_RESPONSE;
        	AppFrameHeaderACK->frameControl.sendSuccess = true;
        	AppFrameHeaderACK->frameControl.ackReq = false;
        	AppFrameHeaderACK->frameControl.tries = 0;
        	AppFrameHeaderACK->DstAddress = gNwk_nib.networkAddress;
        	AppFrameHeaderACK->AppSequenceNumber = AppFrameHeader->AppSequenceNumber;
#ifndef  REDUCE_CHECK_PARMER3
        	AppFrameHeaderACK->frameControl.sourceCount |= 1<<testRssi(rssi);
        	pal_get_current_time(&currentTime);
        	AppFrameHeaderACK->travelTime = currentTime>>8;
        	AppFrameHeaderACK->hops = NWK_MAX_HOPS-radius+1;
#endif
            if (AppFrameHeaderACK->frameControl.frameType == APP_DATA)
            {
            	AppFrameHeaderACK->length = 0;
            	AppFrameHeaderACK->AppPayload[0] = XORSUM(gResponseData,7);
            }
            else
            {
            	AppFrameHeaderACK->length = 1;
            	AppFrameHeaderACK->AppPayload[0] = AppFrameHeader->AppPayload[0];
            	AppFrameHeaderACK->AppPayload[1] = XORSUM(gResponseData,8);
            }


           // if (gResIndex > 207)
          //      gResIndex = 0;
           // memcpy(&gResponseData[gResIndex],nsdu,APP_HEAD_LENGTH + AppFrameHeader->length);

            wpan_nlde_data_req(NWK_DSTADDRMODE_RESPONSE,
                              srcAddr, 
                              APP_HEAD_LENGTH + AppFrameHeaderACK->length,
                              gResponseData,
                              gResponseData,
                              NWK_MAX_HOPS,
                              0,
                              (AppFrameHeader->frameControl.routeType == APP_ROUTE_TREE) ? false :true,
                              false,
                              AppFrameHeader->frameControl.routeType);
            
           // gResIndex += APP_HEAD_LENGTH + AppFrameHeader->length;
#else
            AppFrameHeader->frameControl.state = APP_RESPONSE;
            AppFrameHeader->frameControl.sendSuccess = true;
            AppFrameHeader->frameControl.ackReq = false;
            AppFrameHeader->frameControl.tries = 0;
            AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
#ifndef  REDUCE_CHECK_PARMER3
            AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
        	pal_get_current_time(&currentTime);
            AppFrameHeader->travelTime = currentTime>>8;
            AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
#endif
//            if (AppFrameHeader->frameControl.frameType == APP_DATA)
//            {
            	AppFrameHeader->length = 0;
                AppFrameHeader->AppPayload[0] = XORSUM(nsdu,7);
//            }
//            else
//            {
//            	AppFrameHeader->length = 1;
//            	AppFrameHeader->AppPayload[1] = XORSUM(nsdu,8);
//            }


           // if (gResIndex > 207)
          //      gResIndex = 0;
          //  memcpy(&gResponseData[gResIndex],nsdu,APP_HEAD_LENGTH + AppFrameHeader->length);

            wpan_nlde_data_req(NWK_DSTADDRMODE_RESPONSE,
                              srcAddr,
                              APP_HEAD_LENGTH + AppFrameHeader->length,
                              nsdu,
                              nsdu,
                              NWK_MAX_HOPS,
                              0,
                              (AppFrameHeader->frameControl.routeType == APP_ROUTE_TREE) ? false :true,
                              false,
                              AppFrameHeader->frameControl.routeType);

           // gResIndex += APP_HEAD_LENGTH + AppFrameHeader->length;
#endif
        }

    }
   // pal_sio_tx(0, nsdu, nsduLength);
#ifdef SIO_HUB
  //  static uint32_t rx_cnt = 0;
  //  rx_cnt ++;
  //  char sio_array[255];
  //  sprintf(sio_array, "Frame received: %" PRIu32 ",data: %" PRIu8 "\n",rx_cnt, *nsdu);
  //  printf(sio_array);

#endif  
    
}

