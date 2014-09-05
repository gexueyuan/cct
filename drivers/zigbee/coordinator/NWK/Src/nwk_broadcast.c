/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
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
#ifdef MAC_SECURITY_ZIP
#include "mac_security.h"
#endif  /* MAC_SECURITY_ZIP */



#include "nwk_msg_const.h"
#include "nwk_api.h"
#include "nwk_msg_types.h"
#include "nwk_internal.h"
#include "nwk.h"
#include "nwk_config.h"
#include "nwk1.h"
#include "nwkCommon.h"
#include "nwkConfig.h"
#include "main1.h"
#include "nwkStateMachine.h"
#include "nwkFrame.h"
#include "nwkAddress.h"

//#include "nwkBTT.h"
#include "nwkPassiveAck.h"

#include "nwkDataReq.h"
#include "nldeData.h"
#include "tal_internal.h"
#include "app_uart.h"
#include "pal_uart.h"
#ifdef STM32F051
#include "stm32f0xx_iwdg.h"
#include "stm32f0xx_pwr.h"
#elif defined STM32F10X_MD
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#elif defined STM32F4XX
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_pwr.h"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Globals variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

#include <stm32f4xx.h>
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
extern int vam_rcp_recv(rcp_rxinfo_t *rxinfo, uint8_t *databuf, uint32_t datalen);
uart_communication_buffer_t uart_0_buffer;

bool blackListEnable = 0;
bool whiteListEnable = 0;
uint16_t blackTable[32];
uint32_t blackBitMap = 0;
uint8_t blackListCount = 0;

uint16_t whiteTable[32];
uint32_t whiteBitMap = 0;
uint8_t whiteListCount = 0;
bool isOUtNet = 1;
uint8_t outDebug[20];
uint16_t zigbeeDistance;
uint16_t zigbeeDelta;

int8_t set_fire_state(uint16_t state)
{
	if (state == 0)
	{
		blackListEnable = 0;
		whiteListEnable = 0;
		return 0;
	}
	else if (state == 1)
	{
		blackListEnable = 1;
		whiteListEnable = 0;
		return 1;
	}
	else if (state == 2)
	{
		whiteListEnable = 1;
		blackListEnable = 0;
		return 2;
	}		
	return -1;
}
FINSH_FUNCTION_EXPORT(set_fire_state, debug: set firewall default_state);



int8_t find_black_list(uint16_t srcAddr)
{
	uint8_t i,j=0;

	if (blackListCount > 0)
	{
		for(i=0; i<32; i++)
		{
				if (blackBitMap & (1<<i))
				{
						j++;
						if (blackTable[i] == srcAddr)
						{
								return i;
						}
						if (j == blackListCount)
								break;
				}
		}
	}
	return -1;
}

int8_t add_black_list(uint16_t srcAddr)
{
	int8_t index = -1;
	index = find_black_list(srcAddr);
	if (index != -1)
		return index;
	index = __CLZ(__RBIT(~blackBitMap));
	if (index < 32)
	{
		blackBitMap |= 1 << index;
		blackTable[index] = srcAddr;
		blackListCount++;
	}
	return index;
}
FINSH_FUNCTION_EXPORT(add_black_list, debug: add black list);


int8_t del_black_list(uint16_t srcAddr)
{
	int8_t index = -1;
  index = find_black_list(srcAddr);
	if (index != -1 && index < 32)
	{
		blackBitMap &= ~(1 << index);
		blackTable[index] = 0xFFFF;
		blackListCount--;		
	}
		
	return index;
}
FINSH_FUNCTION_EXPORT(del_black_list, debug: del black list);



int8_t find_white_list(uint16_t srcAddr)
{
	uint8_t i,j=0;

	if (whiteListCount > 0)
	{
		for(i=0; i<32; i++)
		{
				if (whiteBitMap & (1<<i))
				{
						j++;
						if (whiteTable[i] == srcAddr)
						{
								return i;
						}
						if (j == whiteListCount)
								break;
				}
		}
	}
	return -1;
}

int8_t add_white_list(uint16_t srcAddr)
{
	int8_t index = -1;
	index = find_white_list(srcAddr);
	if (index != -1)
		return index;	
	index = __CLZ(__RBIT(~whiteBitMap));
	if (index < 32)
	{
		whiteBitMap |= 1 << index;
		whiteTable[index] = srcAddr;
		whiteListCount++;
	}
	return index;
}
FINSH_FUNCTION_EXPORT(add_white_list, debug: add white list);

int8_t del_white_list(uint16_t srcAddr)
{
	int8_t index = -1;
  index = find_white_list(srcAddr);
	if (index != -1 && index < 32)
	{
		whiteBitMap &= ~(1 << index);
		whiteTable[index] = 0xFFFF;
		whiteListCount--;		
	}
		
	return index;
}
FINSH_FUNCTION_EXPORT(del_white_list, debug: del white list);

uint8_t pal_sio_init(uint8_t UARTx) 
{

  buffer_t *buffer_header;
  NWK_DataReq_t *nlde_data_req;

  buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);
  nlde_data_req = (NWK_DataReq_t *)BMM_BUFFER_POINTER(buffer_header);
  uart_0_buffer.rx_buf_head = ((uint8_t *)nlde_data_req) + (LARGE_BUFFER_SIZE - FCS_LEN - TRANSPARENT_IO_BUF_SIZE);
  uart_0_buffer.rx_buf[0] = (uint8_t *)buffer_header;
	UARTx = UARTx;
	return 0;
}

void out_net()
{
	isOUtNet = !isOUtNet;
}
FINSH_FUNCTION_EXPORT(out_net, debug: out_net_information);


uint8_t pal_sio_tx_vanet(uint16_t srcAddr, uint16_t prevHopAddr, uint8_t rssi, uint8_t radius, uint8_t *data, uint8_t length)
{
	rcp_rxinfo_t rxbd;
	static uint8_t sumCount = 0;
	static uint8_t preSeq;
	static uint16_t lostSeqSum = 0;
	static uint8_t maxAdjLostSeq = 0;
	uint8_t adjLostSeq=0;
	uint16_t packetErrorRate=0;

	
	rxbd.src[0] = srcAddr;
	rxbd.src[1] = srcAddr>>8;
	rxbd.hops = radius;
	rxbd.rssi = rssi;
	if (isOUtNet)
	{
		
		if (sumCount == 0)
		{
			preSeq = data[1];
			maxAdjLostSeq = 0;
			lostSeqSum = 0;
		}
		else
		{
			if (data[1] != preSeq+1)
			{
				adjLostSeq = (data[1]>preSeq?data[1]-preSeq-1:255-preSeq+data[1]);
				lostSeqSum += adjLostSeq;
				if (adjLostSeq>maxAdjLostSeq)
					maxAdjLostSeq = adjLostSeq;
			}
			preSeq = data[1];
		}
		
		sumCount ++;
		
		if (sumCount == 255)
		{
			sumCount = 0;
			packetErrorRate = lostSeqSum*1000/(255+lostSeqSum);
		}		
		
	//	if(sumCount != 0)
//		{
//			outDebug[0] = srcAddr;
//			outDebug[1] = srcAddr>>8;
//			outDebug[2] = data[1];
//			outDebug[3] = adjLostSeq;
//			outDebug[4] = maxAdjLostSeq;
//			outDebug[5] = packetErrorRate;
//			outDebug[6] = packetErrorRate>>8;			
//			outDebug[7] = lostSeqSum;
//			outDebug[8] = lostSeqSum>>8;			
//			outDebug[9] = sumCount;
//			outDebug[10] = radius;
//			outDebug[11] = rssi;	
//			outDebug[12] = zigbeeDistance;
//			outDebug[13] = zigbeeDistance>>8;
//			outDebug[14] = zigbeeDistance>>16;
//			outDebug[15] = zigbeeDistance>>24;
//			for (int i=0; i<16; i++)
//			{
//        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == 0);      
//        USART2->DR = (outDebug[i] & (uint16_t)0x01FF);    	
//			}				
//		}

	//	    rt_kprintf("src=%d, seq=%d, adjLostSeq=%d, maxAdjLostSeq=%d, packetErrorRate=%d%%, lostSeqSum=%d, sumCount=%d, radius=%d, rssi=%d  ", srcAddr,data[1],adjLostSeq,maxAdjLostSeq,packetErrorRate,lostSeqSum,sumCount,radius,rssi);
	//	else 
	//		  rt_kprintf("                     src=%d, seq=%d, adjLostSeq=%d, maxAdjLostSeq=%d, packetErrorRate=%d%%, lostSeqSum=%d, sumCount=%d, radius=%d, rssi=%d  ", srcAddr,data[1],adjLostSeq,maxAdjLostSeq,packetErrorRate,lostSeqSum,sumCount,radius,rssi);
	
	
	}
	else sumCount = 0;

	vam_rcp_recv(&rxbd, (uint8_t *)data, length);


	return 1;
}
int32_t wnet_dataframe_send(rcp_txinfo_t *txinfo, 
                           uint8_t *databuf, 
                           uint32_t datalen)
{
    buffer_t *buffer_header;
    uint8_t *nlde_data_req;
    if (uart_0_buffer.rx_buf_head == NULL)
    	return -1;
     AppFrameHeader_t *app = (AppFrameHeader_t *)uart_0_buffer.rx_buf_head;
    *uart_0_buffer.rx_buf_head = 0xF1;
    *(uart_0_buffer.rx_buf_head+1) = 0x00;



    app->AppSequenceNumber = gNwk_nib.sequenceNumber;
    app->DstAddress = 0xFFFF;

    app->length = datalen;
    memcpy(app->AppPayload,databuf,datalen);
    uart_0_buffer.rx_buf_head[app->length +7] = XORSUM((uint8_t *)app,app->length +7);


    wpan_nlde_data_req( NWK_DSTADDRMODE_RESPONSE,
    					app->DstAddress,
    					app->length +8,
                                 uart_0_buffer.rx_buf_head,
                                 uart_0_buffer.rx_buf[0],
    					txinfo->hops,
    					0,
    					true,
    					UART_ALLOC,
    					APP_ROUTE_MESH);
    					
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);
    if (buffer_header == NULL)
		{
			uart_0_buffer.rx_buf_head = NULL;
    	return -1;
		}
    nlde_data_req = BMM_BUFFER_POINTER(buffer_header);
    uart_0_buffer.rx_buf_head = (nlde_data_req) + (LARGE_BUFFER_SIZE - FCS_LEN - TRANSPARENT_IO_BUF_SIZE);
    uart_0_buffer.rx_buf[0] = (uint8_t *)buffer_header;							

    return 1;
}











bool bubble_sort_broad(uint8_t index)
{
    uint8_t  i, j;
    bool change,restart = false;
    NwkBroadDelay_t temp;


    change = true;
    j=0;
    i = NWK_MAX_BROADCASR_TABLE_ENTRY-1;
    
  //  if (index == 0)
   //   index = 0;

    if ((nwkBroadDelay[minBroadDelayIndex].delayCount - nwkBroadDelay[index].delayCount) < INT32_MAX || minBroadDelayIndex == index)
    	restart = true;


    for (; i>=1 && change; --i)
    {
      change = false;
      for ( j = 0; j<i; ++j)
      {
        if (nwkBroadDelay[j].transmit_frame == NULL
        	|| ( nwkBroadDelay[j].delayCount != nwkBroadDelay[j+1].delayCount
        	     && nwkBroadDelay[j+1].transmit_frame != NULL
                 && (nwkBroadDelay[j].delayCount - nwkBroadDelay[j+1].delayCount) < INT32_MAX )
               )//TODO 反转排序会有问题 用compare
        {
          temp       = nwkBroadDelay[j+1];
          nwkBroadDelay[j+1] = nwkBroadDelay[j];
          nwkBroadDelay[j]   = temp;
          change     = true;
         // if (j == 0)
        //	  restart = true;
        }
      }
    }
    broadDelayBitmap = 0;
    for (i=0; i<broadDelayCount; i++)
    {
        broadDelayBitmap |= 1<<i;
    }
    minBroadDelayIndex = 0;
    return restart;
}
/* Global  functions ---------------------------------------------------------*/
NWK_PRIVATE bool nwkNewPassiveAck(frame_info_t *const txDelay,
  const ShortAddr_t prevHopAddr, const ShortAddr_t srcAddr,
  const NwkSequenceNumber_t seqNum)
{
	uint8_t index;


	index = __CLZ(__RBIT(~gNwkPassiveAckTable.BitMap));
	if (index>=NWK_MAX_BROADCASR_TABLE_ENTRY)//考虑freelargequeue 只有3个多广播源定时器
		return false;
	gNwkPassiveAckTable.table[index].broacCastFrame = txDelay;
	gNwkPassiveAckTable.table[index].seqNum = seqNum;
	gNwkPassiveAckTable.table[index].srcAddr = srcAddr;
	gNwkPassiveAckTable.amount++;
	gNwkPassiveAckTable.BitMap |= 1<<index;
	gNwkPassiveAckTable.table[index].routerNum = 0;
	gNwkPassiveAckTable.table[index].endDeviceNum = 0;
	gNwkPassiveAckTable.table[index].prevHopAddr = prevHopAddr;
	gNwkPassiveAckTable.table[index].timerID = index+APP_TIMER_BC_DATA;
	if (txDelay == NULL)
		gNwkPassiveAckTable.table[index].end = true;
	else
		gNwkPassiveAckTable.table[index].end = false;
	gNwkPassiveAckTable.table[index].reTryTimes = 0;
#ifndef VANET
	uint8_t i,j=0;    
	if (srcAddr != gNwk_nib.networkAddress)
	{
		gNwkPassiveAckTable.table[index].realyNeighborNum = 1;
		NwkNeighbor_t *tempNeighbor;
		tempNeighbor = NWK_FindNeighborByShortAddr(prevHopAddr);
        if (tempNeighbor != NULL)
        {
        	tempNeighbor->isBroadRelay[index] = true;
        }
	}
	else gNwkPassiveAckTable.table[index].realyNeighborNum = 0;


	if (neighborTable.size != 0)
	{
		for (i=0; i<MAX_NEIGHBOR_TABLE_NUMBER ; i++)
		{
		  if (neighborTableArray[i].table.relationship != RELATIONSHIP_EMPTY)
		  {
			 j++;
			 if (neighborTableArray[i].table.deviceType == DEVICE_TYPE_COORDINATOR || neighborTableArray[i].table.deviceType == DEVICE_TYPE_ROUTER)
				 gNwkPassiveAckTable.table[index].routerNum++;
			 else gNwkPassiveAckTable.table[index].endDeviceNum++;

			 if (j == neighborTable.size)
				 break;
		  }
		}
	}
#endif
    return true;
}







NWK_PRIVATE
uint8_t nwkFindPassiveAck(const ShortAddr_t srcAddr, const NwkSequenceNumber_t seqNum)
{
	uint8_t i,j=0;

    if (gNwkPassiveAckTable.amount>0)
    {
      for(i=0; i<NWK_MAX_BROADCASR_TABLE_ENTRY; i++)
      {
          if (gNwkPassiveAckTable.BitMap & (1<<i))
          {
              j++;
              if (gNwkPassiveAckTable.table[i].srcAddr == srcAddr && gNwkPassiveAckTable.table[i].seqNum == seqNum)//没判断组
              {
                  return i;
              }
              if (j == gNwkPassiveAckTable.amount)
                  break;
          }
      }
    }
	return 0xFF;
}

/******************************************************************************
  \brief Frees an passive ack entry.

  \param[in] txDelay - the valid pointer to NwkTxDelayReq_t structure,
                       used to find and free an entry in the passive ack table
  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkFreePassiveAck(const ShortAddr_t srcAddr, const NwkSequenceNumber_t seqNum)
{
	uint8_t i,j=0;
	if (gNwkPassiveAckTable.amount > 0)
	{
		for(i=0; i<NWK_MAX_BROADCASR_TABLE_ENTRY; i++)
		{
			if (gNwkPassiveAckTable.BitMap & (1<<i))
			{
				j++;
				if (gNwkPassiveAckTable.table[i].srcAddr == srcAddr && gNwkPassiveAckTable.table[i].seqNum == seqNum)//没判断组
				{
					gNwkPassiveAckTable.BitMap &= ~(1<<i);

					gNwkPassiveAckTable.amount--;
#ifndef VANET              
                    uint8_t k;
			      	for (k=0; k<MAX_NEIGHBOR_TABLE_NUMBER; k++)
				    {
					    neighborTableArray[k].table.isBroadRelay[i] = false;
				    }
#endif                    
					return;
				}
				if (j == gNwkPassiveAckTable.amount)
					return;
			}
		}

	}
}

/******************************************************************************
  \brief Checks that all expected rebroadcast are received.

  \param[in] offset - offset of entry in the passive ack table.
  \return 'true' if all expected neighbors confirms that they was received
          original broadcast packet otherwise 'false'.
 ******************************************************************************/
//NWK_PRIVATE bool nwkIsPassiveAckDone(const NwkPassiveAckOffset_t offset)
//{
//
//}


/**
 * @brief Callback function for initiation of broadcast data transmission
 *
 * @param parameter Pointer to callback parameter
 *                  (not used in this application, but could be used
 *                  to indicated LED to be switched off)
 */
#ifndef VANET_REDUCE_FUNC     
void bc_data_cb(frame_info_t *transmit_frame)
{
	uint8_t index;
	index  = nwkFindPassiveAck(transmit_frame->NwkFrameHeader->srcAddr, transmit_frame->NwkFrameHeader->sequenceNumber);
	if (index != 0xFF)
	{
		if (gNwkPassiveAckTable.table[index].end == false)
		{
			do
			{
				if (gNwkPassiveAckTable.table[index].realyNeighborNum >= neighborTable.size)
					break;

				while (transmit_frame->AppFrameHeader->frameControl.tries != 0)//以后要用这个NWK_MAX_BROADCAST_RETRIES
				{
					transmit_frame->AppFrameHeader->AppPayload[transmit_frame->AppFrameHeader->length] ^= transmit_frame->AppFrameHeader->frameControl.tries;
					transmit_frame->AppFrameHeader->frameControl.tries--;
					transmit_frame->AppFrameHeader->AppPayload[transmit_frame->AppFrameHeader->length] ^= transmit_frame->AppFrameHeader->frameControl.tries;

					//mac_trx_wakeup();
					retval_t status = FAILURE;

					transmit_frame->mpdu[3] = mac_pib.mac_DSN++;
					/* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
					status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

					if (MAC_SUCCESS == status)
					{
						MAKE_MAC_BUSY();

						/* Start timer to initiate next broadcast data transmission. */
						pal_timer_start(gNwkPassiveAckTable.table[index].timerID,
										((uint32_t)NWK_PASSIVE_ACK_TIMEOUT),
										TIMEOUT_RELATIVE,
										(FUNC_PTR)bc_data_cb,
										transmit_frame);
						gNwkPassiveAckTable.table[index].reTryTimes++;
					}
					else
					{
						break;
					}
					return;
				}
			}while(0);
			NwkState = NWK_MODULE_NONE;
			pal_timer_start(gNwkPassiveAckTable.table[index].timerID,
							((uint32_t)	(NWK_BROADCAST_DELIVERY_TIME-(gNwkPassiveAckTable.table[index].reTryTimes+1)*NWK_PASSIVE_ACK_TIMEOUT)),
							TIMEOUT_RELATIVE,
							(FUNC_PTR)bc_data_cb,
							transmit_frame);
			gNwkPassiveAckTable.table[index].end = true;
			return;
		}
		gNwkPassiveAckTable.table[index].end = false;
		gNwkPassiveAckTable.table[index].reTryTimes = 0;
		nwkFreePassiveAck(transmit_frame->NwkFrameHeader->srcAddr, transmit_frame->NwkFrameHeader->sequenceNumber);
		if (transmit_frame != NULL)
			bmm_buffer_free(transmit_frame->buffer_header);
		//NwkState = NWK_MODULE_NONE;
		mac_sleep_trans();
	}
}
#endif

void bc_data_cb_vanet(frame_info_t *transmit_frame)
{
	uint8_t index;
	index  = nwkFindPassiveAck(transmit_frame->NwkFrameHeader->srcAddr, transmit_frame->NwkFrameHeader->sequenceNumber);
	if (index != 0xFF)
	{
#if 0
		if (gNwkPassiveAckTable.table[index].end == false)
		{

			NwkState = NWK_MODULE_NONE;
			pal_timer_start(gNwkPassiveAckTable.table[index].timerID,
							((uint32_t)	(NWK_BROADCAST_DELIVERY_TIME-(gNwkPassiveAckTable.table[index].reTryTimes+1)*NWK_PASSIVE_ACK_TIMEOUT)),
							TIMEOUT_RELATIVE,
							(FUNC_PTR)bc_data_cb_vanet,
							transmit_frame);
			gNwkPassiveAckTable.table[index].end = true;
			return;
		}
#endif
		gNwkPassiveAckTable.table[index].end = false;
		gNwkPassiveAckTable.table[index].reTryTimes = 0;
		nwkFreePassiveAck(transmit_frame->NwkFrameHeader->srcAddr, transmit_frame->NwkFrameHeader->sequenceNumber);
		if (transmit_frame != NULL)
			bmm_buffer_free(transmit_frame->buffer_header);
		//NwkState = NWK_MODULE_NONE;//if为1时原本是释掉的
		mac_sleep_trans();
	}
}


void bc_delay_cb(frame_info_t *transmit_frame)
{
	retval_t status;

	status = tal_tx_frame(nwkBroadDelay[minBroadDelayIndex].transmit_frame, CSMA_UNSLOTTED, true);
	broadDelayBitmap &= ~(1<<minBroadDelayIndex);
	nwkBroadDelay[minBroadDelayIndex].transmit_frame = NULL;
	nwkBroadDelay[minBroadDelayIndex].delayCount = 0xFFFFFFFF;
	broadDelayCount--;

	if (MAC_SUCCESS == status)
	{
		MAKE_MAC_BUSY();
	}
	else
	{
       // NwkState = NwkState;
        //bmm_buffer_free(nwkBroadDelay[minBroadDelayIndex].transmit_frame->buffer_header);//TODO 要多考虑,如果后面还有数据，需要再启动
        mac_sleep_trans();
	}



	if (broadDelayCount > 0)
	{
		//bubble_sort_broad();

		do
		{
			status = pal_timer_start(APP_TIMER_UART_TIMEOUT,
					nwkBroadDelay[++minBroadDelayIndex].delayCount,
					TIMEOUT_ABSOLUTE, (FUNC_PTR) bc_delay_cb,
					NULL);
			if (MAC_SUCCESS != status)
			{
				broadDelayBitmap &= ~(1<<minBroadDelayIndex);
				nwkBroadDelay[minBroadDelayIndex].transmit_frame = NULL;
				nwkBroadDelay[minBroadDelayIndex].delayCount = 0xFFFFFFFF;
				broadDelayCount--;
			}
		} while (MAC_SUCCESS != status && nwkBroadDelay[minBroadDelayIndex+1].transmit_frame != NULL);
	}
    
    if (broadDelayCount == 0)
      minBroadDelayIndex = 0;
}


NWK_PRIVATE
uint8_t nwkFindBroadDelay(const ShortAddr_t srcAddr, const NwkSequenceNumber_t seqNum)
{
	uint8_t i,j=0;

    if (broadDelayCount > 0)
    {
      for(i=0; i<NWK_MAX_BROADCASR_TABLE_ENTRY; i++)
      {
          if (broadDelayBitmap & (1<<i))
          {
              j++;
              if (nwkBroadDelay[i].transmit_frame->NwkFrameHeader->srcAddr == srcAddr
            	  && nwkBroadDelay[i].transmit_frame->NwkFrameHeader->sequenceNumber == seqNum)//没判断组
              {
                  return i;
              }
              if (j == broadDelayCount)
                  break;
          }
      }
    }
	return 0xFF;
}
