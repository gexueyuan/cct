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
#include "pal_timer.h"
#elif defined STM32F4XX
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_pwr.h"
#include "pal_timer.h"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Globals variables ---------------------------------------------------------*/
//int eee=0;

/* Private function prototypes -----------------------------------------------*/
extern retval_t build_data_frame(mcps_data_req_t *pmdr,
                                 frame_info_t *frame,uint8_t fcfs);

static inline uint32_t gettime(void)
{
    uint16_t current_sys_time;
    uint32_t current_time;

    do
    {
        current_sys_time = TIME_STAMP_HIGH_REGISTER;
        current_time = current_sys_time;
        current_time = current_time << 16;
        current_time = current_time | TIMER_LOW_REGISTER;
        /*
         * This calculation is valid only if the timer has not rolled over.
         * The TIME_STAMP_HIGH_REGISTER variable may have changed in the timer overflow ISR.
         */
    }
    while (current_sys_time != TIME_STAMP_HIGH_REGISTER);

    return current_time;
}


#ifndef REDUCE_CHECK_PARMER
extern uart_communication_buffer_t uart_0_buffer;
#endif
#ifndef _ENDDEVICE_
extern void timer_init(void);
extern uint8_t mac_rx_enable(void);
//extern void init_sio(void);
#endif
/* Private functions ---------------------------------------------------------*/

/* Global  functions ---------------------------------------------------------*/
//uint32_t cloud_test3 = 0;
void nlde_data_request (uint8_t *msg)
{
    NwkFrameHeader_t *NwkFrameHeader;
    AppFrameHeader_t *AppFrameHeader;    
    uint8_t *payload_pos;
    NWK_DataReq_t  ndr;
    retval_t status = FAILURE;
    mcps_data_req_t mdr;
    frame_info_t *transmit_frame;    
    uint16_t N;
#ifndef _ENDDEVICE_
    GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t comData;
  

    NwkRoutingTableEntry_t *tempRoutingTableEntry;

#endif

    memcpy(&ndr, BMM_BUFFER_POINTER((buffer_t *)msg), sizeof(NWK_DataReq_t));
    AppFrameHeader = (AppFrameHeader_t *)ndr.nsdu;  

    if (ndr.securityEnable > 1)
    {
		if (ndr.securityEnable == TIME_OUT)
		{
#ifndef REDUCE_CHECK_PARMER
			if (IsPlusPlusPlus(ndr.nsdu, ndr.nsduLength) == true)
			{
				serialMode = USER_AT_COMMAND_MODE;
				uint8_t data[4] = {'A', 'T', 'O', 'K'};
				uart_0_buffer.rx_buf_head = 0;
				uart_0_buffer.rx_buf_tail = 0;//还需要清理之前没有发出去的nwkdatareq队列
				uart_0_buffer.rx_count = 0;
				pal_sio_tx(0, data, 4);
				bmm_buffer_free((buffer_t *)msg);
				return;
			}
			if (IsSubSubSub(ndr.nsdu, ndr.nsduLength) == true)
			{
				serialMode = USER_TRANSPARENT_MODE;
				uint8_t data[4] = {'T', 'S', 'O', 'K'};
				uart_0_buffer.rx_buf_head = 0;
				uart_0_buffer.rx_buf_tail = 7;//还需要清理之前没有发出去的nwkdatareq队列
				uart_0_buffer.rx_count = 7;
				pal_sio_tx(0, data, 4);
				bmm_buffer_free((buffer_t *)msg);
				return;
			}
#endif
			if (IS_HARD_RESET_CMD(ndr.nsdu, ndr.nsduLength) == true)
			{
				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable ); //访问之前要首先使能寄存器写

				IWDG_SetPrescaler(IWDG_Prescaler_64 ); //64分频 一个周期1.6ms
				IWDG_SetReload(1); //最长12位 [0,4096] 800*1.6=1.28S
				/* Reload IWDG counter */
				IWDG_ReloadCounter();
				IWDG_Enable(); // Enable IWDG (the LSI oscillator will be enabled by hardware)
				while (1)
					;
			}
	        AppFrameHeader->frameControl.state = APP_RESPONSE;
	        AppFrameHeader->frameControl.sendSuccess = false;
	        //AppFrameHeader->frameControl.ackReq = false;
	        //AppFrameHeader->frameControl.tries = 0;
	        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
	        //AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
	    	//pal_get_current_time(&currentTime);
	        //AppFrameHeader->travelTime = currentTime>>8;
	        //AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
	        AppFrameHeader->length = 1;
	        AppFrameHeader->AppPayload[0] = ERROR_XOR_ERROR;
	        AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu, 8);
			pal_sio_tx(SIO_0, ndr.nsdu, 9);

			bmm_buffer_free((buffer_t *)msg);
			return;
		}
#ifndef REDUCE_CHECK_PARMER
		else if (ndr.nsduLength == APP_HEAD_LENGTH)	        //无线发送的数据长度为0，没有意义
	    {
	        AppFrameHeader->frameControl.state = APP_RESPONSE;
	        AppFrameHeader->frameControl.sendSuccess = false;
	        //AppFrameHeader->frameControl.ackReq = false;
	        //AppFrameHeader->frameControl.tries = 0;
	        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
	        //AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
	    	//pal_get_current_time(&currentTime);
	        //AppFrameHeader->travelTime = currentTime>>8;
	        //AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
	        AppFrameHeader->length = 1;
	        AppFrameHeader->AppPayload[0] = ERROR_PAYLOAD_LENGTH_0;
	        AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu, 8);
			pal_sio_tx(SIO_0, ndr.nsdu, 9);

			bmm_buffer_free((buffer_t *)msg);
			return;
	    }
	    else if (AppFrameHeader->frameControl.frameType == APP_CMD && AppFrameHeader->AppPayload[0] >= APP_CMD_LAST)
	    {
			AppFrameHeader->frameControl.state = APP_RESPONSE;
			AppFrameHeader->frameControl.sendSuccess = false;
			//AppFrameHeader->frameControl.ackReq = false;
			//AppFrameHeader->frameControl.tries = 0;
			AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
			//AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
			//pal_get_current_time(&currentTime);
			//AppFrameHeader->travelTime = currentTime>>8;
			//AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;

			AppFrameHeader->length = 2;
			AppFrameHeader->AppPayload[1] = ERROR_CMD_NO_SUPPORT;
			AppFrameHeader->AppPayload[2] = XORSUM(ndr.nsdu, 8);
			pal_sio_tx(SIO_0, ndr.nsdu, 10);

			bmm_buffer_free((buffer_t *) msg);
			return;
	    }
	    else if (ndr.dstAddr == gNwk_nib.networkAddress && (AppFrameHeader->frameControl.frameType == APP_DATA || AppFrameHeader->AppPayload[0] > APP_CMD_REPORT_NEIGHBOR))
	    {
	        AppFrameHeader->frameControl.state = APP_RESPONSE;
	        AppFrameHeader->frameControl.sendSuccess = false;
	        //AppFrameHeader->frameControl.ackReq = false;
	        //AppFrameHeader->frameControl.tries = 0;
	        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
	        //AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
	    	//pal_get_current_time(&currentTime);
	        //AppFrameHeader->travelTime = currentTime>>8;
	        //AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
	        if (AppFrameHeader->frameControl.frameType == APP_DATA)
	        {
				AppFrameHeader->length = 1;
				AppFrameHeader->AppPayload[0] = ERROR_DEST_ERROR;
				AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu, 8);
				pal_sio_tx(SIO_0, ndr.nsdu, 9);
	        }
	        else
	        {
				AppFrameHeader->length = 2;
				AppFrameHeader->AppPayload[1] = ERROR_DEST_ERROR;
				AppFrameHeader->AppPayload[2] = XORSUM(ndr.nsdu, 8);
				pal_sio_tx(SIO_0, ndr.nsdu, 10);
	        }

			bmm_buffer_free((buffer_t *)msg);
			return;
	    }
#endif
    }

    NwkState = NWK_MODULE_ID_DATA_REQ;

#ifndef _ENDDEVICE_    
    NwkSourceRouteSubframe_t *NwkSourceRouteSubframe;
#endif    
    gNwk_conf_buf_ptr = msg;  

#ifndef _ENDDEVICE_
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
	if (AppFrameHeader->frameControl.frameType == APP_CMD && AppFrameHeader->frameControl.state == APP_REQUEST && AppFrameHeader->DstAddress == gNwk_nib.networkAddress)
	{
		switch (AppFrameHeader->AppPayload[0])
		{
			case APP_CMD_REPORT_NEIGHBOR:
			{
				NwkLinkStatusPayload_t *NwkLinkStatus_CommandFrame;

				payload_pos = (uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg);
				memcpy(payload_pos, ndr.nsdu, ndr.nsduLength);
				AppFrameHeader = (AppFrameHeader_t *)payload_pos;
				AppFrameHeader->frameControl.state = APP_RESPONSE;
				AppFrameHeader->frameControl.sendSuccess = true;
				NwkLinkStatus_CommandFrame = (NwkLinkStatusPayload_t *)(payload_pos + ndr.nsduLength - 2);

				NwkLinkStatus_CommandFrame->options.entryCount = 0;
#ifndef VANET_REDUCE_FUNC
                uint8_t i,j = 0;
				for (i=0; i<MAX_NEIGHBOR_TABLE_NUMBER; i++)
				{
					if (neighborTableArray[i].table.relationship < RELATIONSHIP_NONE_OF_ABOVE &&
						neighborTableArray[i].table.deviceType < DEV_TYPE_ENDDEVICE)
					{
						NwkLinkStatus_CommandFrame->options.entryCount ++;
						NwkLinkStatus_CommandFrame->table[j].addr = neighborTableArray[i].table.networkAddr;
						NwkLinkStatus_CommandFrame->table[j].linkStatus.incomingRssi = neighborTableArray[i].table.mutableTable.rssi;
						NwkLinkStatus_CommandFrame->table[j].linkStatus.outgoingRssi = neighborTableArray[i].table.mutableTable.outingRssi;
						j++;
					}
				}
#endif
				if (NwkLinkStatus_CommandFrame->options.entryCount > 0)
				{
					NwkLinkStatus_CommandFrame->options.firstFrame = true;
					NwkLinkStatus_CommandFrame->options.lastFrame = true;
				}
				AppFrameHeader->length += NwkLinkStatus_CommandFrame->options.entryCount*4 + 1;
				AppFrameHeader->AppPayload[AppFrameHeader->length] = XORSUM(payload_pos,AppFrameHeader->length + APP_HEAD_LENGTH-1);
				pal_sio_tx(0,payload_pos,AppFrameHeader->length + APP_HEAD_LENGTH);

				bmm_buffer_free((buffer_t *)msg);
				NwkState = NWK_MODULE_NONE;
				return;
			}
				//break;//statement is unreachable

			case APP_CMD_HARD_REBOOT:
		        AppFrameHeader->frameControl.state = APP_RESPONSE;
		        AppFrameHeader->frameControl.sendSuccess = true;

		        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
		        //AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
		    	//pal_get_current_time(&currentTime);
		        //AppFrameHeader->travelTime = currentTime>>8;
		        //AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
				AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu, 8);
				pal_sio_tx(SIO_0, ndr.nsdu, 9);
				 IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //访问之前要首先使能寄存器写


				 IWDG_SetPrescaler(IWDG_Prescaler_64);//64分频 一个周期1.6ms
				 IWDG_SetReload(1);//最长12位 [0,4096] 800*1.6=1.28S
				 /* Reload IWDG counter */
				 IWDG_ReloadCounter();
				 IWDG_Enable();// Enable IWDG (the LSI oscillator will be enabled by hardware)
				 while(1);
				//break;//statement is unreachable

			case APP_CMD_SLEEP:
			case APP_CMD_POWER_DOWN:
				comData = AppFrameHeader->AppPayload[1];
			comData = comData;
		        AppFrameHeader->frameControl.state = APP_RESPONSE;
		        AppFrameHeader->frameControl.sendSuccess = true;
		        AppFrameHeader->length = 1;
		        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
		        //AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
		    	//pal_get_current_time(&currentTime);
		        //AppFrameHeader->travelTime = currentTime>>8;
		        //AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
				AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu, 8);
				for (uint8_t i=0; i<9; i++)
				{
					while(USART_GetFlagStatus(UART0_BASE, USART_FLAG_TXE) == RESET);
			    	WRITE_TO_UART_0(ndr.nsdu[i]);
					while(USART_GetFlagStatus(UART0_BASE, USART_FLAG_TC) == RESET);
				}
				bmm_buffer_free((buffer_t *)msg);
				mac_trx_init_sleep();

			    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_11;
#ifdef  STM32F4XX
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;        
#else
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
#endif
#ifdef STM32F10X_MD            
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#elif defined (STM32F051) || defined (STM32F4XX)
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#endif   
			    GPIO_Init(GPIOA, &GPIO_InitStructure);

			    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
#ifdef  STM32F4XX
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;        
#else
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
#endif
            
#ifdef STM32F10X_MD            
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#elif defined (STM32F051) || defined (STM32F4XX)
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#endif   
			    GPIO_Init(GPIOB, &GPIO_InitStructure);

//			    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2|GPIO_Pin_14;
//			    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//			    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//			    GPIO_Init(GPIOA, &GPIO_InitStructure);
//			    GPIO_SetBits(GPIOA,GPIO_Pin_2);
//			    GPIO_ResetBits(GPIOA,GPIO_Pin_14);
				GPIO_SetBits(GPIOA,GPIO_Pin_4);
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
				GPIO_ResetBits(GPIOA,GPIO_Pin_7);

				GPIO_ResetBits(GPIOA,GPIO_Pin_9);
				GPIO_ResetBits(GPIOA,GPIO_Pin_11);

				GPIO_ResetBits(GPIOB,GPIO_Pin_2);
			    pal_led(LED_START, LED_OFF);         // indicating application is started
			    pal_led(LED_NWK_SETUP, LED_OFF);    // indicating network is started
			    pal_led(LED_DATA, LED_OFF);         // indicating data reception
				if (AppFrameHeader->AppPayload[0] == APP_CMD_SLEEP)
				{
					RTC_EXTI_INITIAL(ENABLE);
					RTC_SET_ALARM(comData);
					PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
#ifdef STM32F10X_MD            
	    					RTC_ITConfig(RTC_IT_ALR,DISABLE);
#elif defined (STM32F051) || defined (STM32F4XX)

#endif                                           

				}
				else
				{
#ifndef  STM32F4XX                   
				    EXTI_InitTypeDef EXTI_InitStructure;

				//------------EXTI 配置 -------------------
				    EXTI_ClearITPendingBit(EXTI_Line3);
				    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
				    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
				    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
				    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
				    EXTI_Init(&EXTI_InitStructure);
				    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
					PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);
				    EXTI_ClearITPendingBit(EXTI_Line3);
				    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
				    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
				    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
				    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
				    EXTI_Init(&EXTI_InitStructure);
#endif                    
				}
                                extern void SystemInit (void);
				SystemInit();
			    /* Initialising PIO pins */
			    gpio_init();

			    /* Initialising PIO interrupts with Zero prority*/
			    pal_pio_initialize_interrupts(1);

			    /* Initialising tranceiver interface */
			    trx_interface_init();

			    /* Initialising timer for PAL */
			    timer_init();
				mac_trx_wakeup();
			    pal_global_irq_enable();


			    if (pal_sio_init(0) != MAC_SUCCESS)
			    {
			        /* Something went wrong during initialization. */
			        pal_alert();
			    }

			    mac_state = MAC_PAN_COORD_STARTED;
			    mac_rx_enable();
#ifndef _ENDDEVICE_    
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
    nwkStartLinkStatusTimer();
            }
#endif 

			   // init_sio();


				comData = 0x55;
				pal_sio_tx(0,&comData,1);
				NwkState = NWK_MODULE_NONE;
				return;
				//break;//statement is unreachable
//            case APP_CMD_GET_TAG_TABLE://TODO add deal
//
//                return;
//            	break;
			default :
		        AppFrameHeader->frameControl.state = APP_RESPONSE;
		        AppFrameHeader->frameControl.sendSuccess = false;
		        AppFrameHeader->DstAddress = gNwk_nib.networkAddress;
				AppFrameHeader->length = 2;
				AppFrameHeader->AppPayload[1] = ERROR_CMD_NO_SUPPORT;
				AppFrameHeader->AppPayload[2] = XORSUM(ndr.nsdu, 8);
				pal_sio_tx(SIO_0, ndr.nsdu, 10);
				bmm_buffer_free((buffer_t *)msg);
				NwkState = NWK_MODULE_NONE;
				return;
				//break;//statement is unreachable

		}

	}
	else if (AppFrameHeader->DstAddress >= BROADCAST_ADDR_ROUTERS || ndr.dstAddrMode == NWK_DSTADDRMODE_NOADDR)
	{
#ifdef _NWK_PASSIVE_ACK_
		retval_t timer_status;
		uint8_t index;
		bool restart=false;
		gNwk_nib.transmitCounter ++;
//		if (ndr.routeType != APP_ROUTE_BROADCAST)
//		{
//			payload_pos = ((uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg)) + (LARGE_BUFFER_SIZE - FCS_LEN - ndr.nsduLength);
//			memcpy(payload_pos, ndr.nsdu, ndr.nsduLength);
//		}
//		else
		{
			payload_pos = ndr.nsdu;
		}
		AppFrameHeader = (AppFrameHeader_t *)payload_pos;

		payload_pos--;
		if (ndr.routeType != APP_ROUTE_BROADCAST)
		*payload_pos = gNwk_nib.sequenceNumber++;
		else *payload_pos = ndr.nonmemberRadius;//等于转发的网络层序列号

		payload_pos--;
		*payload_pos = ndr.radius;

		payload_pos -= 2;
		if (ndr.routeType != APP_ROUTE_BROADCAST)
		convert_16_bit_to_byte_array(gNwk_nib.networkAddress, payload_pos);
		else convert_16_bit_to_byte_array(ndr.dstAddr, payload_pos);

		payload_pos -= 2;
		convert_16_bit_to_byte_array(0xFFFF, payload_pos);

		payload_pos -= 2;

		NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
		NwkFrameHeader->frameControl.frameType = NWK_FRAMETYPE_DATA;
		NwkFrameHeader->frameControl.protocolVersion = gNwk_nib.protocolVersion;
		NwkFrameHeader->frameControl.discoverRoute = ndr.discoverRoute;
		NwkFrameHeader->frameControl.multicastFlag = false;
		NwkFrameHeader->frameControl.security = false;//ndr.securityEnable;
		NwkFrameHeader->frameControl.sourceRoute = false;
		NwkFrameHeader->frameControl.dstExtAddr = false;
		NwkFrameHeader->frameControl.srcExtAddr = false;
	  //  NwkFrameHeader->field.src.srcExt.value = tal_pib_IeeeAddress;

		mdr.SrcAddrMode = WPAN_ADDRMODE_SHORT;
		mdr.DstAddrMode = WPAN_ADDRMODE_SHORT;
		mdr.DstPANId    = gNwk_nib.panId;
		mdr.DstAddr     = 0xFFFF;
		mdr.msduHandle  = *(uint8_t *)(ndr.nsdu);
		mdr.TxOptions   = WPAN_TXOPT_OFF;
		mdr.msduLength  = ndr.nsduLength + NWK_DATA_FRAME_HEADER_LENGTH;
		mdr.msdu        = payload_pos;

		/* Now all is fine, continue... */
		transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER((buffer_t *)msg);

		transmit_frame->AppFrameHeader = (AppFrameHeader_t *)AppFrameHeader;
		transmit_frame->NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
		/* Store the message type */
		if (ndr.routeType == APP_ROUTE_BROADCAST)
			transmit_frame->msg_type = NWK_DATA_BROADCAST_RELAY_N;//主动发起的要延时一个周期恢复nwkstate，转发的一直nwkstate=none
		else transmit_frame->msg_type = NWK_DATA_BROADCAST_N;

		transmit_frame->msduHandle = mdr.msduHandle;

#if (MAC_INDIRECT_DATA_FFD == 1)
		/* Indirect transmission not ongoing yet. */
		transmit_frame->indirect_in_transit = false;
#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */
		if (ndr.routeType != APP_ROUTE_BROADCAST)
			status = build_data_frame(&mdr, transmit_frame, 1);
		else
			status = build_data_frame(&mdr, transmit_frame, 2);

		if (MAC_SUCCESS != status)
		{
            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free(transmit_frame->buffer_header);
            mac_sleep_trans();
			return;
		}

		//mac_trx_wakeup();

		transmit_frame->buffer_header = (buffer_t *)msg;

		nwk_frame_ptr = transmit_frame;

		if (ndr.routeType != APP_ROUTE_BROADCAST)
		{
			if (nwkNewPassiveAck(transmit_frame, gNwk_nib.networkAddress, gNwk_nib.networkAddress, transmit_frame->NwkFrameHeader->sequenceNumber) == false)
			{
				NwkState = NWK_MODULE_NONE;
				bmm_buffer_free(transmit_frame->buffer_header);
				mac_sleep_trans();
				return;
			}
		}

		index  = nwkFindPassiveAck(transmit_frame->NwkFrameHeader->srcAddr, transmit_frame->NwkFrameHeader->sequenceNumber);
		if (index < 3)
		{
			timer_status =
				pal_timer_start(gNwkPassiveAckTable.table[index].timerID,
								NWK_PASSIVE_ACK_TIMEOUT,//网络层ack最小延时500ms
								TIMEOUT_RELATIVE,
	#ifndef VANET
								(FUNC_PTR)bc_data_cb,
	#else
								(FUNC_PTR)bc_data_cb_vanet,
	#endif
								transmit_frame);

			if (MAC_SUCCESS != timer_status)
			{
				NwkState = NWK_MODULE_NONE;
				bmm_buffer_free(transmit_frame->buffer_header);
				mac_sleep_trans();
				return;
			}
			else
			{
	//			timer_status =
	//				pal_timer_start(APP_TIMER_BC_DATA,
	//								NWK_PASSIVE_ACK_TIMEOUT,//网络层ack最小延时500ms
	//								TIMEOUT_RELATIVE,
	//								(FUNC_PTR)bc_data_cb,
	//								transmit_frame);
	//
	//			if (MAC_SUCCESS != timer_status)
	//			{
	//	            NwkState = NWK_MODULE_NONE;
	//	            bmm_buffer_free(transmit_frame->buffer_header);
	//	            mac_sleep_trans();
	//			}

			}
		}
		else
		{
			NwkState = NWK_MODULE_NONE;
			bmm_buffer_free(transmit_frame->buffer_header);
			mac_sleep_trans();
			return;
		}

		 if (ndr.routeType == APP_ROUTE_BROADCAST)
		 {
            if (broadDelayCount > 2)
            {
                NwkState = NWK_MODULE_NONE;
                //bmm_buffer_free(transmit_frame->buffer_header);
                mac_sleep_trans();
                return;              
            }
              
			uint32_t timeInterval;
			timeInterval = (rand()%NWK_MAX_BROADCAST_JITTER)+1;//rand()%
			//pal_timer_delay(timeInterval*1000);

			index = __CLZ(__RBIT(~broadDelayBitmap));
			//if (index>=NWK_MAX_BROADCASR_TABLE_ENTRY)
			//	return;
			broadDelayBitmap |= 1<<index;
			nwkBroadDelay[index].delayCount = pal_add_time_us(gettime(), timeInterval*1000);
			nwkBroadDelay[index].transmit_frame = transmit_frame;
			broadDelayCount++;
			if (broadDelayCount > 1)
			{
				restart = bubble_sort_broad(index);
			}
			if (broadDelayCount == 1 || restart == true)
			{
				minBroadDelayIndex = 0;
				if (restart == true)
					pal_timer_stop(APP_TIMER_UART_TIMEOUT);

                do
                {
                    timer_status =
                        pal_timer_start(APP_TIMER_UART_TIMEOUT,
                                        nwkBroadDelay[minBroadDelayIndex].delayCount,
                                        TIMEOUT_ABSOLUTE,
                                        (FUNC_PTR)bc_delay_cb,
                                        NULL);
                    if (MAC_SUCCESS != timer_status)
                    {
                        broadDelayBitmap &= ~(1<<minBroadDelayIndex);
                        nwkBroadDelay[minBroadDelayIndex].transmit_frame = NULL;
                        nwkBroadDelay[minBroadDelayIndex].delayCount = 0xFFFFFFFF;
                        broadDelayCount--;
                        minBroadDelayIndex++;
                    }

                } while (MAC_SUCCESS != timer_status && nwkBroadDelay[minBroadDelayIndex].transmit_frame != NULL);
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
            NwkState = NWK_MODULE_NONE;
			return;
		 }

		status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

		if (MAC_SUCCESS == status)
		{
			MAKE_MAC_BUSY();
		}
		else
		{
            NwkState = NWK_MODULE_NONE;
            //bmm_buffer_free(transmit_frame->buffer_header);
            mac_sleep_trans();
            return;
		}


#else
        NwkState = NWK_MODULE_NONE;
        bmm_buffer_free((buffer_t *)msg);
        mac_sleep_trans();
#endif
		return;
	}
#ifdef  END_DEVICE_SUPPORT
	else if (ndr.dstAddrMode != NWK_DSTADDRMODE_RESPONSE)
	{
		NwkNeighbor_t *tempNwkNeighbor;
	    uint8_t src_addr_mode;
	    wpan_addr_spec_t dst_addr;

		tempNwkNeighbor = NWK_FindNeighborByShortAddr(AppFrameHeader->DstAddress);
		if (tempNwkNeighbor->deviceType == DEVICE_TYPE_END_DEVICE && tempNwkNeighbor != NULL && tempNwkNeighbor->relationship == RELATIONSHIP_CHILD && tempNwkNeighbor->rxOnWhenIdle == false)
		{
	        src_addr_mode = WPAN_ADDRMODE_SHORT;
	        dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
	        dst_addr.PANId = tal_pib.PANId;
	        dst_addr.Addr.short_address = AppFrameHeader->DstAddress;

			payload_pos = ((uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg)) + (LARGE_BUFFER_SIZE - FCS_LEN - ndr.nsduLength);
			memcpy(payload_pos, ndr.nsdu, ndr.nsduLength);

			payload_pos--;
			*payload_pos = gNwk_nib.sequenceNumber++;

			payload_pos--;
			*payload_pos = ndr.radius;

			payload_pos -= 2;
			convert_16_bit_to_byte_array(gNwk_nib.networkAddress, payload_pos);

			payload_pos -= 2;
			convert_16_bit_to_byte_array(ndr.dstAddr, payload_pos);

			payload_pos -= 2;

			NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
			NwkFrameHeader->frameControl.frameType = NWK_FRAMETYPE_DATA;
			NwkFrameHeader->frameControl.protocolVersion = gNwk_nib.protocolVersion;
			NwkFrameHeader->frameControl.discoverRoute = ndr.discoverRoute;
			NwkFrameHeader->frameControl.multicastFlag = false;
			NwkFrameHeader->frameControl.security = false;//ndr.securityEnable;
			NwkFrameHeader->frameControl.sourceRoute = false;
			NwkFrameHeader->frameControl.dstExtAddr = false;
			NwkFrameHeader->frameControl.srcExtAddr = false;

	        if (!wpan_mcps_data_req(src_addr_mode,////enddevice 的父亲间接传输给孩子
	                                &dst_addr,
	                                ndr.nsduLength + NWK_DATA_FRAME_HEADER_LENGTH,  // One octet
	                                payload_pos,
	                                mac_pib.mac_DSN,
	                                WPAN_TXOPT_INDIRECT_ACK))
	        {
	            /*
	             * Data could not be queued into the indirect queue.
	             * Add error handling if required.
	             */
	        }

            if (AppFrameHeader->frameControl.state == APP_REQUEST &&  AppFrameHeader->frameControl.ackReq == true)
            {
				AppFrameHeader->frameControl.state = APP_RESPONSE;
				AppFrameHeader->frameControl.sendSuccess = true;
				AppFrameHeader->frameControl.ackReq = false;
				AppFrameHeader->frameControl.tries = 0;
#ifndef  REDUCE_CHECK_PARMER3
				pal_get_current_time(&currentTime);
				AppFrameHeader->travelTime = currentTime>>8;
				AppFrameHeader->hops = 0;
#endif
				if (AppFrameHeader->frameControl.frameType == APP_DATA)
				{
					AppFrameHeader->length = 0;
					AppFrameHeader->AppPayload[0] = XORSUM(ndr.nsdu,7);
				}
				else
				{
					AppFrameHeader->length = 1;
					AppFrameHeader->AppPayload[1] = XORSUM(ndr.nsdu,8);
				}
				pal_sio_tx(SIO_0, ndr.nsdu, AppFrameHeader->frameControl.frameType ? 8:9);
            }

            NwkState = NWK_MODULE_NONE;
            bmm_buffer_free((buffer_t *)msg);
            mac_sleep_trans();
			return;
		}
	}
#endif
		switch (ndr.routeType)
		{
			case	APP_ROUTE_MESH:
					goto route_mesh;
				//break;//statement is unreachable

			case	APP_ROUTE_TREE:
					goto route_tree;
				//break;//statement is unreachable

			case	APP_ROUTE_SOURCE_PC:
					gNwk_nib.transmitCounter ++;
					payload_pos = ((uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg)) + (LARGE_BUFFER_SIZE - FCS_LEN - ndr.nsduLength + AppFrameHeader->frameControl.sourceCount);
					memcpy(payload_pos, ndr.nsdu, ndr.nsduLength-AppFrameHeader->frameControl.sourceCount-1);
					payload_pos[ndr.nsduLength-AppFrameHeader->frameControl.sourceCount-1] = ndr.nsdu[ndr.nsduLength-1];
					AppFrameHeader = (AppFrameHeader_t *)payload_pos;
					AppFrameHeader->length -= AppFrameHeader->frameControl.sourceCount;
					AppFrameHeader->frameControl.routeType = APP_ROUTE_MESH;

					payload_pos -= AppFrameHeader->frameControl.sourceCount;
					memcpy(payload_pos, ndr.nsdu + ndr.nsduLength - AppFrameHeader->frameControl.sourceCount-1, AppFrameHeader->frameControl.sourceCount);

					AppFrameHeader->frameControl.sourceCount = 0;
					AppFrameHeader->AppPayload[AppFrameHeader->length] = XORSUM((uint8_t *)AppFrameHeader, AppFrameHeader->length+APP_HEAD_LENGTH-1);

					NwkSourceRouteSubframe = (NwkSourceRouteSubframe_t *)payload_pos;

					payload_pos--;
					*payload_pos = gNwk_nib.sequenceNumber++;

					payload_pos--;
					*payload_pos = ndr.radius;

					payload_pos -= 2;
					convert_16_bit_to_byte_array(gNwk_nib.networkAddress, payload_pos);

					payload_pos -= 2;
					convert_16_bit_to_byte_array(ndr.dstAddr, payload_pos);

					payload_pos -= 2;

					NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
					NwkFrameHeader->frameControl.frameType = NWK_FRAMETYPE_DATA;
					NwkFrameHeader->frameControl.protocolVersion = gNwk_nib.protocolVersion;
					NwkFrameHeader->frameControl.discoverRoute = ndr.discoverRoute;
					NwkFrameHeader->frameControl.multicastFlag = false;
					NwkFrameHeader->frameControl.security = ndr.securityEnable;
					NwkFrameHeader->frameControl.sourceRoute = false;
					NwkFrameHeader->frameControl.dstExtAddr = false;
					NwkFrameHeader->frameControl.srcExtAddr = false;
					NwkFrameHeader->frameControl.sourceRoute = true;

					mdr.SrcAddrMode = WPAN_ADDRMODE_SHORT;
					mdr.DstAddrMode = WPAN_ADDRMODE_SHORT;
					mdr.DstPANId    = gNwk_nib.panId;
					mdr.DstAddr     = NwkSourceRouteSubframe->relayList[NwkSourceRouteSubframe->relayIndex];
					mdr.msduHandle  = *(uint8_t *)(ndr.nsdu);
					mdr.TxOptions   = WPAN_TXOPT_ACK;
					mdr.msduLength  = ndr.nsduLength + NWK_DATA_FRAME_HEADER_LENGTH;
					mdr.msdu        = payload_pos;

					/* Now all is fine, continue... */
					transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER((buffer_t *)msg);

					transmit_frame->AppFrameHeader = (AppFrameHeader_t *)AppFrameHeader;
					transmit_frame->NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
					/* Store the message type */
					if (AppFrameHeader->frameControl.state == APP_REQUEST)
						transmit_frame->msg_type = NWK_DATA_REQUEST_N;
					else transmit_frame->msg_type = NWK_DATA_RESPONSE_N;

					transmit_frame->msduHandle = mdr.msduHandle;

				#if (MAC_INDIRECT_DATA_FFD == 1)
					/* Indirect transmission not ongoing yet. */
					transmit_frame->indirect_in_transit = false;
				#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */

					status = build_data_frame(&mdr, transmit_frame, 2);

					if (MAC_SUCCESS != status)
					{
						gNwk_nib.transmitFailureCounter ++;
						/* The frame is too long. */
						nwk_gen_nlde_data_conf((buffer_t *)msg,
											   (uint8_t)status,
											   ndr.nsdu,0,0);
						mac_sleep_trans();
						return;
					}

						//mac_trx_wakeup();

					transmit_frame->buffer_header = (buffer_t *)msg;

					 nwk_frame_ptr = transmit_frame;
					/* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
					status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

					if (MAC_SUCCESS == status)
					{
						MAKE_MAC_BUSY();
					}
					else
					{
						gNwk_nib.transmitFailureCounter ++;
						nwk_gen_nlde_data_conf((buffer_t *)msg,
											   (uint8_t)status,
											   ndr.nsdu,0,0);
						mac_sleep_trans();
					}
					return;
				//break;//statement is unreachable

//			case	APP_ROUTE_SOURCE_TABLE:
//				break;
//
//			case	APP_ROUTE_SOURCE_ASSIGN:
//				break;
//
//			case	APP_ROUTE_NONE:
//				break;

			default:
					nwk_gen_nlde_data_conf((buffer_t *)msg,
											ERROR_NO_ROUTING_CAPACITY,
										   ndr.nsdu,0,0);
					mac_sleep_trans();
					return;
				//break;//statement is unreachable
		}


route_mesh:
		tempRoutingTableEntry = nwkFindRoutingEntry(ndr.dstAddr, false);

		if (tempRoutingTableEntry != NULL && tempRoutingTableEntry->status == ACTIVE)
		{
#ifdef _NWK_ROUTE_CACHE_
			if (tempRoutingTableEntry->manyToOne == true && tempRoutingTableEntry->routeRecordRequired == true)
			{
				tempRoutingTableEntry->routeRecordRequired = false;
				wpan_nlme_RouteRecord_CommandFrame_req(ndr.dstAddr, NULL);
			}
#endif

			N = tempRoutingTableEntry->nextHopAddr;
		}
		else
		{
			if (ndr.discoverRoute == true)
			{
				if (ndr.dstAddrMode != NWK_DSTADDRMODE_RESPONSE)
				{
					NWK_DataReq_t * tempNdr = (NWK_DataReq_t *)BMM_BUFFER_POINTER((buffer_t *)msg);
					tempNdr->dstAddrMode = NWK_DSTADDRMODE_RESPONSE;
					payload_pos = ((uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg)) + (LARGE_BUFFER_SIZE - FCS_LEN - ndr.nsduLength);
					memcpy(payload_pos, ndr.nsdu, ndr.nsduLength);
					tempNdr->nsdu = payload_pos;
				}
				wpan_nlme_RouteDiscovery_req(ndr.dstAddrMode, ndr.dstAddr, ndr.radius, true, (buffer_t *)msg, NULL);
				NwkState = NWK_MODULE_NONE;
				//cloud_test3 ++;
				return;
			}
			else
			{

				if (gNwk_nib.addrAlloc == NWK_ADDR_ALLOC_DISTRIBUTED)//路由方法的选择？
				{
route_tree:
					N = Find_NextHopAddress_InTree(ndr.dstAddr);
				}
				else//等待中心节点3次没收到回应，来重新发现路由
				{
					nwk_gen_nlde_data_conf((buffer_t *)msg,
							ERROR_NO_ROUTING_CAPACITY,
										   ndr.nsdu,0,0);
					mac_sleep_trans();
					return;
				}

			}
		}
            }
#ifndef REDUCE_CHECK_PARMER1
        	else
        	{
        		N = gNwk_nib.parentNetworkAddress;
        	}
#endif
#else
                N = gNwk_nib.parentNetworkAddress;
#endif                
		gNwk_nib.transmitCounter ++;
		if (ndr.dstAddrMode != NWK_DSTADDRMODE_RESPONSE)
		{
			payload_pos = ((uint8_t *)BMM_BUFFER_POINTER((buffer_t *)msg)) + (LARGE_BUFFER_SIZE - FCS_LEN - ndr.nsduLength);
			memcpy(payload_pos, ndr.nsdu, ndr.nsduLength);
		}
		else
		{
			payload_pos = ndr.nsdu;
		}
		AppFrameHeader = (AppFrameHeader_t *)payload_pos;

		payload_pos--;
		*payload_pos = gNwk_nib.sequenceNumber++;

		payload_pos--;
		*payload_pos = ndr.radius;

		payload_pos -= 2;
		convert_16_bit_to_byte_array(gNwk_nib.networkAddress, payload_pos);

		payload_pos -= 2;
		convert_16_bit_to_byte_array(ndr.dstAddr, payload_pos);

		payload_pos -= 2;

		NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
		NwkFrameHeader->frameControl.frameType = NWK_FRAMETYPE_DATA;
		NwkFrameHeader->frameControl.protocolVersion = gNwk_nib.protocolVersion;
		NwkFrameHeader->frameControl.discoverRoute = ndr.discoverRoute;
		NwkFrameHeader->frameControl.multicastFlag = false;
		NwkFrameHeader->frameControl.security = false;//ndr.securityEnable;
		NwkFrameHeader->frameControl.sourceRoute = false;
		NwkFrameHeader->frameControl.dstExtAddr = false;
		NwkFrameHeader->frameControl.srcExtAddr = false;
	  //  NwkFrameHeader->field.src.srcExt.value = tal_pib_IeeeAddress;

		mdr.SrcAddrMode = WPAN_ADDRMODE_SHORT;
		mdr.DstAddrMode = WPAN_ADDRMODE_SHORT;
		mdr.DstPANId    = gNwk_nib.panId;
		mdr.DstAddr     = N;
		mdr.msduHandle  = *(uint8_t *)(ndr.nsdu);
		mdr.TxOptions   = WPAN_TXOPT_ACK;
		mdr.msduLength  = ndr.nsduLength + NWK_DATA_FRAME_HEADER_LENGTH;
		mdr.msdu        = payload_pos;

		/* Now all is fine, continue... */
		transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER((buffer_t *)msg);

		transmit_frame->AppFrameHeader = (AppFrameHeader_t *)AppFrameHeader;
		transmit_frame->NwkFrameHeader = (NwkFrameHeader_t *)payload_pos;
		/* Store the message type */
		if (AppFrameHeader->frameControl.state == APP_REQUEST)
			transmit_frame->msg_type = NWK_DATA_REQUEST_N;
		else transmit_frame->msg_type = NWK_DATA_RESPONSE_N;

		transmit_frame->msduHandle = mdr.msduHandle;

#if (MAC_INDIRECT_DATA_FFD == 1)
		/* Indirect transmission not ongoing yet. */
		transmit_frame->indirect_in_transit = false;
#endif  /* (MAC_INDIRECT_DATA_FFD == 1) */
		if (ndr.securityEnable >= XOR_PASS)
			status = build_data_frame(&mdr, transmit_frame, 1);
		else
			status = build_data_frame(&mdr, transmit_frame, 2);

		if (MAC_SUCCESS != status)
		{
  		    gNwk_nib.transmitFailureCounter ++;
			/* The frame is too long. */
			nwk_gen_nlde_data_conf((buffer_t *)msg,
								   (uint8_t)status,
								   ndr.nsdu,0,0);
			mac_sleep_trans();
			return;
		}

		//mac_trx_wakeup();

		transmit_frame->buffer_header = (buffer_t *)msg;

		 nwk_frame_ptr = transmit_frame;
		/* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
		status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

		if (MAC_SUCCESS == status)
		{
			MAKE_MAC_BUSY();
		}
		else
		{
			gNwk_nib.transmitFailureCounter ++;
			nwk_gen_nlde_data_conf((buffer_t *)msg,
								   (uint8_t)status,
								   ndr.nsdu,0,0);
			mac_sleep_trans();
		}

}

#ifdef dataReqContinue
void app_t_DataRequest_cb(frame_info_t *transmit_framea)
{
    buffer_t *buffer_ack_frame = NULL;
    frame_info_t *transmit_frame;
    static uint8_t first = 2;

    if (first == 1)
    {
		first = 0;
    	buffer_ack_frame = qmm_queue_read(&wait_nwk_ack_q, NULL);
	 transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER(buffer_ack_frame); 	
		while (buffer_ack_frame != NULL)
		{
		 
                  mac_trx_wakeup();
			retval_t status = FAILURE;

			/* In Nonbeacon build the frame is sent with unslotted CSMA-CA. */
			while (tal_state != TAL_IDLE);
			status = tal_tx_frame(transmit_frame, CSMA_UNSLOTTED, true);

			if (MAC_SUCCESS == status)
			{
				MAKE_MAC_BUSY();
			}
			else
			{
				nwk_gen_nlde_data_conf(transmit_frame->buffer_header,
									   (uint8_t)status,
									   (void *)transmit_frame->AppFrameHeader,0,0);
				mac_sleep_trans();
			}
			buffer_ack_frame = buffer_ack_frame->next;
			transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER(buffer_ack_frame);
		}
    }
    else
    {
    	first = 2;
    	buffer_ack_frame = qmm_queue_remove(&wait_nwk_ack_q, NULL);
		transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER(buffer_ack_frame);
		while (buffer_ack_frame != NULL)
		{

				nwk_gen_nlde_data_conf(transmit_frame->buffer_header,
									   NWK_ROUTE_ERROR_STATUS,
									   (void *)transmit_frame->AppFrameHeader,0,0);
				mac_sleep_trans();

                  buffer_ack_frame = qmm_queue_remove(&wait_nwk_ack_q, NULL);
                  transmit_frame = (frame_info_t *)BMM_BUFFER_POINTER(buffer_ack_frame);
		}
    }
}


#else
#ifndef _ENDDEVICE_
void app_t_DataRequest_cb(frame_info_t *transmit_frame)
{
	NwkRoutingTableEntry_t    *NwkRoutingTableEntry;

	if (pal_is_timer_running(APP_TIMER_REQUEST_DATA) == true)
		pal_timer_stop(APP_TIMER_REQUEST_DATA);

	NwkRoutingTableEntry =  nwkFindRoutingEntry(transmit_frame->mpdu[12] | (transmit_frame->mpdu[13]<<8), false);
	if (NwkRoutingTableEntry != NULL && NwkRoutingTableEntry->status == ACTIVE)
	{
		NwkRoutingTableEntry->failures++;
		if (NwkRoutingTableEntry->failures >= NWK_ROUTE_FAIL_TRIES)
		{
			wpan_nlme_RouteDiscovery_req(NWK_DSTADDRMODE_SHORT, transmit_frame->mpdu[12] | (transmit_frame->mpdu[13]<<8), NWK_MAX_HOPS, true, NULL, NULL);
			NwkRoutingTableEntry->failures = 0;
			//NwkRoutingTableEntry->cost = 0xFFFF;
			NwkRoutingTableEntry->status = DISCOVERY_UNDERWAY;
		}
	}


    while (transmit_frame->AppFrameHeader->frameControl.tries != 0)
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
        }
        else
        {
            nwk_gen_nlde_data_conf(transmit_frame->buffer_header,
                                   (uint8_t)status,
                                   (void *)transmit_frame->AppFrameHeader,0,0);
            mac_sleep_trans();
        } 
        return;
    
    }

	if (transmit_frame->errorStatus == NWK_ROUTE_ERROR_STATUS_N)
	{
        if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
        {
			transmit_frame->AppFrameHeader->frameControl.state = APP_RESPONSE;
			transmit_frame->AppFrameHeader->frameControl.sendSuccess = false;
			//AppFrameHeader->frameControl.ackReq = false;
			//AppFrameHeader->frameControl.tries = 0;
			transmit_frame->AppFrameHeader->DstAddress = gNwkFrameHeader_p->srcAddr;
			//AppFrameHeader->frameControl.sourceCount |= 1<<testRssi(rssi);
			//pal_get_current_time(&currentTime);
			//AppFrameHeader->travelTime = currentTime>>8;
			//AppFrameHeader->hops = NWK_MAX_HOPS-radius+1;
			if (transmit_frame->AppFrameHeader->frameControl.frameType == APP_DATA)
			{
				transmit_frame->AppFrameHeader->length = gNwkParse.payloadSize;
				memcpy(gNwkParse.payload-APP_HEAD_LENGTH+1, transmit_frame->AppFrameHeader, APP_HEAD_LENGTH-1);
				gNwkParse.payload[0] = NWK_ROUTE_ERROR_STATUS_N;
				gNwkParse.payload[gNwkParse.payloadSize] = XORSUM(gNwkParse.payload-APP_HEAD_LENGTH+1, gNwkParse.payloadSize+APP_HEAD_LENGTH-1);
				//if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
				pal_sio_tx(0, gNwkParse.payload-APP_HEAD_LENGTH+1, gNwkParse.payloadSize+APP_HEAD_LENGTH);
			}
			else
			{
				transmit_frame->AppFrameHeader->length = gNwkParse.payloadSize + 1;
				memcpy(gNwkParse.payload-APP_HEAD_LENGTH, transmit_frame->AppFrameHeader, APP_HEAD_LENGTH);
				gNwkParse.payload[0] = NWK_ROUTE_ERROR_STATUS_N;
				gNwkParse.payload[gNwkParse.payloadSize] = XORSUM(gNwkParse.payload-APP_HEAD_LENGTH, gNwkParse.payloadSize+APP_HEAD_LENGTH);
				pal_sio_tx(0, gNwkParse.payload-APP_HEAD_LENGTH, gNwkParse.payloadSize+APP_HEAD_LENGTH+1);
			}
        }



        bmm_buffer_free(transmit_frame->buffer_header);
        NwkState = NWK_MODULE_NONE;
	}
	else
    nwk_gen_nlde_data_conf(transmit_frame->buffer_header,
    						transmit_frame->errorStatus,
                           (void *)transmit_frame->AppFrameHeader,0,0);
    mac_sleep_trans();
}
#endif
#endif





void nwk_gen_nlde_data_conf(buffer_t *buf_ptr,
                            uint8_t status,
                            void *handle,
                            uint32_t timestamp,
                            uint8_t hops)
{
    NWK_DataConf_t *data_conf = (NWK_DataConf_t *)BMM_BUFFER_POINTER(buf_ptr);
    
    data_conf->cmdcode = NLDE_DATA_CONFIRM;
    data_conf->status = (NWK_Status_t)status;
    data_conf->nsduHandle = handle;//problem buf_ptr==handle
    data_conf->txTime = timestamp;
    data_conf->hops = hops;
    
    qmm_queue_append(&nwk_aps_q, buf_ptr); 
}              
