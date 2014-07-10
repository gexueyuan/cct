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
#include "app_uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Globals variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Global functions prototypes -----------------------------------------------*/
#ifdef NIBFromFlash
extern void FLASH_WriteTotalPageData(uint32_t addr);
#endif

#ifndef REDUCE_CHECK_PARMER2
extern  queue_t free_large_buffer_q;
#endif

/* Global functions ---------------------------------------------------------*/
//int bbb=0;
//uint32_t cloud_test2 = 0;
void usr_nlde_data_conf(uint8_t status, void *nsduHandle, 
                        uint32_t txTime, NwkRadius_t hops)
{
    NwkState = NWK_MODULE_NONE;
    static uint8_t report_my_address_failtres = 0;
    AppFrameHeader_t *tempApp = nsduHandle;

    if (status != MAC_SUCCESS)
    {
    	gNwk_nib.transmitFailureCounter ++;
		if (tempApp->frameControl.frameType == APP_CMD && tempApp->AppPayload[0]  == APP_CMD_REPORT_MY_ADDRESS)//problem 当达到3次后，应该重启，重新加入
		{
			if (report_my_address_failtres < 3)
			{
				wpan_nlde_data_req(NWK_DSTADDRMODE_SHORT,
									*(uint16_t *)&gMyAddress_report[4],
									gMyAddress_report[6]+APP_HEAD_LENGTH,
									gMyAddress_report,
									gMyAddress_report,
									NWK_MAX_HOPS,
									0,
									true,
									false,
									APP_ROUTE_MESH);
			}
			else
			{
				wpan_nlme_reset_req(true);
				report_my_address_failtres = 0;
				return;
			}
			report_my_address_failtres++;
		}
		else
		{
	        if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
	        {
				AppFrameHeader_t *AppFrameHeader = nsduHandle;
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
					AppFrameHeader->AppPayload[0] = status;
					AppFrameHeader->AppPayload[1] = XORSUM(nsduHandle, 8);
					//if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
					pal_sio_tx(SIO_0, nsduHandle, 9);
				}
				else
				{
					AppFrameHeader->length = 2;
					AppFrameHeader->AppPayload[1] = status;
					AppFrameHeader->AppPayload[2] = XORSUM(nsduHandle, 9);
					//if (GET_USER_FRAME_MODE() == USER_MIBEE_FRAME_MODE)
					pal_sio_tx(SIO_0, nsduHandle, 10);
				}
	        }
		}
    }
    else
    {//cloud_test2 ++;
		//*(uint8_t *)nsduHandle |= 0x08;//senscuccess =1
		// pal_sio_tx(SIO_0, nsduHandle, 5);
        if (tempApp->frameControl.frameType == APP_CMD && tempApp->AppPayload[0]  == APP_CMD_REPORT_MY_ADDRESS)
        {
          pal_led(LED_DATA, LED_ON);


#ifdef NIBFromFlash
         interModuleCfgInfo.AtCfgInfo.byModuleCfgFlag = 0x20002000L;
         interModuleCfgInfo.macInfo = mac_pib;
         interModuleCfgInfo.talInfo = tal_pib;
         interModuleCfgInfo.nwkInfo = gNwk_nib;
         FLASH_WriteTotalPageData(FLASH_OFFSET);
#endif
        }

        /* Start a timer that polls for pending data at the coordinator. */
#ifndef REDUCE_CHECK_PARMER2
        if (gNwk_nib.deviceType == DEVICE_TYPE_END_DEVICE && (free_large_buffer_q.size >= 0x0B || aps_nwk_q.size == 0))
        {
      	  		GPIO_InitTypeDef GPIO_InitStructure;
				mac_trx_init_sleep();

				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_11;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_Init(GPIOA, &GPIO_InitStructure);

				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_Init(GPIOB, &GPIO_InitStructure);

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
			    RTC_EXTI_INITIAL(ENABLE);
			    RTC_SET_ALARM(App_System_Para_Pointer->AtCfgInfo.byAppLocalizerTime);
			    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
        }

#endif


       // bbb++;
#ifndef _ENDDEVICE_        
#ifndef REDUCE_CHECK_PARMER1
        	if (gNwk_nib.deviceType != DEVICE_TYPE_END_DEVICE)
#endif
            {
		NwkRoutingTableEntry_t    *NwkRoutingTableEntry;
		NwkRoutingTableEntry =  nwkFindRoutingEntry((*((uint8_t *)nsduHandle+4)) | ((*((uint8_t *)nsduHandle+5))<<8), false);
		if (NwkRoutingTableEntry != NULL && NwkRoutingTableEntry->status == ACTIVE)
			NwkRoutingTableEntry->failures = 0;
            }
#endif                
    }
}
                    
