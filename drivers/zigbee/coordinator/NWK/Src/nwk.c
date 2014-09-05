/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
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
#endif  /* MAC_SECURITY_ZIP */

#include "nwk_msg_const.h"
#include "nwk_api.h"
#include "nwk_msg_types.h"
#include "nwk_internal.h"
#include "nwk.h"
#include "nwkStateMachine.h"
#include "nwk_config.h"

#include "nwkNeighbor.h"
#include "nwkPassiveAck.h"
#include "app_uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Globals variables ---------------------------------------------------------*/
//bool nwk_busy;
NwkState_t NwkState = NWK_MODULE_NONE;

NwkPermitJoiningState_t NwkPermitJoiningState    = NWK_PERMIT_JOINING_IDLE_STATE;
NwkFormationState_t     NwkFormationState        = NWK_FORMATION_IDLE_STATE;
NwkDiscoveryState_t     NwkNetworkDiscoveryState = NWK_DISCOVERY_IDLE_STATE; 
NwkJoinReqState_t       NwkJoinReqState          = JOIN_REQ_IDLE_STATE;
NwkJoinIndObjState_t    NwkJoinIndState          = NWK_JOIN_IND_FREE_STATE;
NwkStartRouterState_t   NwkStartRouterState      = NWK_START_ROUTER_IDLE_STATE;
NwkLeaveState_t         NwkLeaveReqState         = NWK_LEAVE_IDLE_STATE;
NwkLinkStatusState_t    NwkLinkStatusState       = NWK_LINK_STATUS_IDLE_STATE;
NwkRouteDiscoveryState_t NwkRouteDiscoveryState  = NWK_ROUTE_DISCOVERY_IDLE_STATE;
NwkRouteRecordState_t   NwkRouteRecordState      = NWK_ROUTE_RECORD_IDLE_STATE;
NwkStateOfStatusSender_t NwkStatusState          = NWK_STATUS_SENDER_IDLE_STATE;


uint8_t         gNwk_RouteDiscovery_sequence=0;
NwkAddrMap_t    gNwkAddrMap[MAX_NWKADDRESS_MAP_NUM];
uint32_t        gNwkAddrMap_BitMap = 0x00;


queue_t         aps_nwk_q;
queue_t         nwk_aps_q;
#ifdef dataReqContinue
queue_t         wait_nwk_ack_q;
#endif
NIB_t           gNwk_nib;

//ExtPanId_t      gExtendedPANId;
//bool            gCoordRealignment;
uint8_t         *gNwk_conf_buf_ptr;
NwkNeighbor_t   *gNeighborParent_p;
uint16_t        gJoinCoordShortAddress;

//frame_msgtype_t         gMlmeCommStatus_FrameType = (frame_msgtype_t)0;
//uint64_t                gMlmeCommStatus_DestAddr = 0;

NWK_NibAttr_t Nwk_NibAttrValue;

NwkParseHeader_t  gNwkParse;
NwkFrameHeader_t *gNwkFrameHeader_p;
frame_info_t *nwk_frame_ptr;
#ifdef UART_DMA_TX
uint8_t gResponseData[8];
#endif
uint8_t gResIndex = 0;

NwkRoutingTableEntry_t gNwkRoutingTableEntry[NWK_MAX_ROUTE_TABLE_ENTRY];
NwkRoutingTable_t      gNwkRoutingTable;

NwkRouteCacheRecord_t  gNwkRouteCacheRecordEntry[NWK_MAX_ROUTE_TABLE_ENTRY];
NwkRouteCache_t        gNwkRouteCacheTable;

NwkRouteDiscoveryEntry_t gNwkRouteDiscoveryTableEntry[NWK_MAX_ROUTE_DISCOVERY_TABLE_ENTRY];
NwkRouteDiscoveryTable_t gNwkRouteDiscoveryTable;

uint8_t gRouteRelayRetries = 0;
uint8_t gMyAddress_report[22];
uint8_t gSelectChannel = 0;
bool    gPowerUpFlag = true;
nwkSquence_t gNwkSequence[NWK_MAX_ROUTE_TABLE_ENTRY];
uint32_t currentTime;
uint8_t serialMode;

NwkPassiveAckEntry_t gNwkPassiveAckEntry[NWK_MAX_BROADCASR_TABLE_ENTRY];
NwkPassiveAck_t gNwkPassiveAckTable;

uint32_t broadDelayBitmap=0;
uint8_t minBroadDelayIndex=0;
uint8_t broadDelayCount=0;
NwkBroadDelay_t nwkBroadDelay[NWK_MAX_BROADCASR_TABLE_ENTRY];

bool uartEnableRequire = false;
BACKGROUND_CMD_PROCE  BackgroundCmdProce  = { 0xFF };
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/* Globals functions ---------------------------------------------------------*/
bool nwk_task(void)
{
    uint8_t *event = NULL;
    bool processed_event = false;

    if ((NwkState == NWK_MODULE_NONE) && (!mac_busy))
    {
        /* Check whether queue is empty */
        if (aps_nwk_q.size != 0)
        {
            event = (uint8_t *)qmm_queue_remove(&aps_nwk_q, NULL);

            /* If an event has been detected, handle it. */
            if (NULL != event)
            {
                /* Process event due to NHLE requests */
                nwk_dispatch_event(event);
                processed_event = true;
            }

        }
#ifdef SYS_OS		
		else event_processed &=~0x01;
#endif		
    }

    /*
     * Internal event queue should be dispatched
     * irrespective of the dispatcher state.
     */
    /* Check whether queue is empty */
//    if (mac_nwk_q.size != 0)
//    {
//        event = (uint8_t *)qmm_queue_remove(&mac_nwk_q, NULL);
//
//        /* If an event has been detected, handle it. */
//        if (NULL != event)
//        {
//            nwk_dispatch_event(event);
//            processed_event = true;
//        }
//    }

    return processed_event;
}

void Delay(volatile uint32_t nCount)
{
  volatile uint32_t index = 0; 
  for(index = (10000 * nCount); index != 0; index--)
  {
  }
}

uint32_t __cmsis_iar_clz(uint32_t map)//TODO 32 not 64
{
	uint32_t i;
        
        map = ~map;
        
	for (i=0; i<32; i++)
	{
		if (((map>>i) & 1) == 0)
			return i;
	}
	return 31;
}
