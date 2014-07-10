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

#include "nwkReset.h"
#include "nlmeReset.h"

#include "nwk1.h"
#include "nwkCommon.h"
#include "nwkIB.h"
#include "nwkConfig.h"

#include "nwkAddress.h"
#include "nwkNeighbor.h"

#include "nwkStateMachine.h"
#include "nwk.h"

#include "tal_constants.h"
#include "nwk_config.h"
#include "app_uart.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define NO_TIMER                        (0xFF)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Globals variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void reset_nwk_globals(void)
{
	uint8_t i;
    NwkState = NWK_MODULE_NONE;
    InitTable();
    NwkPermitJoiningState    = NWK_PERMIT_JOINING_IDLE_STATE;
    NwkFormationState        = NWK_FORMATION_IDLE_STATE;
    NwkNetworkDiscoveryState = NWK_DISCOVERY_IDLE_STATE; 
    NwkJoinReqState          = JOIN_REQ_IDLE_STATE;
    NwkJoinIndState          = NWK_JOIN_IND_FREE_STATE;
    NwkStartRouterState      = NWK_START_ROUTER_IDLE_STATE;
    NwkLeaveReqState         = NWK_LEAVE_IDLE_STATE;
    NwkLinkStatusState       = NWK_LINK_STATUS_IDLE_STATE;
    NwkRouteDiscoveryState  = NWK_ROUTE_DISCOVERY_IDLE_STATE;
    NwkRouteRecordState      = NWK_ROUTE_RECORD_IDLE_STATE;
    NwkStatusState          = NWK_STATUS_SENDER_IDLE_STATE;


    gNwk_RouteDiscovery_sequence = 0;



    gNwk_conf_buf_ptr = NULL;


    //gMlmeCommStatus_FrameType = (frame_msgtype_t)0;
    //gMlmeCommStatus_DestAddr = 0;

    gNwkParse = (NwkParseHeader_t){0};
    gNwkFrameHeader_p = NULL;
    nwk_frame_ptr = NULL;

    gResIndex = 0;
    gRouteRelayRetries = 0;
    gSelectChannel = 0;
    gJoinCoordShortAddress = 0xFFFF;
    

    //InitNeighborTable();

    gNwkRoutingTable.failOrder = NWK_ROUTE_FAIL_TRIES;
    gNwkRoutingTable.size = 0;
    gNwkRoutingTable.table = gNwkRoutingTableEntry;
    gNwkRoutingTable.BitMap = 0;

    gNwkRouteCacheTable.size = 0;
    gNwkRouteCacheTable.record = gNwkRouteCacheRecordEntry;
    gNwkRouteCacheTable.BitMap[0] = 0;
    gNwkRouteCacheTable.BitMap[1] = 0;

    gNwkRouteDiscoveryTable.table = gNwkRouteDiscoveryTableEntry;
    gNwkRouteDiscoveryTable.BitMap = 0;
    gNwkRouteDiscoveryTable.isTimerStarted = false;
    gNwkRouteDiscoveryTable.size = 0;
    gNwkRouteDiscoveryTable.state = NWK_RDISC_TABLE_IDLE_STATE;

    uartEnableRequire = false;
    BackgroundCmdProce.byCmd = HPP_NET_CMD_NULL;

    gNwkPassiveAckTable.table = gNwkPassiveAckEntry;
    for (i=0; i<NWK_MAX_BROADCASR_TABLE_ENTRY; i++)
    {
    	gNwkPassiveAckTable.table[i].timerID = NO_TIMER;
    	nwkBroadDelay[i].delayCount = 0xFFFFFFFF;
    	nwkBroadDelay[i].transmit_frame = NULL;
    }
    gNwkPassiveAckTable.BitMap = 0;
    gNwkPassiveAckTable.amount = 0;
    broadDelayBitmap = 0;


    for (i=0; i<NWK_MAX_ROUTE_TABLE_ENTRY; i++)
    {
    	gNwkSequence[i].srcAddress = 0xFFFF;
    }
}



void nwk_soft_reset(uint8_t init_pib)
{
    reset_nwk_globals();

    ENTER_CRITICAL_REGION();
    pal_timer_stop(T_NWK_PERMIT_JOINING);
    pal_timer_stop(T_NWK_LINK_STATUS);    
    pal_timer_stop(T_NWK_ROUTE_DISCOVERY_TIME);
    pal_timer_stop(T_NWK_INIT_ROUTE_REQUEST_INTERVAL);
    pal_timer_stop(T_NWK_RELAY_ROUTE_REQUEST_INTERVAL);
    LEAVE_CRITICAL_REGION();

    if (init_pib)
    {
        nwk_init();
        InitNeighborTable();
        memset (gMyAddress_report, 0xFF, 22);
        gNwkAddrMap_BitMap = 0x00;
        gNeighborParent_p = NULL;
    }
    
    qmm_queue_flush(&nwk_aps_q);
    qmm_queue_flush(&aps_nwk_q);
#ifdef dataReqContinue
    qmm_queue_flush(&wait_nwk_ack_q);
#endif
}











/* Globals functions ---------------------------------------------------------*/
void nlme_reset_request(uint8_t *m)
{
    nlme_reset_req_t *nrr = (nlme_reset_req_t *)BMM_BUFFER_POINTER((buffer_t *)m);
    
    nwk_soft_reset(!nrr->WarmStart);
    gNwk_conf_buf_ptr = m;
    
    if (nrr->WarmStart)
       wpan_mlme_reset_req(false);
    else wpan_mlme_reset_req(true);

    NwkState = NWK_MODULE_ID_RESET;
} /* nlme_reset_request() */


void nwk_init(void)
{
#ifdef NIBFromFlash
  CheckSystemConfig();
	if (ModuleCfgInfo.AtCfgInfo.byModuleCfgFlag != ModuleDefaultConfig.AtCfgInfo.byModuleCfgFlag)
	{
		  gNwk_nib.transmitCounter = 0;
		  gNwk_nib.transmitFailureCounter = 0;
		  //gNwk_nib.managerAddr = 0x0000;
		  *(uint8_t *)&gNwk_nib.capabilityInformation = WPAN_CAP_ALLOCADDRESS;
		  gNwk_nib.sequenceNumber = (uint8_t)rand();
		 // gNwk_nib.networkBroadcastDeliveryTime

		  gNwk_nib.rejoinPermit = true;

		 // gNwk_nib.beacon.payload = mac_beacon_payload;

		  gNwk_nib.addrAlloc            = NWK_ADDR_ALLOC_DISTRIBUTED;

		  gNwk_nib.symLink              = true;
		  gNwk_nib.uniqueAddr           = true;
		  gNwk_nib.stackProfile         = 0;
		  gNwk_nib.protocolVersion      = NWKC_PROTOCOL_ID;
		  gNwk_nib.maxDepth             = 6;
		  gNwk_nib.maxRouters           = 6;
		  gNwk_nib.maxEndDevices        = 0;
		  gNwk_nib.maxChildren          = 6;
		  gNwk_nib.maxNeighborRouteCost = GOOD_CHANNEL;

		  gNwk_nib.channelPage          = TAL_CURRENT_PAGE_DEFAULT;
		  gNwk_nib.logicalChannel       = TAL_CURRENT_CHANNEL_DEFAULT;
		  gNwk_nib.networkAddress       = TAL_SHORT_ADDRESS_DEFAULT;
		  gNwk_nib.panId                = TAL_PANID_BC_DEFAULT;

		  gNwk_nib.updateId = 0;
		  gNwk_nib.parentNetworkAddress = macCoordShortAddress_def;
		  gNwk_nib.extendedPanId        = 0;
		#if defined _COORDINATOR_
		  gNwk_nib.depth = 0;
		  gNwk_nib.deviceType = DEVICE_TYPE_COORDINATOR;
		#elif defined _ROUTER_
		  gNwk_nib.depth = 0x0F;
		  gNwk_nib.deviceType = DEVICE_TYPE_ROUTER;
		#else
		  gNwk_nib.depth = 0x0F;
		  gNwk_nib.deviceType = DEV_TYPE_ENDDEVICE;
		#endif
	}
	else
	{
		gNwk_nib = App_System_Para_Pointer->nwkInfo;
	}




#else

  gNwk_nib.parentNetworkAddress = macCoordShortAddress_def;
  gNwk_nib.transmitCounter = 0;
  gNwk_nib.transmitFailureCounter = 0;
  //gNwk_nib.managerAddr = 0x0000;

  gNwk_nib.sequenceNumber = (uint8_t)rand();
 // gNwk_nib.networkBroadcastDeliveryTime
  gNwk_nib.updateId = 0;
  gNwk_nib.rejoinPermit = true;

 // gNwk_nib.beacon.payload = mac_beacon_payload;
  if (App_System_Para_Pointer->AtCfgInfo.isJoin == true && App_System_Para_Pointer->AtCfgInfo.requestAddress == true)
  gNwk_nib.addrAlloc            = NWK_ADDR_ALLOC_DISTRIBUTED;
  else gNwk_nib.addrAlloc       = NWK_ADDR_ALLOC_RESERVED;

  gNwk_nib.symLink              = false;
  gNwk_nib.uniqueAddr           = true;
  gNwk_nib.stackProfile         = 0;
  gNwk_nib.protocolVersion      = NWKC_PROTOCOL_ID;
  gNwk_nib.maxDepth             = App_System_Para_Pointer->AtCfgInfo.byNetClusterLm;

  gNwk_nib.maxNeighborRouteCost = GOOD_CHANNEL;

  gNwk_nib.channelPage          = tal_pib.CurrentPage;
//#ifdef NO_JOIN
  gNwk_nib.logicalChannel       = App_System_Para_Pointer->AtCfgInfo.byRfChannelID + 11;
  gNwk_nib.networkAddress       = App_System_Para_Pointer->AtCfgInfo.wNwkNodeID;
  gNwk_nib.panId                = App_System_Para_Pointer->AtCfgInfo.wMacPanID;
//#else
//		  gNwk_nib.logicalChannel       = TAL_CURRENT_CHANNEL_DEFAULT;
//		  gNwk_nib.networkAddress       = TAL_SHORT_ADDRESS_DEFAULT;
//		  gNwk_nib.panId                = TAL_PANID_BC_DEFAULT;
//#endif
  gNwk_nib.extendedPanId        = 0;
  gNwk_nib.maxRouters           = App_System_Para_Pointer->AtCfgInfo.byNetClusterRm;
  gNwk_nib.maxEndDevices        = App_System_Para_Pointer->AtCfgInfo.byNetClusterEm;
  gNwk_nib.maxChildren          = gNwk_nib.maxRouters + gNwk_nib.maxEndDevices;
if (App_System_Para_Pointer->AtCfgInfo.byNodeRouterType == DEVICE_TYPE_COORDINATOR)
{
  gNwk_nib.depth = 0;
  gNwk_nib.deviceType = DEVICE_TYPE_COORDINATOR;
  *(uint8_t *)&gNwk_nib.capabilityInformation = 0x8B;
}
else if (App_System_Para_Pointer->AtCfgInfo.byNodeRouterType == DEVICE_TYPE_ROUTER)
{
  gNwk_nib.depth = 0x0F;
  gNwk_nib.deviceType = DEVICE_TYPE_ROUTER;
  if (App_System_Para_Pointer->AtCfgInfo.requestAddress == true)
	  *(uint8_t *)&gNwk_nib.capabilityInformation = 0x8A;
  else *(uint8_t *)&gNwk_nib.capabilityInformation = 0x0A;
}
else
{
  gNwk_nib.depth = 0x0F;  
  gNwk_nib.deviceType = DEVICE_TYPE_END_DEVICE;  
  if (App_System_Para_Pointer->AtCfgInfo.requestAddress == true)
	  *(uint8_t *)&gNwk_nib.capabilityInformation = 0x80;
  else *(uint8_t *)&gNwk_nib.capabilityInformation = 0x00;
}
  
#endif

}
