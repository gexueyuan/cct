/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "pal.h"
#include "return_val.h"
#include "bmm.h"
#include "qmm.h"
#include "ieee_const.h"
#include "mac_msg_const.h"
#include "mac_api.h"
#include "mac_msg_types.h"
#include "stack_config.h"
#include "mac.h"
#include "mac_build_config.h"
#include "pal.h"
#include "mac_internal.h"

#include "nwk_msg_const.h"
#include "nwk_api.h"
#include "nwk_msg_types.h"
#include "nwk.h"
#include "nwk_internal.h"





//atmel_Inc
#include "nlmeSetGet.h"
#include "nlmeNetworkFormation.h"
#include "nwkFormation.h"
#include "nwkPermitJoining.h"
#include "nlmePermitJoining.h"
#include "nwkStateMachine.h"

#include "nwkDiscovery.h"
#include "nlmeNetworkDiscovery.h"

#include "nlmejoin.h"
#include "nwkJoinInd.h"
#include "nwkJoinReq.h"

#include "nlmeStartRouter.h"

#include "nlmeLeave.h"
#include "app_uart.h"
#include "pal_uart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Globals variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
//extern queue_t tal_incoming_frame_queue;
//extern buffer_t *tal_rx_buffer;
/* Private functions ---------------------------------------------------------*/




/* Globals functions ---------------------------------------------------------*/

bool wpan_nlme_reset_req(bool WarmStart)
{
    buffer_t *buffer_header;
    nlme_reset_req_t *nlme_reset_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_reset_req = (nlme_reset_req_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_reset_req->cmdcode = NLME_RESET_REQUEST;
    nlme_reset_req->WarmStart = WarmStart;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nek_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    NwkState = NWK_MODULE_NONE;
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
   // NwkState = NWK_MODULE_NONE;
    return true;
}




//#ifdef MAC_SECURITY_ZIP
//bool wpan_nlme_set_req(NWK_NibId_t PIBAttribute,
//                       uint8_t PIBAttributeIndex,
//                       void *PIBAttributeValue)
//#else
bool wpan_nlme_set_req(NWK_NibId_t PIBAttribute,
                       void *PIBAttributeValue)
//#endif  /* MAC_SECURITY_ZIP */
{
    buffer_t *buffer_header;
    NWK_SetReq_t *nlme_set_req;
    uint8_t pib_attribute_octet_no;
    
//    if (PIBAttribute == NWK_NIB_NETWORK_ADDRESS_ID)
//    {
//      wpan_mlme_set_req(macShortAddress, PIBAttributeValue);      
//    }
//    else 
    {
    /*
     * Allocate a large buffer for set request as maximum beacon payload
     * should be accommodated
     */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    /* Check for buffer availability */
    if (NULL == buffer_header)
    {
        return false;
    }

    /* Get size of PIB attribute to be set */
    pib_attribute_octet_no = nwk_get_ib_attribute_size(PIBAttribute);//cloudm

    /* Get the buffer body from buffer header */
    nlme_set_req = (NWK_SetReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Construct mlme_set_req_t message */
    nlme_set_req->cmdcode = NLME_SET_REQUEST;

    /* Attribute and attribute value length */
    nlme_set_req->attrId = PIBAttribute;
#ifdef NWK_SECURITY_ZIP
    nlme_set_req->PIBAttributeIndex = PIBAttributeIndex;
#endif  /* MAC_SECURITY_ZIP */

    /* Attribute value */



    memcpy((void *)&(nlme_set_req->attrValue),
                    (void *)PIBAttributeValue,
                    (size_t)pib_attribute_octet_no);


    /* Insert message into NHLE MAC queue */
#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
       /*
        * MLME-SET.request is not appended into NHLE MAC
        * queue, hence free the buffer allocated and return false
        */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
   }
    return true;
}
 // ChannelsMask_t scanChannels;
  /** The time spent scanning each channel is
   *  (aBaseSuperframeDuration * (2n + 1)) symbols,
   * where n is the value of the ScanDuration parameter. */
 // ScanDuration_t scanDuration;

bool wpan_nlme_formation_req(ChannelsMask_t scanChannels, ScanDuration_t scanDuration, bool coordRealignment)
{
    buffer_t *buffer_header;
    NWK_NetworkFormationReq_t *nlme_formation_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_formation_req = (NWK_NetworkFormationReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_formation_req->cmdcode = NLME_FORMATION_REQUEST;
    nlme_formation_req->scanChannels = scanChannels;
    nlme_formation_req->scanDuration = scanDuration;
    nlme_formation_req->coordRealignment = coordRealignment;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nek_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */

    return true;

}









bool wpan_nlme_PermitJoining_req(uint8_t permitDuration)
{
    buffer_t *buffer_header;
    NWK_PermitJoiningReq_t *nlme_PermintJoining_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_PermintJoining_req = (NWK_PermitJoiningReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_PermintJoining_req->cmdcode        = NLME_PERMIT_JOINING_REQUEST;
    nlme_PermintJoining_req->permitDuration = permitDuration;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;

}


bool wpan_nlme_NetworkDiscovery_req(ScanDuration_t scanDuration, ChannelsMask_t scanChannels,  bool clearNeighborTable)
{
    buffer_t *buffer_header;
    NWK_NetworkDiscoveryReq_t *nlme_NetworkDiscovery_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_NetworkDiscovery_req = (NWK_NetworkDiscoveryReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_NetworkDiscovery_req->cmdcode      = NLME_NETWORK_DISCOVERY_REQUEST;
    nlme_NetworkDiscovery_req->scanDuration = scanDuration;
    nlme_NetworkDiscovery_req->scanChannels = scanChannels;
    nlme_NetworkDiscovery_req->clearNeighborTable = clearNeighborTable;


#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}

  

  

bool wpan_nlme_join_req(ExtPanId_t extendedPANId, ScanDuration_t scanDuration, ChannelsMask_t scanChannels, NWK_RejoinNetwork_t rejoinNetwork)
{
    buffer_t *buffer_header;
    NWK_JoinReq_t *nlme_join_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_join_req = (NWK_JoinReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_join_req->cmdcode       = NLME_JOIN_REQUEST;
    nlme_join_req->extendedPANId = extendedPANId;
    nlme_join_req->scanDuration  = scanDuration;
    nlme_join_req->scanChannels  = scanChannels;
    nlme_join_req->rejoinNetwork = rejoinNetwork;


#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}




bool wpan_nlme_StartRouter_req(bool coordRealignment)
{
    buffer_t *buffer_header;
    NWK_StartRouterReq_t *nlme_StartRouter_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_StartRouter_req = (NWK_StartRouterReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_StartRouter_req->cmdcode          = NLME_START_ROUTER_REQUEST;
    nlme_StartRouter_req->coordRealignment = coordRealignment;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


bool wpan_nlme_leave_req(ExtAddr_t deviceAddress, bool removeChildren, bool rejoin)
{
    buffer_t *buffer_header;
    NWK_LeaveReq_t *nlme_leave_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_leave_req = (NWK_LeaveReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_leave_req->cmdcode        = NLME_LEAVE_REQUEST;
    nlme_leave_req->deviceAddress  = deviceAddress;
    nlme_leave_req->removeChildren = removeChildren;
    nlme_leave_req->rejoin         = rejoin;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


bool wpan_nlme_leave_ind_forward(ExtAddr_t deviceAddress, bool removeChildren, bool rejoin)
{
    buffer_t *buffer_header;
    NWK_LeaveReq_t *nlme_leave_req;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_leave_req = (NWK_LeaveReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_leave_req->cmdcode        = NLME_LEAVE_IND_FORWARD;
    nlme_leave_req->deviceAddress  = deviceAddress;
    nlme_leave_req->removeChildren = removeChildren;
    nlme_leave_req->rejoin         = rejoin;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}





bool wpan_nlme_NwkLinkStatus_CommandFrame_req(ShortAddr_t dstAddr)
{
    buffer_t *buffer_header;

    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */

    /* Update the reset request structure */
    buffer_header->body[0]       = NLME_NWK_LINK_STATUS_REQUEST;
    buffer_header->body[1]       = dstAddr>>8;
    buffer_header->body[2]       = dstAddr;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


//uint32_t clou_test1 = 0;
bool wpan_nlde_data_req(NWK_DstAddrMode_t dstAddrMode, 
                        ShortAddr_t dstAddr, 
                        uint8_t nsduLength,
                        uint8_t *nsdu,
                        uint8_t *nsduHandle,
                        uint8_t radius,
                        uint8_t nonMemberRadius,
                        bool    discoverRoute,
                        uint8_t    secutityEnable,
                        app_route_method_t routeType
                        )
{
    buffer_t *buffer_header;
    NWK_DataReq_t *nlde_data_req;
    uint8_t *payload_pos;
    //static uint8_t fullTimes=0;



    /* Allocate a small buffer for reset request */
    if (secutityEnable >= TIME_OUT)
    	buffer_header = (buffer_t *)nsduHandle;
    else buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
#ifndef REDUCE_CHECK_PARMER
        /* Buffer is not available */
		AppFrameHeader_t *AppFrameHeader = (AppFrameHeader_t *)nsduHandle;
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
        AppFrameHeader->AppPayload[0] = ERROR_BUFFER_FULL;
        AppFrameHeader->AppPayload[1] = XORSUM(nsduHandle, 8);
		pal_sio_tx(SIO_0, nsduHandle, 9);
#endif
       // DISABLE_UART_0_RX_INT();
        uartEnableRequire = true;
//        fullTimes++;
//        if (fullTimes > 3)
//        {
//        	//qmm_queue_flush(&tal_incoming_frame_queue);
//          //  qmm_queue_flush(&nhle_mac_q);
//          //  qmm_queue_flush(&tal_mac_q);
//
//        #if (HIGHEST_STACK_LAYER == MAC)
//            /* Flush MAC-NHLE queue */
//          //  qmm_queue_flush(&mac_nhle_q);
//        #endif
//
//        #if (MAC_INDIRECT_DATA_FFD == 1)
//            /* Flush MAC indirect queue */
//            qmm_queue_flush(&indirect_data_q);
//		#endif
//          //  qmm_queue_flush(&nwk_aps_q);
//            qmm_queue_flush(&aps_nwk_q);
//            bmm_buffer_init();// cloum2
//          //  if (tal_rx_buffer == NULL)
//          //  	tal_rx_buffer = bmm_buffer_alloc(LARGE_BUFFER_SIZE);
//        	fullTimes = 0;
//        }
        return false;
    }
    //fullTimes = 0;
    /* Get the buffer body from buffer header */
    nlde_data_req = (NWK_DataReq_t *)BMM_BUFFER_POINTER(buffer_header);
	if (routeType == APP_ROUTE_BROADCAST)
		if (nwkNewPassiveAck((frame_info_t *)nlde_data_req, mac_parse_data.src_addr.short_address,
				gNwkFrameHeader_p->srcAddr, gNwkFrameHeader_p->sequenceNumber) == false)
		{
			bmm_buffer_free(buffer_header);
			 return false;
		}

    /* Update the reset request structure */
    nlde_data_req->cmdcode      = NLDE_DATA_REQUEST;
    nlde_data_req->dstAddrMode  = dstAddrMode;
    nlde_data_req->dstAddr      = dstAddr;
    nlde_data_req->nsduLength   = nsduLength;
    nlde_data_req->nsduHandle   = nsduHandle;
    nlde_data_req->radius       = radius;
    nlde_data_req->nonmemberRadius = nonMemberRadius;
    nlde_data_req->discoverRoute   = discoverRoute;
    nlde_data_req->securityEnable  = secutityEnable;
    nlde_data_req->routeType = routeType;
    nlde_data_req->nsdu         = nsdu;
    if (dstAddrMode == NWK_DSTADDRMODE_NOADDR || dstAddrMode == NWK_DSTADDRMODE_RESPONSE)
    {
		/* Find the position where the data payload is to be updated */
    	if (secutityEnable < XOR_PASS)
    	{
			payload_pos = ((uint8_t *)nlde_data_req) + (LARGE_BUFFER_SIZE - FCS_LEN - nsduLength);

			/* Copy the payload to the end of buffer */
			memcpy(payload_pos, nsdu, nsduLength);
			nlde_data_req->nsdu = payload_pos;
    	}
    }


#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
   // clou_test1++;
    return true;
}


//uint32_t cloud_test4 = 0;
bool wpan_nlme_RouteDiscovery_req(NWK_DstAddrMode_t dstAddrMode, ShortAddr_t dstAddr, NwkRadius_t radius, bool noRouteCache, buffer_t *msg, buffer_t *uartBuffer)
{
    buffer_t *buffer_header;
    NWK_RouteDiscoveryReq_t *nlme_RouteDiscovery_req;

    /* Allocate a small buffer for reset request */
    if (uartBuffer == NULL)
    {
		buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

		if (NULL == buffer_header)
		{
			/* Buffer is not available */
			if (msg != NULL)
			{
			  //cloud_test4++;
			  qmm_queue_append(&aps_nwk_q,msg);
			}

			return false;
		}
    }
    else buffer_header = uartBuffer;


    /* Get the buffer body from buffer header */
    nlme_RouteDiscovery_req = (NWK_RouteDiscoveryReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_RouteDiscovery_req->cmdcode        = NLME_ROUTE_DISCOVERY_REQUEST;
    nlme_RouteDiscovery_req->dstAddr        = dstAddr;
    nlme_RouteDiscovery_req->dstAddrMode    = dstAddrMode;
    nlme_RouteDiscovery_req->radius         = radius;
    nlme_RouteDiscovery_req->noRouteCache   = noRouteCache;
    nlme_RouteDiscovery_req->msg = msg;
    if (uartBuffer == NULL)
    	nlme_RouteDiscovery_req->uartApp = NULL;
    else
    	nlme_RouteDiscovery_req->uartApp = ((uint8_t *)BMM_BUFFER_POINTER(buffer_header)) + (LARGE_BUFFER_SIZE - FCS_LEN - TRANSPARENT_IO_BUF_SIZE);


#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


bool wpan_nlme_RouteDiscovery_reply(NwkRouteDiscoveryEntry_t *const entry, uint8_t index)
{
    buffer_t *buffer_header;
    NWK_RouteDiscoveryReply_t *nlme_RouteDiscovery_reply;
    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
    	if (index != 0xFF)
    	{
    		gNwkRouteDiscoveryTable.table[index].ttl = 0;
			gNwkRouteDiscoveryTable.BitMap &= ~(1<<index);
			gNwkRouteDiscoveryTable.size --;
    	}
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    nlme_RouteDiscovery_reply = (NWK_RouteDiscoveryReply_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Update the reset request structure */
    nlme_RouteDiscovery_reply->cmdcode        = NLME_ROUTE_DISCOVERY_REPLY;
    nlme_RouteDiscovery_reply->entry = entry;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


bool wpan_nlme_RouteDiscoveryRelay_req(uint8_t payloadSize, uint8_t *payload, uint8_t index, uint8_t rssi)
{

	    buffer_t *buffer_header;
	    NWK_RouteDiscoveryReqRelay_t *NWK_RouteDiscoveryReqRelay;
	    uint8_t *payload_pos;

	    /* Allocate a large buffer for mcps data request */
	    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

	    if (NULL == buffer_header)
	    {
	    	if (index != 0xFF)
	    	{
	    		gNwkRouteDiscoveryTable.table[index].ttl = 0;//新建的表，要清0
				gNwkRouteDiscoveryTable.BitMap &= ~(1<<index);
				gNwkRouteDiscoveryTable.size --;
	    	}
	        /* Buffer is not available */
	        return false;
	    }

	    /* Get the buffer body from buffer header */
	    NWK_RouteDiscoveryReqRelay = (NWK_RouteDiscoveryReqRelay_t *)BMM_BUFFER_POINTER(buffer_header);

	    /* Construct mcps_data_req_t message */
	    NWK_RouteDiscoveryReqRelay->cmdcode = NLME_ROUTE_DISCOVERY_RELAY_REQUEST;

	    NWK_RouteDiscoveryReqRelay->payloadSize = payloadSize;
	    NWK_RouteDiscoveryReqRelay->rssi = rssi;
	    /* Find the position where the data payload is to be updated */
	    payload_pos = ((uint8_t *)NWK_RouteDiscoveryReqRelay) + (LARGE_BUFFER_SIZE - FCS_LEN - payloadSize);
	    NWK_RouteDiscoveryReqRelay->payload = payload_pos;
	    /* Copy the payload to the end of buffer */
	    memcpy(payload_pos, payload, payloadSize);

	#ifdef ENABLE_QUEUE_CAPACITY
	    if (MAC_SUCCESS != qmm_queue_append(&nhle_mac_q, buffer_header))
	    {
	        /*
	         * MCPS-DATA.request is not appended into NHLE MAC
	         * queue, hence free the buffer allocated and return false
	         */
	        bmm_buffer_free(buffer_header);
	        return false;
	    }
	#else
	    qmm_queue_append(&aps_nwk_q, buffer_header);
	#endif  /* ENABLE_QUEUE_CAPACITY */

	    return true;
}



bool wpan_nlme_RouteRelpy_Relay_req(frame_info_t *const outPkt, NwkRouteDiscoveryEntry_t *const entry)
{

	    buffer_t *buffer_header;
	    NWK_RouteDiscoveryRelpyRelay_t *NWK_RouteDiscoveryRelpyRelay;
	  //  uint8_t *payload_pos;

	    /* Allocate a large buffer for mcps data request */
	    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

	    if (NULL == buffer_header)
	    {
	        /* Buffer is not available */
	        return false;
	    }

	    /* Get the buffer body from buffer header */
	    NWK_RouteDiscoveryRelpyRelay = (NWK_RouteDiscoveryRelpyRelay_t *)BMM_BUFFER_POINTER(buffer_header);

	    /* Construct mcps_data_req_t message */
	    NWK_RouteDiscoveryRelpyRelay->cmdcode = NLME_ROUTE_REPLY_RELAY_REQUEST;

	    NWK_RouteDiscoveryRelpyRelay->entry = entry;

	    /* Find the position where the data payload is to be updated */
	   // payload_pos = ((uint8_t *)NWK_RouteDiscoveryReqRelay) + (LARGE_BUFFER_SIZE - FCS_LEN - payloadSize);
	    NWK_RouteDiscoveryRelpyRelay->outPkt = outPkt;
	    /* Copy the payload to the end of buffer */
	  //  memcpy(payload_pos, payload, payloadSize);

	#ifdef ENABLE_QUEUE_CAPACITY
	    if (MAC_SUCCESS != qmm_queue_append(&nhle_mac_q, buffer_header))
	    {
	        /*
	         * MCPS-DATA.request is not appended into NHLE MAC
	         * queue, hence free the buffer allocated and return false
	         */
	        bmm_buffer_free(buffer_header);
	        return false;
	    }
	#else
	    qmm_queue_append(&aps_nwk_q, buffer_header);
	#endif  /* ENABLE_QUEUE_CAPACITY */

	    return true;
}



bool wpan_nlme_RouteRecord_CommandFrame_req(ShortAddr_t dstAddr,buffer_t *uartBuffer)
{
    buffer_t *buffer_header;
    NWK_RouteRecordReq_t *NWK_RouteRecordReq;
    /* Allocate a small buffer for reset request */
    if (uartBuffer == NULL)
    {
    	buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    	if (NULL == buffer_header)
    	{
    		return false;
    	}
    }
    else buffer_header = uartBuffer;

    NWK_RouteRecordReq = (NWK_RouteRecordReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Construct mcps_data_req_t message */
    NWK_RouteRecordReq->cmdcode = NLME_NWK_ROUTE_RECORD_REQUEST;
    NWK_RouteRecordReq->MacDestAddress = dstAddr;
    if (uartBuffer == NULL)
    	NWK_RouteRecordReq->uartApp = NULL;
      else
    	  NWK_RouteRecordReq->uartApp = ((uint8_t *)BMM_BUFFER_POINTER(buffer_header)) + (LARGE_BUFFER_SIZE - FCS_LEN - TRANSPARENT_IO_BUF_SIZE);


#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


bool wpan_nlme_RouteRecord_Relay_CommandFrame_req(uint8_t payloadSize, uint8_t *payload, uint8_t *relayPayload, uint8_t realySize, uint16_t MacDestAddress)
{

	    buffer_t *buffer_header;
	    NWK_RouteRecordRelay_t *NWK_RouteRecordRelay;
	    uint8_t *payload_pos;

	    /* Allocate a large buffer for mcps data request */
	    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

	    if (NULL == buffer_header)
	    {
	        /* Buffer is not available */
	        return false;
	    }

	    /* Get the buffer body from buffer header */
	    NWK_RouteRecordRelay = (NWK_RouteRecordRelay_t *)BMM_BUFFER_POINTER(buffer_header);

	    /* Construct mcps_data_req_t message */
	    NWK_RouteRecordRelay->cmdcode = NLME_ROUTE_RECORD_RELAY_REQUEST;
	    NWK_RouteRecordRelay->MacDestAddress = MacDestAddress;
	    NWK_RouteRecordRelay->payloadSize = payloadSize + realySize;

	    /* Find the position where the data payload is to be updated */
	    payload_pos = ((uint8_t *)NWK_RouteRecordRelay) + (LARGE_BUFFER_SIZE - FCS_LEN - payloadSize - realySize);
	    NWK_RouteRecordRelay->payload = payload_pos;
	    /* Copy the payload to the end of buffer */
	    memcpy(payload_pos, payload, payloadSize);
	    memcpy(&payload_pos[payloadSize], relayPayload, realySize);


	#ifdef ENABLE_QUEUE_CAPACITY
	    if (MAC_SUCCESS != qmm_queue_append(&nhle_mac_q, buffer_header))
	    {
	        /*
	         * MCPS-DATA.request is not appended into NHLE MAC
	         * queue, hence free the buffer allocated and return false
	         */
	        bmm_buffer_free(buffer_header);
	        return false;
	    }
	#else
	    qmm_queue_append(&aps_nwk_q, buffer_header);
	#endif  /* ENABLE_QUEUE_CAPACITY */

	    return true;
}


bool wpan_nlme_NwkStatus_CommandFrame_req(NWK_StatusIndErrorCodes_t statusCode,
											ShortAddr_t targetAddr,
											ShortAddr_t dstAddr,
											ShortAddr_t nextHopAddr,
											ShortAddr_t prevHopAddr)
{
    buffer_t *buffer_header;
    NwkStatusReq_t  *NWK_StatusReq;
    NwkStatusState = NWK_STATUS_SENDER_IDLE_STATE;
    /* Allocate a small buffer for reset request */
    buffer_header = bmm_buffer_alloc(LARGE_BUFFER_SIZE);

    if (NULL == buffer_header)
    {
        /* Buffer is not available */
        return false;
    }

    /* Get the buffer body from buffer header */
    NWK_StatusReq = (NwkStatusReq_t *)BMM_BUFFER_POINTER(buffer_header);

    /* Construct mcps_data_req_t message */
    NWK_StatusReq->cmdcode = NLME_NETWORK_STATUS_REQUEST;
    NWK_StatusReq->statusCode = statusCode;
    NWK_StatusReq->targetAddr = targetAddr;
    NWK_StatusReq->dstAddr = dstAddr;
    NWK_StatusReq->nextHopAddr = nextHopAddr;
    NWK_StatusReq->prevHopAddr = prevHopAddr;

#ifdef ENABLE_QUEUE_CAPACITY
    if (MAC_SUCCESS != qmm_queue_append(&aps_nwk_q, buffer_header))
    {
        /*
         * MLME-RESET.request is not appended into NHLE MAC
         * queue, hence free the buffer allocated and return false
         */
        bmm_buffer_free(buffer_header);
        return false;
    }
#else
    qmm_queue_append(&aps_nwk_q, buffer_header);
#endif  /* ENABLE_QUEUE_CAPACITY */
    return true;
}


