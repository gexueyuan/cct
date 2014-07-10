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

#include "nwkLinkStatus.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//#if (HIGHEST_STACK_LAYER == NWK)
/* Regular MAC Dispatcher table */
static FLASH_DECLARE(handler_t nwk_dispatch_table[LAST_NWK_MESSAGE + 1]) =
{
    /* Internal message */
    [NLME_RESET_REQUEST]                  = NULL,//nlme_reset_request,



    [NLME_SET_REQUEST]                    = NULL,//nlme_set_request,
    [NLME_NETWORK_DISCOVERY_REQUEST]      = NULL,//nlme_NetworkDiscovery_request,
#ifndef _ENDDEVICE_      
    [NLME_PERMIT_JOINING_REQUEST]         = NULL,//nlme_PermitJoining_request,
    [NLME_FORMATION_REQUEST]              = NULL,//nlme_formation_request,
#endif


    [NLME_LEAVE_REQUEST]                  = NULL,//nlme_leave_request,
    [NLME_JOIN_REQUEST]                   = NULL,//nlme_join_request,

#ifndef _ENDDEVICE_     
    [NLME_JOIN_INDICATION]                = NULL,//nlme_join_ind,
    [NLME_START_ROUTER_REQUEST]           = NULL,//nlme_StartRouter_request,
#ifndef VANET_REDUCE_FUNC   
    [NLME_ROUTE_DISCOVERY_REQUEST]        = nlme_RouteDiscovery_request,
    [NLME_ROUTE_DISCOVERY_REPLY]          = nwkPrepareRouteReplyTx,
    [NLME_ROUTE_DISCOVERY_RELAY_REQUEST]  = nwkPrepareRouteRequestRelayTx,
    [NLME_ROUTE_REPLY_RELAY_REQUEST]	  = nwkPrepareRouteReplyRelayTx,
   
    [NLME_NWK_ROUTE_RECORD_REQUEST]       = nwkPrepareRouteRecordTx,
    [NLME_ROUTE_RECORD_RELAY_REQUEST]	  = nwkPrepareRouteRecordRelayTx,
#endif  
#endif   
    [NLME_LEAVE_IND_FORWARD]              = NULL,//nlme_leave_ind_forward,
#ifndef _ENDDEVICE_   
#ifndef VANET_REDUCE_FUNC    
    [NLME_NWK_LINK_STATUS_REQUEST]        = nwkPrepareLinkStatusTx,
#endif    
#ifndef VANET_REDUCE_FUNC  
    [NLME_NETWORK_STATUS_REQUEST]         = nwkPrepareNetworkStatusTx,
#endif    
#endif    
    [NLDE_DATA_REQUEST]                   = nlde_data_request,
#ifndef _ENDDEVICE_  
    [NLME_ROUTE_DISCOVERY_CONFIRM]        = nlme_RouteDiscovery_conf,
#endif       
    [NLDE_DATA_CONFIRM]                   = nlde_data_conf,
    [NLDE_DATA_INDICATION]                = nlde_data_ind,

    [NLME_LEAVE_INDICATION]               = NULL,//nlme_leave_ind,

    [NLME_LEAVE_CONFIRM]                  = NULL,//nlme_leave_conf,

    [NLME_NETWORK_DISCOVERY_CONFIRM]      = NULL,//nlme_NetworkDiscovery_conf,
#ifndef _ENDDEVICE_ 
    [NLME_START_ROUTER_CONFIRM]           = NULL,//nlme_StartRouter_conf,
#endif    

    [NLME_JOIN_CONFIRM]                   = NULL,//nlme_join_conf,


 //   [MLME_GET_CONFIRM]                    = mlme_get_conf,


    [NLME_SET_CONFIRM]                    = NULL,//nlme_set_conf,//cloudm
    [NLME_RESET_CONFIRM]                  = NULL,//nlme_reset_conf,
#ifndef _ENDDEVICE_ 
    [NLME_PERMIT_JOINING_CONFIRM]         = NULL,//nlme_PermitJoining_conf,

    [NLME_FORMATION_CONFIRM]              = NULL,//nlme_formation_conf,
#endif  
};
/* Globals variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/* Globals functions ---------------------------------------------------------*/
void nwk_dispatch_event(uint8_t *event)
{
    /*
     * A pointer to the body of the buffer is obtained from the pointer to the
     * received header.
     */
    uint8_t *buffer_body = BMM_BUFFER_POINTER((buffer_t *)event);

    if (buffer_body[CMD_ID_OCTET] <= NWK_LAST_MESSAGE)
    {
        /*
         * The following statement reads the address from the dispatch table
         * of the function to be called by utilizing function pointers.
         */
        handler_t handler = (handler_t)PGM_READ_WORD(&nwk_dispatch_table[buffer_body[CMD_ID_OCTET]]);

        if (handler != NULL)
        {
            handler(event);
        }
        else
        {
            bmm_buffer_free((buffer_t *)event);
#if (DEBUG > 0)
            ASSERT("Dispatch handler unavailable" == 0);
#endif
        }
    }
#ifdef ENABLE_RTB
    else
    {
        dispatch_rtb_event(event);
}
#endif  /* ENABLE_RTB */
}












