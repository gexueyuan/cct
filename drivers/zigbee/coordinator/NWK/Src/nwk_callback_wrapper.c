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

#include "nlmeNetworkFormation.h"
#include "nwkFormation.h"

#include "nlmeNetworkDiscovery.h"

#include "nlmeStartRouter.h"
#include "nldeData.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Globals variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/* Globals functions ---------------------------------------------------------*/
//void nlme_reset_conf(uint8_t *m)
//{
//    nlme_reset_conf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (nlme_reset_conf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//    usr_nlme_reset_conf(pmsg->status, pmsg->WarmStart);
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}




/**
 * @brief Wrapper function for messages of type mlme_set_conf_t
 *
 * This function is a callback for mlme set confirm.
 *
 * @param m Pointer to message structure
 */
//void nlme_set_conf(uint8_t *m)//cloudm
//{
//    NWK_SetConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_SetConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}

#ifndef _ENDDEVICE_
//void nlme_PermitJoining_conf(uint8_t *m)
//{
//
//   NWK_PermitJoiningConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_PermitJoiningConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_PermitJoining_conf(pmsg->status);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//
//   // NwkPermitJoiningState = NWK_PERMIT_JOINING_WAIT_TIMER_FIRED_STATE;
//}




//void nlme_formation_conf(uint8_t *m)
//{
//
//    NWK_NetworkFormationConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_NetworkFormationConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_formation_conf(pmsg->status);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//
//}
#endif
//void nlme_NetworkDiscovery_conf(uint8_t *m)
//{
//    NWK_NetworkDiscoveryConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_NetworkDiscoveryConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_NetworkDiscovery_conf(pmsg->status, pmsg->networkCount, pmsg->networkDescriptors);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}


//void nlme_join_conf(uint8_t *m)
//{
//    NWK_JoinConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_JoinConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_join_conf(pmsg->status, pmsg->networkAddress, pmsg->extendedPANId, pmsg->activeChannel);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}


#ifndef _ENDDEVICE_
//void nlme_join_ind(uint8_t *m)
//{
//
//  NWK_JoinInd_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_JoinInd_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//    usr_nlme_join_ind(pmsg->networkAddress, pmsg->extendedAddress, pmsg->capabilityInformation,
//                      pmsg->rejoinNetwork, pmsg->secureJoin, pmsg->isRejoin);
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}


//void nlme_StartRouter_conf(uint8_t *m)
//{
//
//    NWK_StartRouterConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_StartRouterConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_StartRouter_conf(pmsg->status);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}
#endif

//void nlme_leave_conf(uint8_t *m)
//{
//    NWK_LeaveConf_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_LeaveConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//#ifdef MAC_SECURITY_ZIP
//    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
//#else
//    usr_nlme_leave_conf(pmsg->status, pmsg->DeviceAddress);
//#endif  /* MAC_SECURITY_ZIP */
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}


//
//void nlme_leave_ind(uint8_t *m)
//{
//    NWK_LeaveInd_t *pmsg;
//
//    /* Get the buffer body from buffer header */
//    pmsg = (NWK_LeaveInd_t *)BMM_BUFFER_POINTER(((buffer_t *)m));
//
//    usr_nlme_leave_ind(pmsg->deviceAddress, pmsg->rejoin, pmsg->removeChildren);
//
//    /* Free the buffer */
//    bmm_buffer_free((buffer_t *)m);
//}



void nlde_data_ind(uint8_t *m)
{
    NWK_DataInd_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (NWK_DataInd_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

    usr_nlde_data_ind(pmsg->dstAddrMode, 
                      pmsg->dstAddr,
                      pmsg->srcAddr,
                      pmsg->prevHopAddr,
                      pmsg->nsduLength,                      
                      pmsg->nsdu,
                      pmsg->linkQuality,                      
                      pmsg->rssi,
                      pmsg->rxTime,                      
                      pmsg->securityUse,
                      pmsg->radius
                      );

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}


void nlde_data_conf(uint8_t *m)
{
    NWK_DataConf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (NWK_DataConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));

#ifdef MAC_SECURITY_ZIP
    usr_nlde_data_conf(pmsg->status, pmsg->nsduHandle, pmsg->txTime, pmsg->hops);
#else
    usr_nlde_data_conf(pmsg->status, pmsg->nsduHandle, pmsg->txTime, pmsg->hops);
#endif  /* MAC_SECURITY_ZIP */

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}




#ifndef _ENDDEVICE_
void nlme_RouteDiscovery_conf(uint8_t *m)
{



#ifdef MAC_SECURITY_ZIP
	NWK_RouteDiscoveryConf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (NWK_RouteDiscoveryConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));	
    usr_nlme_set_conf(pmsg->status, pmsg->attrId, pmsg->PIBAttributeIndex);
#else
#ifndef VANET_REDUCE_FUNC    
	NWK_RouteDiscoveryConf_t *pmsg;

    /* Get the buffer body from buffer header */
    pmsg = (NWK_RouteDiscoveryConf_t *)BMM_BUFFER_POINTER(((buffer_t *)m));	
    usr_nlme_RouteDiscovery_conf(pmsg->status, pmsg->networkStatusCode, pmsg->nextHop);
#endif
#endif  /* MAC_SECURITY_ZIP */

    /* Free the buffer */
    bmm_buffer_free((buffer_t *)m);
}
#endif
