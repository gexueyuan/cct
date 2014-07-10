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
//#include "nwkMem.h"



#include "nlmeNetworkFormation.h"
#include "nwkFormation.h"

#include "main1.h"
#include "nwkPermitJoining.h"
#include "nlmePermitJoining.h"
#include "nwkStateMachine.h"
#include "nwkFrame.h"
#include "nldeData.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Globals variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Global  functions ---------------------------------------------------------*/
NWK_PRIVATE void nwkParseHeader(NwkFrameHeader_t *const nwkHeader,
                                NwkParseHeader_t *const parse,
                                buffer_t *m)
{
        NWK_DataInd_t *nlde_data_ind;
        /* Get the buffer body from buffer header */
        nlde_data_ind = (NWK_DataInd_t *)BMM_BUFFER_POINTER(m);
        
        /* Update the reset request structure */
        nlde_data_ind->cmdcode = NLDE_DATA_INDICATION;
        nlde_data_ind->dstAddrMode = nwkHeader->frameControl.multicastFlag ? NWK_DSTADDRMODE_MULTICAST : NWK_DSTADDRMODE_SHORT;
        nlde_data_ind->dstAddr = nwkHeader->dstAddr;
        nlde_data_ind->srcAddr = nwkHeader->srcAddr;
        nlde_data_ind->prevHopAddr =  parse->macSrcAddr;
        nlde_data_ind->nsduLength = parse->payloadSize; 
        nlde_data_ind->nsdu =  parse->payload;
        nlde_data_ind->linkQuality = parse->lqi;
        nlde_data_ind->rssi = parse->rssi;
        nlde_data_ind->rxTime = 0xFF;
        nlde_data_ind->securityUse = nwkHeader->frameControl.security;
        nlde_data_ind->radius = nwkHeader->radius; 
          
        qmm_queue_append(&nwk_aps_q, m);    
}

uint8_t XORSUM(uint8_t *data, uint8_t length)
{
	uint8_t result = 0;

	for (uint8_t i=0; i<length; i++)
	{
		result ^= data[i];
	}
	return result;
}

uint8_t testRssi(uint8_t rssi)
{
        rssi = 91-(84-rssi);
	if (rssi>85)
          return 4;
        else if (rssi>75)
          return 3;
        else if (rssi>65)
          return 2;
        else if (rssi>55)
          return 1;
        else 
          return 0;
}
