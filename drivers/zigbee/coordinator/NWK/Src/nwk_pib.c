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
#include "nwkIB.h"
#include "nwkAttributes.h"

#include "nwk1.h"
#include "nwkCommon.h"
#include "nwkConfig.h"
//#include "nwkMem.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Update this one the arry private_pib_size is updated. */
//#define MIN_PRIVATE_PIB_ATTRIBUTE_ID            (macIeeeAddress)
/* Update this one the arry phy_pib_size is updated. */
//#define MAX_PHY_PIB_ATTRIBUTE_ID            (phySymbolsPerOctet)

/* Update this one the arry mac_pib_size is updated. */
#define MIN_NWK_PIB_ATTRIBUTE_ID            (NWK_NIB_PANID_ID)
#define MAX_NWK_PIB_ATTRIBUTE_ID            (NWK_NIB_ADDRESS_MAP_ID)
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/






/* Update this one the arry phy_pib_size is updated. */
//#define MAX_PHY_PIB_ATTRIBUTE_ID            (phySymbolsPerOctet)

/* Size constants for MAC PIB attributes */
static FLASH_DECLARE(uint8_t nwk_pib_size[]) =
{
    sizeof(uint16_t),                // 0x40: macAckWaitDuration
    sizeof(uint8_t),                // 0x41: macAssociationPermit
    sizeof(uint16_t),                // 0x42: macAutoRequest
    sizeof(uint8_t),                // 0x43: macBattLifeExt
    sizeof(uint8_t),                // 0x44: macBattLifeExtPeriods
    sizeof(uint8_t),                // 0x45: macBeaconPayload
    sizeof(uint8_t),                // 0x86: macBeaconPayloadLength
    sizeof(uint32_t),                // 0x47: macBeaconOrder
    sizeof(uint8_t),               // 0x48: macBeaconTxTime
    sizeof(uint8_t),                // 0x49: macBSN
    sizeof(uint8_t),               // 0x4A: macCoordExtendedAddress
    sizeof(uint32_t),               // 0x4B: macCoordShortAddress
    sizeof(uint8_t),                // 0x4C: macDSN
    sizeof(uint16_t),                // 0x4D: macGTSPermit
    sizeof(uint8_t),                // 0x4E: macMaxCSMAbackoffs
    sizeof(uint8_t),                // 0x4F: macMinBE
    sizeof(uint8_t),               // 0x50: macPANId
    sizeof(uint8_t),                // 0x51: macPromiscuousMode
    sizeof(uint16_t),                // 0x52: macRxOnWhenIdle
    sizeof(uint8_t),               // 0x53: macShortAddress
    sizeof(uint8_t),                // 0x54: macSuperframeOrder
    sizeof(uint16_t),               // 0x55: macTransactionPersistenceTime
    sizeof(uint16_t),                // 0x56: macAssociatedPANCoord
    sizeof(uint8_t),                // 0x57: macMaxBE
    sizeof(uint32_t),               // 0x58: macMaxFrameTotalWaitTime
    sizeof(uint32_t),                // 0x59: macMaxFrameRetries
    sizeof(uint64_t),               // 0x5A: macResponseWaitTime
    sizeof(uint8_t),               // 0x5B: macSyncSymbolOffset
    sizeof(uint32_t),                // 0x5C: macTimestampSupported
    sizeof(uint8_t),                // 0x5D: macSecurityEnabled
    sizeof(uint8_t),                // 0x5E: macMinLIFSPeriod
    sizeof(uint8_t),                 // 0x5F: macMinSIFSPeriod
    sizeof(uint32_t), 
    sizeof(uint32_t), 
    sizeof(uint32_t), 
    sizeof(uint32_t), 
    sizeof(uint32_t), 
    sizeof(uint32_t), 
    sizeof(uint8_t), 
    sizeof(uint8_t), 
    sizeof(uint8_t), //a8
    sizeof(uint32_t)
};

/* Globals variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/* Globals functions ---------------------------------------------------------*/

/**
 * @brief Gets the size of a PIB attribute
 *
 * @param attribute PIB attribute
 *
 * @return Size (number of bytes) of the PIB attribute
 */
uint8_t nwk_get_ib_attribute_size(NWK_NibId_t pib_attribute_id)
{

//    if (MAX_PHY_PIB_ATTRIBUTE_ID >= pib_attribute_id)
//    {
//       return (PGM_READ_BYTE(&phy_pib_size[pib_attribute_id]));
//    }

    if (MIN_NWK_PIB_ATTRIBUTE_ID <= pib_attribute_id && MAX_NWK_PIB_ATTRIBUTE_ID >= pib_attribute_id)
    {
       return(PGM_READ_BYTE(&nwk_pib_size[pib_attribute_id - MIN_NWK_PIB_ATTRIBUTE_ID]));
    }

//#ifdef MAC_SECURITY_ZIP
//    if (MIN_MAC_SEC_PIB_ATTRIBUTE_ID <= pib_attribute_id && MAX_MAC_SEC_PIB_ATTRIBUTE_ID >= pib_attribute_id)
//    {
//       return(PGM_READ_BYTE(&mac_sec_pib_size[pib_attribute_id - MIN_MAC_SEC_PIB_ATTRIBUTE_ID]));
//    }
//#endif  /* MAC_SECURITY_ZIP */
//
//    if (MIN_PRIVATE_PIB_ATTRIBUTE_ID <= pib_attribute_id)
//    {
//        return(PGM_READ_BYTE(&private_pib_size[pib_attribute_id - MIN_PRIVATE_PIB_ATTRIBUTE_ID]));
//    }

    return(0);
}








//
//
//void NWK_SetReq(NWK_SetReq_t *const req);
//
//
//
void nlme_set_request(uint8_t *m)
{
    NWK_SetReq_t  *msr = (NWK_SetReq_t *)BMM_BUFFER_POINTER((buffer_t *)m);

    /* Do the actual PIB attribute set operation */
    {
        pib_value_t *attribute_value = (void *)&(msr->attrValue);
        NWK_Status_t status = NWK_SUCCESS_STATUS;
        NWK_SetConf_t *msc;
#ifdef MAC_SECURITY_ZIP
        /*
         * Store attribute index in local var, because
         * it will be overwritten later.
         */
        uint8_t attribute_index = msr->PIBAttributeIndex;
#endif  /* MAC_SECURITY_ZIP */

        /*
         * Call internal PIB attribute handling function. Always force
         * the trx back to sleep when using request primitives via the
         * MLME queue.
         */
#ifdef MAC_SECURITY_ZIP
        status = nlme_set(msr->PIBAttribute, msr->PIBAttributeIndex, attribute_value, true);
#else
        status = nlme_set(msr->attrId, attribute_value, true);
#endif
        msc = (NWK_SetConf_t *)msr;
        msc->attrId = msr->attrId;
#ifdef MAC_SECURITY_ZIP
        msc->PIBAttributeIndex = attribute_index;
#endif  /* MAC_SECURITY_ZIP */
        msc->cmdcode      = NLME_SET_CONFIRM;
        msc->status       = status;
    }

    /* Append the mlme set confirmation message to the MAC-NHLE queue */
    qmm_queue_append(&nwk_aps_q, (buffer_t *)m);
}




//#ifdef NWK_SECURITY_ZIP
//NWK_Status_t nlme_set(NWK_NibId_t attribute, uint8_t attribute_index, pib_value_t *attribute_value, bool set_trx_to_sleep)
//#else
NWK_Status_t nlme_set(NWK_NibId_t attribute, pib_value_t *attribute_value, bool set_trx_to_sleep)
//#endif
{
    NWK_Status_t status = NWK_SUCCESS_STATUS;

    switch (attribute)
    {
        case NWK_NIB_PANID_ID:
            gNwk_nib.panId = attribute_value->pib_value_16bit;
            break;

        case NWK_NIB_SEQUENCE_NUMBER_ID:
            gNwk_nib.sequenceNumber = attribute_value->pib_value_8bit;
            break;

        case NWK_NIB_MAX_CHILDREN_ID:
            gNwk_nib.maxEndDevices = attribute_value->pib_value_8bit-gNwk_nib.maxRouters;
            break;

        case NWK_NIB_MAX_DEPTH_ID:
            gNwk_nib.maxDepth = attribute_value->pib_value_8bit;
            break;
            
        case NWK_NIB_MAX_ROUTERS_ID:
            gNwk_nib.maxRouters = attribute_value->pib_value_8bit;
            break;        
            
        case NWK_NIB_CAPABILITY_INFORMATION_ID:
            *(uint8_t *)&gNwk_nib.capabilityInformation = attribute_value->pib_value_8bit;
            break;
        
        case NWK_NIB_ADDR_ALLOC_ID:     
            gNwk_nib.addrAlloc = (NWK_AddrAlloc_t)attribute_value->pib_value_8bit; 
            break;
        
        case NWK_NIB_INT_DEPTH_ID:
            gNwk_nib.depth = attribute_value->pib_value_8bit;
            break;
              
        case NWK_NIB_NETWORK_ADDRESS_ID:
            gNwk_nib.networkAddress = attribute_value->pib_value_16bit;
            break;

        case NWK_NIB_EXTENDED_PANID_ID:
            gNwk_nib.extendedPanId = attribute_value->pib_value_64bit;  
            break;
        
        case NWK_NIB_UPDATE_ID:
            gNwk_nib.updateId = attribute_value->pib_value_8bit;
            break;
            
   /** The time in seconds between link status command frames. */
           
     //   case     NWK_NIB_LINK_STATUS_PERIOD_ID:
          
     //     break;
      /** The number of missed link status command frames before resetting the link
       * costs to zero. */
    //    case NWK_NIB_ROUTER_AGE_LIMIT_ID:
          
   //       break;
      /** Use static addressing or not. */
        case NWK_NIB_UNIQUE_ADDR_ID:
            gNwk_nib.uniqueAddr = attribute_value->pib_value_8bit;
          
          break;
//          
//          
//#if (MAC_START_REQUEST_CONFIRM == 1)
//        case macBeaconPayload:
//            memcpy(mac_beacon_payload, attribute_value,
//                   mac_pib_macBeaconPayloadLength);
//            break;
//
//        case macBeaconPayloadLength:
//#ifndef REDUCED_PARAM_CHECK
//            /*
//             * If the application sits directly  on top of the MAC,
//             * this is also checked in mac_api.c.
//             */
//            if (attribute_value->pib_value_8bit > aMaxBeaconPayloadLength)
//            {
//                status = MAC_INVALID_PARAMETER;
//                break;
//            }
//#endif  /* REDUCED_PARAM_CHECK */
//            mac_pib_macBeaconPayloadLength = attribute_value->pib_value_8bit;
//            break;
//
//        case macBSN:
//            mac_pib_macBSN = attribute_value->pib_value_8bit;
//            break;
//#endif  /* (MAC_START_REQUEST_CONFIRM == 1) */
//
//#if (MAC_INDIRECT_DATA_FFD == 1)
//         case macTransactionPersistenceTime:
//            mac_pib_macTransactionPersistenceTime = attribute_value->pib_value_16bit;
//            break;
//#endif /* (MAC_INDIRECT_DATA_FFD == 1) */
//            case macCoordExtendedAddress:
//            mac_pib_macCoordExtendedAddress = attribute_value->pib_value_64bit;
//            break;
//
//        case macCoordShortAddress:
//            mac_pib_macCoordShortAddress = attribute_value->pib_value_16bit;
//            break;
//
//        case macDSN:
//            mac_pib_macDSN = attribute_value->pib_value_8bit;
//            break;
//
//
//        case macBattLifeExt:
//        case macBeaconOrder:
//        case macMaxCSMABackoffs:
//        case macMaxBE:
//        case macMaxFrameRetries:
//        case macMinBE:
//        case macPANId:
//#ifdef PROMISCUOUS_MODE
//        case macPromiscuousMode:
//#endif/* PROMISCUOUS_MODE */
//        case macShortAddress:
//        case macSuperframeOrder:
//        case macIeeeAddress:
//        case phyCurrentChannel:
//        case phyCurrentPage:
//        case phyTransmitPower:
//        case phyCCAMode:
//#ifdef TEST_HARNESS
//        case macPrivateCCAFailure:
//        case macPrivateDisableACK:
//#endif /* TEST_HARNESS */
//            {
//                /* Now only TAL PIB attributes are handled anymore. */
//                status = tal_pib_set(attribute, attribute_value);
//
//                if (status == TAL_TRX_ASLEEP)
//                {
//                    /*
//                     * Wake up the transceiver and repeat the attempt
//                     * to set the TAL PIB attribute.
//                     */
//                    tal_trx_wakeup();
//                    status = tal_pib_set(attribute, attribute_value);
//                    if (status == MAC_SUCCESS)
//                    {
//                        /*
//                         * Set flag indicating that the trx has been woken up
//                         * during PIB setting.
//                         */
//                        //trx_pib_wakeup = true;
//                    }
//               }
//
//#if ((MAC_INDIRECT_DATA_BASIC == 1) || defined(BEACON_SUPPORT))
//               /*
//                * In any case that the PIB setting was successful (no matter
//                * whether the trx had to be woken up or not), the PIB attribute
//                * recalculation needs to be done.
//                */
//               if (status == MAC_SUCCESS)
//               {
//                   /*
//                    * The value of the PIB attribute
//                    * macMaxFrameTotalWaitTime depends on the values of the
//                    * following PIB attributes:
//                    * macMinBE
//                    * macMaxBE
//                    * macMaxCSMABackoffs
//                    * phyMaxFrameDuration
//                    * In order to save code space and since changing of PIB
//                    * attributes is going to happen not too often, this is done
//                    * always whenever a PIB attribute residing in TAL is changed
//                    * (since all above mentioned PIB attributes are in TAL).
//                    */
//                   ;//recalc_macMaxFrameTotalWaitTime();//cloudm
//               }
//#endif  /* ((MAC_INDIRECT_DATA_BASIC == 1) || defined(BEACON_SUPPORT)) */
//            }
//            break;


        default:
            status = NWK_UNSUPPORTED_ATTRIBUTE_STATUS;
            break;


    }

    return status;
}
