/******************************************************************************
  \file csSIB.h

  \brief
    BitCloud stack information base

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    21.10.10 A. Razinkov - Created.
******************************************************************************/

#ifndef _CSSIB_H_
#define _CSSIB_H_

/******************************************************************************
                    Includes section
******************************************************************************/
#include <types.h>
#include <csDefaults.h>
#ifdef _MAC_BAN_NODE_
  #include <macBanNode.h>
#endif /* _MAC_BAN_NODE_ */
#ifndef _MAC2_
  #include <apsConfigServer.h>
  #include <nwkConfigServer.h>
#endif

/******************************************************************************
                    Types section
******************************************************************************/
/**//**
 * \brief Config Server parameter handler
 */
typedef struct _CS_MemoryItem_t
{
  union
  {
    const void FLASH_PTR* flashValue;
    void* ramValue;
  } value;
  uint16_t size;
} CS_MemoryItem_t;

/**//**
 * \brief Read-only parametres, stored in FLASH
 */
typedef struct _CS_ReadOnlyItems_t
{
  /* MAC parameters */
  uint16_t csMacFrameRxBufferSize;
#ifdef _MAC_BAN_NODE_
  MAC_BanTableSize_t csMacBanTableSize;
#endif /* _MAC_BAN_NODE_ */

  /* NWK parameters */
#ifndef _MAC2_
  NwkRouteCacheSize_t csNwkRouteCacheSize;
  NwkSizeOfNeighborTable_t csNeibTableSize;
  NwkSizeOfAddressMap_t csAddressMapTableSize;
#endif
  uint8_t csMacPanDescriptorAmount;
  uint8_t csMaxChildrenAmount;
  uint8_t csMaxChildrenRouterAmount;
  uint8_t csMaxNetworkDepth;
  uint8_t csAddressAssignmentMethod;
  uint8_t csNwkBuffersAmount;
  uint8_t csJoinIndObjAmount;
  uint8_t csRouteTableSize;
  uint8_t csRouteDiscoveryTableSize;
  uint8_t csNwkBttSize;
#ifdef _GROUP_TABLE_
  uint8_t csGroupTableSize;
#endif /* _GROUP_TABLE_ */
#ifdef _NWK_PASSIVE_ACK_
  uint8_t csPassiveAckAmount;
#endif /* _NWK_PASSIVE_ACK_ */

  /* APS parameters */
  uint8_t  csApsDataReqBuffersAmount;
  uint8_t  csApsAckFrameBuffesAmount;
  uint8_t  csDuplicateRejectionTableSize;
#ifdef _BINDING_
  uint8_t  csApsBindingTableSize;
#endif /* _BINDING_ */
#ifdef _APS_FRAGMENTATION_
  uint8_t  csApsMaxBlocksAmount;
  uint16_t csApsBlockSize;
#endif /* _APS_FRAGMENTATION_ */

  /* ZCL parameters */
#ifdef _ZCL_
  uint8_t csZclMemoryBuffersAmount;
  #ifdef _OTAU_
    uint8_t  csOtauDiscoveredServerAmount;
    uint8_t  csOtauClientSessionAmount;
    uint16_t csOtauServerPageRequestResponseSpacing;
    uint16_t csOtauServerPageSize;
    uint8_t csOtauClientMissedBlocksBufferSize;
  #endif /* _OTAU_ */
#endif /* _ZCL_ */

  /* Other parameters */
  uint32_t csStackVersion;
  uint16_t csManufacturerCode;

  /* Security parameters */
#ifdef _SECURITY_
#ifdef _TRUST_CENTRE_
  uint8_t   csMaxTcAuthenticProcessAmount;
  #ifdef _TC_PERMISSION_TABLE_
    uint8_t csMaxTcAllowedDevicesAmount;
  #endif /* _TC_PERMISSION_TABLE_ */
#endif /* _TRUST_CENTRE_ */
  uint32_t  csApsSecurityTimeoutPeriod;
#ifdef _LINK_SECURITY_ 
  #ifdef _TRUST_CENTRE_
    uint8_t csMaxTcEstablishKeyAmount;
  #endif /* _TRUST_CENTRE_ */
  uint8_t csApsKeyPairDescriptorsAmount;
#endif /* _LINK_SECURITY_ */

#endif /* _SECURITY_ */
} CS_ReadOnlyItems_t;

/**//**
 * \brief Common stack information base
 */
typedef struct _SIB_t
{
  /* MAC parameters */
  uint32_t    csMacTransactionTime;
  int8_t      csRfTxPower;
#ifdef AT86RF212
  bool        csLbtMode;
#endif /* AT86RF212 */
#ifndef _MAC2_

  /* NWK parameters */
  PanId_t     csNwkPanid;
  bool        csNwkUseMulticast;
  uint8_t     csNwkMaxLinkStatusFailures;
  uint8_t     csNwkEndDeviceMaxFailures;
  uint8_t     csNwkLogicalChannel;
  bool        csNwkPredefinedPanid;
#ifdef _NWK_CONCENTRATOR_
  uint32_t     csNwkConcentratorDiscoveryTime;
#endif

  /* ZDO parameters */
  uint32_t         csEndDeviceSleepPeriod;
  uint32_t         csFfdSleepPeriod;
  bool             csRxOnWhenIdle;
  bool             csComplexDescriptorAvailable;
  bool             csUserDescriptorAvailable;
  uint32_t         csIndirectPollRate;
  uint8_t          csZdoJoinAttempts;
  uint32_t         csZdoJoinInterval;
  UserDescriptor_t csUserDescriptor; /* to EEPROM */
  ExtPanId_t       csExtPANID; /* user-defined PAN ID */
#ifdef _TC_SWAPOUT_
  uint8_t  csZdoTcKeepAliveInterval;
#endif // _TC_SWAPOUT_

  /* APS parameters */
  uint8_t csApsMaxFrameRetries;
#ifdef _APS_FRAGMENTATION_
  uint8_t csApsMaxTransmissionWindowSize;
#endif /* _APS_FRAGMENTATION_ */

  /* PDS parameters*/
  bool    csDtrWakeup;

  /* ZCL parameters */
#ifdef _ZCL_
  uint16_t csZclAttributeReportTimerInterval;
  #ifdef _OTAU_
    ExtAddr_t csOtauDefaultServerAddress;
    uint32_t  csOtauServerDiscoveryPeriod;
    bool csOtauServerPageRequestUsage;
  #endif /* _OTAU_ */
#endif /* _ZCL_ */

  /* Other parameters */
#ifdef _COMMISSIONING_
  uint32_t csPdsStoringInterval;
  #ifdef _POWER_FAILURE_
  bool     csPowerFailure;
  #endif /* _POWER_FAILURE_ */
#endif /* _COMMISSIONING_ */

  /* Security parameters */
#ifdef _SECURITY_
  bool      csSecurityOn;
  uint8_t   csZdoSecurityStatus;
#endif /* _SECURITY_ */
#endif /* _MAC2_ */
} SIB_t;

#endif /* _CSSIB_H_ */
/* eof csSIB.h */
