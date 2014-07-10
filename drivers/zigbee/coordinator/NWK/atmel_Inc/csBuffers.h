/******************************************************************************
  \file csBuffers.h

  \brief
    BitCloud memory buffers

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    21.10.10 A. Razinkov - Created.
******************************************************************************/

#ifndef _CSBUFFERS_H_
#define _CSBUFFERS_H_

/******************************************************************************
                    Includes section
******************************************************************************/
#ifndef _MAC2_
#include <apsConfigServer.h>
#include <nwkConfigServer.h>
#else
#include <configServer.h>
#endif

#ifdef _SECURITY_
#include <tcAuthentic.h>
#endif /* _SECURITY */
#ifdef _LINK_SECURITY_ 
#include <tcKeyEstablish.h>
#endif /* _LINK_SECURITY_ */

#ifdef _ZCL_
#include <zclMemoryManager.h>
  #ifdef _OTAU_
#include <zclOTAUCluster.h>
#include <zclOtauDiscovery.h>
#include <zclOtauManager.h>
  #endif /* _OTAU_ */
#endif /* _ZCL_ */

/******************************************************************************
                    Types section
******************************************************************************/

/**//**
 * \brief Stack buffers list
 */
typedef struct _CS_StackBuffers_t
{
  /* MAC buffers */
  uint8_t csMacFrameRxBuffer[CS_MAC_FRAME_RX_BUFFER_SIZE];
#ifndef _MAC2_
  MAC_PanDescriptor_t csMacPanDescriptorBuffer[CS_NEIB_TABLE_SIZE];
#ifdef _MAC_BAN_NODE_
  MAC_BanTableEntry_t csBanTable[CS_BAN_TABLE_SIZE];
#endif /* _MAC_BAN_NODE_ */

  /* NWK buffers */
  Neib_t csNeibTable[CS_NEIB_TABLE_SIZE];
  MutableNeib_t csMutableNeibTable[CS_NEIB_TABLE_SIZE];
#if CS_ROUTE_CACHE_SIZE > 0
  NwkRouteCacheRecord_t csRouteCache[CS_ROUTE_CACHE_SIZE];
#endif
#if CS_NWK_BUFFERS_AMOUNT > 0
  NwkPacket_t csNwkBuffer[CS_NWK_BUFFERS_AMOUNT];
#endif
#if CS_ROUTE_TABLE_SIZE > 0
  NwkRoutingTableEntry_t csRoutingTable[CS_ROUTE_TABLE_SIZE];
#endif
#if CS_ROUTE_DISCOVERY_TABLE_SIZE > 0
  NwkRouteDiscoveryEntry_t csRouteDiscoveryTable[CS_ROUTE_DISCOVERY_TABLE_SIZE];
#endif
#if CS_ADDRESS_MAP_TABLE_SIZE > 0
  NwkAddressMapEntry_t csAddressMapTable[CS_ADDRESS_MAP_TABLE_SIZE];
#endif
#if CS_NWK_BTT_SIZE > 0
  SYS_DuplicateTableEntry_t csBTR[CS_NWK_BTT_SIZE];
#endif
#ifdef _GROUP_TABLE_
  NWK_GroupTableEntry_t csGroupTable[CS_GROUP_TABLE_SIZE];
#endif
#ifdef _SECURITY_
  NWK_SecurityKey_t csNwkSecKeys[CS_NWK_SECURITY_KEYS_AMOUNT];
  NWK_SecurityFrameCounters_t csNwkSecCounters[CS_NWK_SECURITY_KEYS_AMOUNT];
#endif /* _SECURITY_ */

  /* APS buffers */
#if CS_APS_DATA_REQ_BUFFERS_AMOUNT > 0
  ApsDataBuffer_t csApsDataReqBuffer[CS_APS_DATA_REQ_BUFFERS_AMOUNT];
#endif
#if CS_APS_ACK_FRAME_BUFFERS_AMOUNT > 0
  ApsAckBuffer_t csApsAckFrameBuffer[CS_APS_ACK_FRAME_BUFFERS_AMOUNT];
#endif
#if CS_JOIN_IND_OBJ_AMOUNT > 0
  NwkJoinIndObj_t csJoinIndObjects[CS_JOIN_IND_OBJ_AMOUNT];
#endif
#if CS_DUPLICATE_REJECTION_TABLE_SIZE > 0
  SYS_DuplicateTableEntry_t csDuplicateRejectionTable[CS_DUPLICATE_REJECTION_TABLE_SIZE];
#endif
#if defined(_BINDING_) && (CS_APS_BINDING_TABLE_SIZE > 0)
  ApsBindingEntry_t csApsBindingTable[CS_APS_BINDING_TABLE_SIZE];
#endif /* _BINDING_ */
#if defined(_APS_FRAGMENTATION_) && (CS_APS_MAX_BLOCKS_AMOUNT > 0)
  uint8_t csFragmentationMemoryPool[CS_APS_MAX_BLOCKS_AMOUNT * CS_APS_BLOCK_SIZE + CS_APS_MAX_BLOCKS_AMOUNT];
  uint8_t csFragmentationPacketBuffer[APS_AFFIX_LENGTH + CS_APS_BLOCK_SIZE];
#endif /* _APS_FRAGMENTATION_ */

  /* Trust Center buffers */
#ifdef _SECURITY_
  #if CS_MAX_TC_AUTHENTIC_PROCESS_AMOUNT > 0
    TC_AuthenticObj_t csTcAuthenticProcessBuffer[CS_MAX_TC_AUTHENTIC_PROCESS_AMOUNT];
  #endif
  #if defined(_TC_PERMISSION_TABLE_) && (CS_MAX_TC_ALLOWED_DEVICES_AMOUNT > 0)
    ExtAddr_t csTcDevicePermissionTable[CS_MAX_TC_ALLOWED_DEVICES_AMOUNT];
  #endif
  #ifdef _LINK_SECURITY_ 
    ApsKeyPairDescriptor_t csApsKeyPairDescriptors[CS_APS_KEY_PAIR_DESCRIPTORS_AMOUNT];
    ApsKeyCounters_t csApsKeyCounters[CS_APS_KEY_PAIR_DESCRIPTORS_AMOUNT];
    #if CS_MAX_TC_ESTABLISH_KEY_AMOUNT > 0
      TC_KeyEstablishObj_t csTcEstablishKeyBuffer[CS_MAX_TC_ESTABLISH_KEY_AMOUNT];
    #endif
  #endif /* _LINK_SECURITY_ */

#endif /* _SECURITY_ */

  /* ZCL buffers */
#ifdef _ZCL_
  ZclMmBufferDescriptor_t zclMmBuffers[CS_ZCL_MEMORY_BUFFERS_AMOUNT];
  #ifdef _OTAU_
    ZclOtauDiscoveryResult_t      csOtauDiscoveryResult[CS_ZCL_OTAU_DISCOVERED_SERVER_AMOUNT];
    ZclOtauServerTransac_t        csOtauSimultaneousClientSession[CS_ZCL_OTAU_CLIENT_SESSION_AMOUNT];
    ZclOtauMissedBlockResponses_t csOtauMissedBlocksBuffer[CS_ZCL_OTAU_MISSED_BLOCKS_BUFFER_SIZE];
    uint8_t                       csOtauImagePageDataBuffer[CS_ZCL_OTAU_IMAGE_PAGE_REQUEST_PAGE_SIZE];
  #endif /* _OTAU_ */
#endif /* _ZCL_ */

#if (defined _NWK_PASSIVE_ACK_) && (CS_NWK_PASSIVE_ACK_AMOUNT > 0)
  NwkPassiveAckEntry_t csNwkPassiveAckTable[CS_NWK_PASSIVE_ACK_AMOUNT];
#endif
#endif /* _MAC2_ */
} CS_StackBuffers_t;

#endif /* _CSBUFFERS_H_ */
/* eof csBuffers.h */
