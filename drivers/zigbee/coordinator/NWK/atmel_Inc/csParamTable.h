/******************************************************************************
\file csParamTable.h

  \brief
    Configuration Server parameters information table

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2010 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
    History:
    22.12.10 A. Razinkov - Created.
******************************************************************************/

/* This table contains information about particular paramter displacement address,
 * memory type it occupies (FLASH or RAM), and its representation in memory
 * (as memory region or atomic entity).
 *
 * This table should be inlined to external module.
 * RAM_PARAMETER(id, addr), FLASH_PARAMETER(id, addr), MEMORY_REGION(idm addr),
 * SEPARATOR(id) and DUMMY_MEMORY(id) macroses should be defined first to provide
 * specific table morphing.
 *
 * SEPARATOR(id) macro is used to separate atomic parameters stored in RAM from
 * the ones stored in FLASH. It's the only restriction on parameters order in this table.
 * Memory regions could be described everywhere.
 *
 * DUMMY_MEMORY(id) macro is used to specify few stack buffers sizes during the application
 * compilation phase.
 */

RAM_PARAMETER(CS_UID_ID, csPIB.macAttr.extAddr)
RAM_PARAMETER(CS_MAX_FRAME_TRANSMISSION_TIME_ID, csPIB.macAttr.maxFrameTransmissionTime)
RAM_PARAMETER(CS_MAC_TRANSACTION_TIME_ID, csSIB.csMacTransactionTime)
RAM_PARAMETER(CS_RF_TX_POWER_ID, csSIB.csRfTxPower)
#ifndef _MAC2_
RAM_PARAMETER(CS_MAX_NEIGHBOR_ROUTE_COST_ID, csNIB.maxNeighborRouteCost)
RAM_PARAMETER(CS_NWK_EXT_PANID_ID, csNIB.extendedPanId)
RAM_PARAMETER(CS_NWK_ADDR_ID, csNIB.networkAddress)
RAM_PARAMETER(CS_NWK_PARENT_ADDR_ID, csNIB.parentNetworkAddress)
RAM_PARAMETER(CS_NWK_DEPTH_ID, csNIB.depth)
RAM_PARAMETER(CS_NWK_UNIQUE_ADDR_ID, csNIB.uniqueAddr)
RAM_PARAMETER(CS_CHANNEL_PAGE_ID, csNIB.channelPage)
RAM_PARAMETER(CS_NWK_USE_MULTICAST_ID, csSIB.csNwkUseMulticast)
RAM_PARAMETER(CS_NWK_MAX_LINK_STATUS_FAILURES_ID, csSIB.csNwkMaxLinkStatusFailures)
RAM_PARAMETER(CS_NWK_END_DEVICE_MAX_FAILURES_ID, csSIB.csNwkEndDeviceMaxFailures)
RAM_PARAMETER(CS_NWK_LOGICAL_CHANNEL_ID, csSIB.csNwkLogicalChannel)
RAM_PARAMETER(CS_NWK_PANID_ID, csSIB.csNwkPanid)
RAM_PARAMETER(CS_NWK_PREDEFINED_PANID_ID, csSIB.csNwkPredefinedPanid)
RAM_PARAMETER(CS_PROTOCOL_VERSION_ID, csNIB.protocolVersion)
RAM_PARAMETER(CS_STACK_PROFILE_ID, csNIB.stackProfile)
RAM_PARAMETER(CS_SCAN_DURATION_ID, csZIB.scanDuration)
RAM_PARAMETER(CS_PERMIT_DURATION_ID, csZIB.permitJoinDuration)
RAM_PARAMETER(CS_EXT_PANID_ID, csSIB.csExtPANID)
RAM_PARAMETER(CS_CHANNEL_MASK_ID, csZIB.channelMask)
RAM_PARAMETER(CS_INDIRECT_POLL_RATE_ID, csSIB.csIndirectPollRate)
RAM_PARAMETER(CS_END_DEVICE_SLEEP_PERIOD_ID, csSIB.csEndDeviceSleepPeriod)
RAM_PARAMETER(CS_FFD_SLEEP_PERIOD_ID, csSIB.csFfdSleepPeriod)
RAM_PARAMETER(CS_RX_ON_WHEN_IDLE_ID, csSIB.csRxOnWhenIdle)
RAM_PARAMETER(CS_COMPLEX_DESCRIPTOR_AVAILABLE_ID, csSIB.csComplexDescriptorAvailable)
RAM_PARAMETER(CS_USER_DESCRIPTOR_AVAILABLE_ID, csSIB.csUserDescriptorAvailable)
RAM_PARAMETER(CS_ZDP_USER_DESCRIPTOR_ID, csSIB.csUserDescriptor)
RAM_PARAMETER(CS_DEVICE_TYPE_ID, csNIB.deviceType)
RAM_PARAMETER(CS_ZDO_JOIN_ATTEMPTS_ID, csSIB.csZdoJoinAttempts)
RAM_PARAMETER(CS_ZDO_JOIN_INTERVAL_ID, csSIB.csZdoJoinInterval)
RAM_PARAMETER(CS_APS_MAX_FRAME_RETRIES_ID, csSIB.csApsMaxFrameRetries)
RAM_PARAMETER(CS_ZDP_RESPONSE_TIMEOUT_ID, csZIB.zdpResponseTimeout)
RAM_PARAMETER(CS_DTR_WAKEUP_ID, csSIB.csDtrWakeup)
#endif

#ifdef AT86RF212
RAM_PARAMETER(CS_LBT_MODE_ID, csSIB.csLbtMode)
#endif
#ifdef _NWK_CONCENTRATOR_
RAM_PARAMETER(CS_CONCENTRATOR_DISCOVERY_TIME_ID, csSIB.csNwkConcentratorDiscoveryTime)
#endif
#ifdef _TC_SWAPOUT_
RAM_PARAMETER(CS_ZDO_TC_KEEP_ALIVE_INTERVAL_ID, csSIB.csZdoTcKeepAliveInterval)
#endif
#ifdef _APS_FRAGMENTATION_
RAM_PARAMETER(CS_APS_MAX_TRANSMISSION_WINDOW_SIZE_ID, csSIB.csApsMaxTransmissionWindowSize)
#endif
#ifdef _COMMISSIONING_
RAM_PARAMETER(CS_PDS_STORING_INTERVAL_ID, csSIB.csPdsStoringInterval)
  #ifdef _POWER_FAILURE_
RAM_PARAMETER(CS_POWER_FAILURE_ID, csSIB.csPowerFailure)
  #endif /* _POWER_FAILURE_ */
#endif /* _COMMISSIONING_ */

#ifdef _ZCL_
RAM_PARAMETER(CS_ZCL_ATTRIBUTE_REPORT_TIMER_INTERVAL_ID, csSIB.csZclAttributeReportTimerInterval)
  #ifdef _OTAU_
RAM_PARAMETER(CS_ZCL_OTAU_DEFAULT_UPGRADE_SERVER_IEEE_ADDRESS_ID, csSIB.csOtauDefaultServerAddress)
RAM_PARAMETER(CS_ZCL_OTAU_DEFAULT_SERVER_DISCOVERY_PERIOD_ID, csSIB.csOtauServerDiscoveryPeriod)
RAM_PARAMETER(CS_ZCL_OTAU_IMAGE_PAGE_REQUEST_USAGE_ID, csSIB.csOtauServerPageRequestUsage)
  #endif /* _OTAU_ */
#endif /* _ZCL_ */
#ifdef _SECURITY_
RAM_PARAMETER(CS_NETWORK_KEY_ID, defaultKey)
RAM_PARAMETER(CS_SECURITY_ON_ID, csSIB.csSecurityOn)
RAM_PARAMETER(CS_SECURITY_LEVEL_ID, csNIB.securityIB.securityLevel)
RAM_PARAMETER(CS_SECURITY_ALL_FRAMES_ID, csNIB.securityIB.secureAllFrames)
RAM_PARAMETER(CS_ZDO_SECURITY_STATUS_ID, csSIB.csZdoSecurityStatus)
RAM_PARAMETER(CS_APS_TRUST_CENTER_ADDRESS_ID, csAIB.trustCenterAddress)
#endif /* _SECURITY_ */

SEPARATOR(CS_FLASH_PARAMETERS_START_ID)
FLASH_PARAMETER(CS_MAC_FRAME_RX_BUFFER_SIZE_ID, csReadOnlyItems.csMacFrameRxBufferSize)
#ifndef _MAC2_
FLASH_PARAMETER(CS_NEIB_TABLE_SIZE_ID, csReadOnlyItems.csNeibTableSize)
FLASH_PARAMETER(CS_ROUTE_CACHE_SIZE_ID, csReadOnlyItems.csNwkRouteCacheSize)
#endif
FLASH_PARAMETER(CS_MAC_PAN_DESCRIPTOR_AMOUNT_ID, csReadOnlyItems.csMacPanDescriptorAmount)
FLASH_PARAMETER(CS_MAX_CHILDREN_AMOUNT_ID, csReadOnlyItems.csMaxChildrenAmount)
FLASH_PARAMETER(CS_MAX_CHILDREN_ROUTER_AMOUNT_ID, csReadOnlyItems.csMaxChildrenRouterAmount)
FLASH_PARAMETER(CS_MAX_NETWORK_DEPTH_ID, csReadOnlyItems.csMaxNetworkDepth)
FLASH_PARAMETER(CS_ADDRESS_ASSIGNMENT_METHOD_ID, csReadOnlyItems.csAddressAssignmentMethod)
FLASH_PARAMETER(CS_NWK_BUFFERS_AMOUNT_ID, csReadOnlyItems.csNwkBuffersAmount)
FLASH_PARAMETER(CS_JOIN_IND_OBJ_AMOUNT_ID, csReadOnlyItems.csJoinIndObjAmount)
FLASH_PARAMETER(CS_ROUTE_TABLE_SIZE_ID, csReadOnlyItems.csRouteTableSize)
#ifndef _MAC2_
FLASH_PARAMETER(CS_ADDRESS_MAP_TABLE_SIZE_ID, csReadOnlyItems.csAddressMapTableSize)
#endif
FLASH_PARAMETER(CS_ROUTE_DISCOVERY_TABLE_SIZE_ID, csReadOnlyItems.csRouteDiscoveryTableSize)
FLASH_PARAMETER(CS_BTT_SIZE_ID, csReadOnlyItems.csNwkBttSize)
FLASH_PARAMETER(CS_MANUFACTURER_CODE_ID, csReadOnlyItems.csManufacturerCode)
FLASH_PARAMETER(CS_APS_DATA_REQ_BUFFERS_AMOUNT_ID, csReadOnlyItems.csApsDataReqBuffersAmount)
FLASH_PARAMETER(CS_APS_ACK_FRAME_BUFFERS_AMOUNT_ID, csReadOnlyItems.csApsAckFrameBuffesAmount)
FLASH_PARAMETER(CS_DUPLICATE_REJECTION_TABLE_SIZE_ID, csReadOnlyItems.csDuplicateRejectionTableSize)
FLASH_PARAMETER(CS_STACK_VERSION_ID, csReadOnlyItems.csStackVersion)

#ifdef _MAC_BAN_NODE_
FLASH_PARAMETER(CS_BAN_TABLE_SIZE_ID, csReadOnlyItems.csMacBanTableSize)
#endif
#ifdef _GROUP_TABLE_
FLASH_PARAMETER(CS_GROUP_TABLE_SIZE_ID, csReadOnlyItems.csGroupTableSize)
#endif
#ifdef _NWK_PASSIVE_ACK_
FLASH_PARAMETER(CS_NWK_PASSIVE_ACK_AMOUNT_ID, csReadOnlyItems.csPassiveAckAmount)
#endif
#ifdef _BINDING_
FLASH_PARAMETER(CS_APS_BINDING_TABLE_SIZE_ID, csReadOnlyItems.csApsBindingTableSize)
#endif
#ifdef _APS_FRAGMENTATION_
FLASH_PARAMETER(CS_APS_MAX_BLOCKS_AMOUNT_ID, csReadOnlyItems.csApsMaxBlocksAmount)
FLASH_PARAMETER(CS_APS_BLOCK_SIZE_ID, csReadOnlyItems.csApsBlockSize)
#endif
#ifdef _ZCL_
FLASH_PARAMETER(CS_ZCL_MEMORY_BUFFERS_AMOUNT_ID, csReadOnlyItems.csZclMemoryBuffersAmount)
  #ifdef _OTAU_
FLASH_PARAMETER(CS_ZCL_OTAU_DISCOVERED_SERVER_AMOUNT_ID, csReadOnlyItems.csOtauDiscoveredServerAmount)
FLASH_PARAMETER(CS_ZCL_OTAU_CLIENT_SESSION_AMOUNT_ID, csReadOnlyItems.csOtauClientSessionAmount)
FLASH_PARAMETER(CS_ZCL_OTAU_IMAGE_PAGE_REQUEST_RESPONSE_SPACING_ID, csReadOnlyItems.csOtauServerPageRequestResponseSpacing)
FLASH_PARAMETER(CS_ZCL_OTAU_IMAGE_PAGE_REQUEST_PAGE_SIZE_ID, csReadOnlyItems.csOtauServerPageSize)
FLASH_PARAMETER(CS_ZCL_OTAU_MISSED_BLOCKS_BUFFER_SIZE_ID, csReadOnlyItems.csOtauClientMissedBlocksBufferSize)
  #endif /* _OTAU_ */
#endif
#ifdef _SECURITY_
FLASH_PARAMETER(CS_APS_SECURITY_TIMEOUT_PERIOD_ID, csReadOnlyItems.csApsSecurityTimeoutPeriod)
  #ifdef _TRUST_CENTRE_
FLASH_PARAMETER(CS_MAX_TC_AUTHENTIC_PROCESS_AMOUNT_ID, csReadOnlyItems.csMaxTcAuthenticProcessAmount)
    #ifdef _TC_PERMISSION_TABLE_
FLASH_PARAMETER(CS_MAX_TC_ALLOWED_DEVICES_AMOUNT_ID, csReadOnlyItems.csMaxTcAllowedDevicesAmount)
    #endif /* _TC_PERMISSION_TABLE_ */
  #endif /* _TRUST_CENTRE_ */
#endif /* _SECURITY_ */
#ifdef _LINK_SECURITY_ 
FLASH_PARAMETER(CS_APS_KEY_PAIR_DESCRIPTORS_AMOUNT_ID, csReadOnlyItems.csApsKeyPairDescriptorsAmount)
  #ifdef _TRUST_CENTRE_
FLASH_PARAMETER(CS_MAX_TC_ESTABLISH_KEY_AMOUNT_ID, csReadOnlyItems.csMaxTcEstablishKeyAmount)
  #endif /* _TRUST_CENTRE_ */
#endif /* _LINK_SECURITY_ */

MEMORY_REGION(CS_MAC_FRAME_RX_BUFFER_ID, stackBuffers.csMacFrameRxBuffer)
#ifndef _MAC2_
MEMORY_REGION(CS_NEIB_TABLE_ID, stackBuffers.csNeibTable)
MEMORY_REGION(CS_MAC_PAN_DESCRIPTOR_BUFFER_ID, stackBuffers.csMacPanDescriptorBuffer)
MEMORY_REGION(CS_MUTABLE_NEIB_TABLE_ID, stackBuffers.csMutableNeibTable)
#endif

#ifdef _MAC_BAN_NODE_
MEMORY_REGION(CS_BAN_TABLE_ID, stackBuffers.csBanTable)
#endif

#if CS_ROUTE_CACHE_SIZE > 0
MEMORY_REGION(CS_ROUTE_CACHE_ID, stackBuffers.csRouteCache)
#else
DUMMY_MEMORY(CS_ROUTE_CACHE_ID)
#endif /* CS_ROUTE_CACHE_SIZE > 0 */

#if CS_NWK_BUFFERS_AMOUNT > 0
MEMORY_REGION(CS_NWK_BUFFERS_ID, stackBuffers.csNwkBuffer)
#else
DUMMY_MEMORY(CS_NWK_BUFFERS_ID)
#endif /* CS_NWK_BUFFERS_AMOUNT > 0 */

#if CS_JOIN_IND_OBJ_AMOUNT > 0
MEMORY_REGION(CS_JOIN_IND_OBJ_ID, stackBuffers.csJoinIndObjects)
#else
DUMMY_MEMORY(CS_JOIN_IND_OBJ_ID)
#endif /* CS_JOIN_IND_OBJ_AMOUNT > 0 */

#if CS_DUPLICATE_REJECTION_TABLE_SIZE > 0
MEMORY_REGION(CS_DUPLICATE_REJECTION_TABLE_ID, stackBuffers.csDuplicateRejectionTable)
#else
DUMMY_MEMORY(CS_DUPLICATE_REJECTION_TABLE_ID)
#endif /* CS_DUPLICATE_REJECTION_TABLE_SIZE > 0 */

#if CS_ROUTE_TABLE_SIZE > 0
MEMORY_REGION(CS_ROUTE_TABLE_ID, stackBuffers.csRoutingTable)
#else
DUMMY_MEMORY(CS_ROUTE_TABLE_ID)
#endif /* CS_ROUTE_TABLE_SIZE > 0 */

#if CS_ADDRESS_MAP_TABLE_SIZE > 0
MEMORY_REGION(CS_ADDRESS_MAP_TABLE_ID, stackBuffers.csAddressMapTable)
#else
DUMMY_MEMORY(CS_ADDRESS_MAP_TABLE_ID)
#endif /* CS_ADDRESS_MAP_TABLE_SIZE > 0 */

#if CS_ROUTE_DISCOVERY_TABLE_SIZE > 0
MEMORY_REGION(CS_ROUTE_DISCOVERY_TABLE_ID, stackBuffers.csRouteDiscoveryTable)
#else
DUMMY_MEMORY(CS_ROUTE_DISCOVERY_TABLE_ID)
#endif /* CS_ROUTE_DISCOVERY_TABLE_SIZE > 0 */

#if CS_NWK_BTT_SIZE > 0
MEMORY_REGION(CS_BTT_ID, stackBuffers.csBTR)
#else
DUMMY_MEMORY(CS_BTT_ID)
#endif /* CS_NWK_BTT_SIZE > 0 */

#if CS_APS_DATA_REQ_BUFFERS_AMOUNT > 0
MEMORY_REGION(CS_APS_DATA_REQ_BUFFER_ID, stackBuffers.csApsDataReqBuffer)
#else
DUMMY_MEMORY(CS_APS_DATA_REQ_BUFFER_ID)
#endif /* CS_APS_DATA_REQ_BUFFERS_AMOUNT > 0 */

#if CS_APS_ACK_FRAME_BUFFERS_AMOUNT > 0
MEMORY_REGION(CS_APS_ACK_FRAME_BUFFER_ID, stackBuffers.csApsAckFrameBuffer)
#else
DUMMY_MEMORY(CS_APS_ACK_FRAME_BUFFER_ID)
#endif /* CS_APS_ACK_FRAME_BUFFERS_AMOUNT > 0 */

#if defined(_APS_FRAGMENTATION_) && (CS_APS_MAX_BLOCKS_AMOUNT > 0)
MEMORY_REGION(CS_APS_FRAGMENTATION_MEMORY_POOL_ID, stackBuffers.csFragmentationMemoryPool)
MEMORY_REGION(CS_APS_FRAGMENTATION_PACKET_BUFFER_ID, stackBuffers.csFragmentationPacketBuffer)
#else
DUMMY_MEMORY(CS_APS_FRAGMENTATION_MEMORY_POOL_ID)
DUMMY_MEMORY(CS_APS_FRAGMENTATION_PACKET_BUFFER_ID)
#endif /* _APS_FRAGMENTATION_ */

#ifdef _GROUP_TABLE_
MEMORY_REGION(CS_GROUP_TABLE_ID, stackBuffers.csGroupTable)
#endif
#ifdef _BINDING_
MEMORY_REGION(CS_APS_BINDING_TABLE_ID, stackBuffers.csApsBindingTable)
#endif

#ifdef _ZCL_
MEMORY_REGION(CS_ZCL_MEMORY_BUFFERS_ID, stackBuffers.zclMmBuffers)
  #ifdef _OTAU_
MEMORY_REGION(CS_ZCL_OTAU_DISCOVERED_SERVER_RESULT_ID, stackBuffers.csOtauDiscoveryResult)
MEMORY_REGION(CS_ZCL_OTAU_CLIENT_SESSION_MEMORY_ID, stackBuffers.csOtauSimultaneousClientSession)
MEMORY_REGION(CS_ZCL_OTAU_MISSED_BLOCKS_BUFFER_ID, stackBuffers.csOtauMissedBlocksBuffer)
MEMORY_REGION(CS_ZCL_OTAU_PAGE_REQUEST_PAGE_BUFFER_ID, stackBuffers.csOtauImagePageDataBuffer)
  #endif /* _OTAU_ */
#endif

#ifdef _SECURITY_
MEMORY_REGION(CS_NWK_SECURITY_IB_ID, csNIB.securityIB)
MEMORY_REGION(CS_NWK_SECURITY_KEYS_ID, stackBuffers.csNwkSecKeys)
MEMORY_REGION(CS_NWK_MUTABLE_SECURITY_IB_ID, stackBuffers.csNwkSecCounters)
  #ifdef _TRUST_CENTRE_
    #if (CS_MAX_TC_AUTHENTIC_PROCESS_AMOUNT > 0)
MEMORY_REGION(CS_TC_AUTHENTIC_PROCESS_BUFFER_ID, stackBuffers.csTcAuthenticProcessBuffer)
    #else
DUMMY_MEMORY(CS_TC_AUTHENTIC_PROCESS_BUFFER_ID)
    #endif /* (CS_MAX_TC_AUTHENTIC_PROCESS_AMOUNT > 0) */
    #if defined(_TC_PERMISSION_TABLE_) && (CS_MAX_TC_ALLOWED_DEVICES_AMOUNT > 0)
MEMORY_REGION(CS_TC_PERMISSION_TABLE_ID, stackBuffers.csTcDevicePermissionTable)
    #else
DUMMY_MEMORY(CS_TC_PERMISSION_TABLE_ID)
    #endif /* defined(_TC_PERMISSION_TABLE_) & (CS_MAX_TC_ALLOWED_DEVICES_AMOUNT > 0) */
  #endif /* _TRUST_CENTRE_ */
#endif /* _SECURITY_ */
#ifdef _LINK_SECURITY_ 
MEMORY_REGION(CS_APS_KEY_PAIR_DESCRIPTORS_ID, stackBuffers.csApsKeyPairDescriptors)
MEMORY_REGION(CS_APS_KEY_PAIR_COUNTERS_ID, stackBuffers.csApsKeyCounters)
  #if defined(_TRUST_CENTRE_) && (CS_MAX_TC_ESTABLISH_KEY_AMOUNT > 0)
MEMORY_REGION(CS_TC_ESTABLISH_KEY_BUFFER_ID, stackBuffers.csTcEstablishKeyBuffer)
  #else
DUMMY_MEMORY(CS_TC_ESTABLISH_KEY_BUFFER_ID)
  #endif /* defined _LINK_SECURITY_ & defined(_TRUST_CENTRE_) & (CS_MAX_TC_ESTABLISH_KEY_AMOUNT > 0)*/

#endif /* _LINK_SECURITY_ */

#if (defined _NWK_PASSIVE_ACK_) && (CS_NWK_PASSIVE_ACK_AMOUNT > 0)
MEMORY_REGION(CS_NWK_PASSIVE_ACK_TABLE_ID, stackBuffers.csNwkPassiveAckTable)
#endif /* _NWK_PASSIVE_ACK_ */
