/**************************************************************************//**
  \file nwkRouteDiscoveryTable.h

  \brief Interface of the route discovery table.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2007-09-17 V. Panov - Created.
    2010-02-04 M. Gekk  - Redesign.
   Last change:
    $Id: nwkRouteDiscoveryTable.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
/**//**
 *  A ZigBee router or ZigBee coordinator maintains a route discovery table
 * containing the information that is required during route discovery.
 **/
#if !defined _NWK_ROUTE_DISCOVERY_TABLE_H
#define _NWK_ROUTE_DISCOVERY_TABLE_H

/******************************************************************************
                                 Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <nwkSystem.h>
#include <appFramework.h>
#include <appTimer.h>
#include <nwkCommon.h>
#include <nlmeRouteDiscovery.h>
#include <nwkPacket.h>

/******************************************************************************
                                 Define(s) section
 ******************************************************************************/
#define NWK_RDISC_IS_MANY_TO_ONE(routeDiscoveryEntry) \
  (NWK_DSTADDRMODE_NOADDR == (routeDiscoveryEntry)->dstAddrMode)
#define NWK_RDISC_IS_MULTICAST(routeDiscoveryEntry) \
  (NWK_DSTADDRMODE_MULTICAST == (routeDiscoveryEntry)->dstAddrMode)
#define NWK_RDISC_IS_UNICAST(routeDiscoveryEntry) \
  (NWK_DSTADDRMODE_SHORT == (routeDiscoveryEntry)->dstAddrMode)

/******************************************************************************
                                  Types section
 ******************************************************************************/
/** Route discovery table entry. See ZigBee spec r18, Table 3.53, page 391. */
typedef struct _NwkRouteDiscoveryEntry_t
{
  /** Time-to-live of entry. If this field is equal to 0 then entry is inactive.
   **/
  uint16_t ttl;
  /* For this entry route request transmission is required or not. */
  bool routeRequestRequired : 1;
  /** A flag indicating that route request is required on originator. */
  bool initialRouteRequest  : 1;
  /* For this entry route reply transmission is required or not. */
  bool routeReplyRequired   : 1;
  bool highPriority         : 1;
  /** A flag indicating that the concentrator does not store source routes. */
  bool noRouteCache         : 1;
  /* - NWK_DSTADDRMODE_NOADDR - this value indicating that the source node
   *     is a concentrator that issued a many-to-one route request.
   * - NWK_DSTADDRMODE_MULTICAST - the destination address is a Group ID.
   * - NWK_DSTADDRMODE_SHORT - unicast route discovery.
   * */
  NwkBitField_t dstAddrMode : 2;
  bool reserved             : 1;
  /** The accumulated path cost from the source of
   * the route request to the current device. */
  uint16_t forwardCost;
  /** The accumulated path cost from the current device
   * to the destination device. */
  uint16_t residualCost;
  /** The 16-bit network address of the route request's initiator. */
  ShortAddr_t srcAddr;
  /** The 16-bit network address of the device that has sent the most recent
   * lowest cost route request command frame corresponding to this entry's
   * route request identifier and source address. This field is used to
   * determine the path that an eventual route reply command frame should
   * follow. */
  ShortAddr_t senderAddr;
  /** Destination short address or group id of route discovery request. */
  ShortAddr_t dstAddr;
  /** A sequence number for a route request command frame that is incremented
   * each time a device initiates a route request. */
  NwkRouteRequestId_t routeRequestId;
  /** Current hop distance from originator of route descovery. */
  NwkRadius_t radius;
  /** Sequence number of initial route request command. */
  uint8_t sequenceNumber;
  /* 上一跳到自己的rssi，用于路由回复的累加residualCost中*/
  uint8_t senderAddrCost;
  /** Initial route request parameters. */
  NWK_RouteDiscoveryReq_t *req;
  /** Pointer to output network packet. */
  buffer_t *head;
  buffer_t *tail;
} NwkRouteDiscoveryEntry_t;

typedef struct _NWK_RouteDiscoveryReply_t
{
	enum nwk_msg_code cmdcode;
	NwkRouteDiscoveryEntry_t *entry;

}NWK_RouteDiscoveryReply_t;

typedef struct _NWK_RouteDiscoveryRelpyRelay_t
{
  enum nwk_msg_code cmdcode;
  frame_info_t *outPkt;
  NwkRouteDiscoveryEntry_t *entry;
} NWK_RouteDiscoveryRelpyRelay_t;
/** State of processing of incoming requests to send route request
 * or reply commands. */
typedef enum _NwkRouteDiscoveryTableState_t
{
  NWK_RDISC_TABLE_IDLE_STATE = 0x00,
  /** Preparing and sending of route reqply or request command. */
  NWK_RDISC_TABLE_REQUEST_PROCESSING_STATE = 0x93,
  /** Wating a free packet from the network packet manager. */
  NWK_RDISC_TABLE_WAIT_PACKET_STATE = 0x1A
} NwkRouteDiscoveryTableState_t;

/**
 * Route discovery table - a table used by a ZigBee coordinator or ZigBee router
 * to store temporary information used during route discovery.
 **/
typedef struct _NwkRouteDiscoveryTable_t
{
  /** The countdown timer that is used for decrement of time-to-live fields.*/
 // uint16_t timer;
  /** Pointer to the route discovery table. */
  NwkRouteDiscoveryEntry_t *table;
  /** Pointer to memory area after last entry of the route discovery table. */
 // NwkRouteDiscoveryEntry_t *end;
  /** Current state of the route discovery table. */
  NwkRouteDiscoveryTableState_t state;
  /** Boolean flag indicating that the countdown timer is started. */
  bool isTimerStarted;
  uint8_t size;
  uint32_t BitMap;
} NwkRouteDiscoveryTable_t;

/******************************************************************************
                               Prototypes section
 ******************************************************************************/
#if defined NWK_ROUTING_CAPACITY \
  && (defined _NWK_MESH_ROUTING_ || defined _NWK_MANY_TO_ONE_ROUTING_)
/**************************************************************************//**
  \brief Allocate place for new discovery request in the route discovery table.

  \param[in] highPriority - if new entry is important
    then this parameter is 'true' otherwise 'false'.
  \return Pointer to an allocated route discovery entry or NULL.
 ******************************************************************************/
NWK_PRIVATE
NwkRouteDiscoveryEntry_t* nwkAllocRouteDiscoveryEntry(const bool highPriority);

/**************************************************************************//**
  \brief Find an active route discovery entry by route request id and source address.

  \param[in] srcAddr - network address of route discovery originator.
  \param[in] requestId - identifier of initial route request.
  \return Pointer to a route discovery entry or NULL if entry is not found.
 ******************************************************************************/
NWK_PRIVATE NwkRouteDiscoveryEntry_t* nwkFindRouteDiscoveryEntry(
  const ShortAddr_t srcAddr,
  const NwkRouteRequestId_t requestId);

/**************************************************************************//**
  \brief Main task handler of the NWK route discovery component.
 ******************************************************************************/
NWK_PRIVATE void nwkRouteDiscoveryTableTaskHandler(void);

/**************************************************************************//**
  \brief Indication about network packet realizing.
 ******************************************************************************/
NWK_PRIVATE void nwkRouteDiscoveryTableFreePacketInd(void);

/**************************************************************************//**
  \brief Reset internal state and entries of the route discovery table.
 ******************************************************************************/
NWK_PRIVATE void nwkResetRouteDiscoveryTable(void);

/**************************************************************************//**
  \brief Stop processing of route discovery entries.
 ******************************************************************************/
NWK_PRIVATE void nwkFlushRouteDiscoveryTable(void);

/**************************************************************************//**
  \brief Prepare and send the route request command.

  \param[in] entry - pointer to a route discovery entry that is required
    transmission of route request command.
  \param[in] isInitialRequest - 'true' if this node is originator
    of route discovery otherwise 'false'.
  \return None.
 ******************************************************************************/
NWK_PRIVATE
void nwkSendRouteDiscoveryRequest(NwkRouteDiscoveryEntry_t *const entry,
  const bool isInitialRequest);

/******************************************************************************
  \brief nwkRouteDiscoveryTable idle checking.

  \return true, if nwkRouteDiscoveryTable performs no activity, false - otherwise.
 ******************************************************************************/
NWK_PRIVATE bool nwkRouteDiscoveryTableIsIdle(void);

#if defined _NWK_MESH_ROUTING_
/**************************************************************************//**
  \brief Prepare and send the route reply command.

  \param[in] entry - pointer to a route discovery entry that is required
    transmission of route reply command.
  \return None.
 ******************************************************************************/
NWK_PRIVATE
void nwkSendRouteDiscoveryReply(NwkRouteDiscoveryEntry_t *const entry);
#endif /* _NWK_MESH_ROUTING_ */

/**************************************************************************//**
  \brief Reliably adding a path cost and a link cost.

  \param[in] pathCost - cost of path.
  \param[in] linkCost - cost of link.
  \return Summary path cost.
 ******************************************************************************/
NWK_PRIVATE NwkPathCost_t nwkSumPathCost(const NwkPathCost_t pathCost,
  const NwkLinkCost_t linkCost);

/**************************************************************************//**
  \brief Confirmation of the route request transmission.

  \param[in] outPkt - pointer to output packet.
  \param[in] status - network status of route request transmission.
  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkConfirmRouteRequestTx(NwkOutputPacket_t *const outPkt,
  const NWK_Status_t status);

#else /* NWK_ROUTING_CAPACITY and _NWK_MESH_ROUTING_ */

#define nwkRouteDiscoveryTableTaskHandler NULL
#define nwkConfirmRouteRequestTx NULL
#define nwkRouteDiscoveryTableIsIdle NULL
#define nwkResetRouteDiscoveryTable() (void)0
#define nwkRouteDiscoveryTableFreePacketInd() (void)0

#endif /* NWK_ROUTING_CAPACITY and (_NWK_MESH_ROUTING_ or _NWK_MANY_TO_ONE_ROUTING_) */
#endif /* _NWK_ROUTE_DISCOVERY_TABLE_H */
/* eof nwkRouteDiscoveryTable.h */

