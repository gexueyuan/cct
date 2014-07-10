/**************************************************************************//**
  \file nwkRoutingTable.h

  \brief Interface of the routing table.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-11-15 Max Gekk - Created.
   Last change:
    $Id: nwkRoutingTable.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NWK_ROUTING_TABLE_H
#define _NWK_ROUTING_TABLE_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <appFramework.h>
#include <nwkCommon.h>
#include <nwkSystem.h>

/******************************************************************************
                                 Define(s) section
 ******************************************************************************/
typedef enum _routeTableStatus_t
{
	ACTIVE = 0,
	DISCOVERY_UNDERWAY = 1,
	DISCOVERY_FAILED = 2,
	INACTIVE = (0x3),
	VALIDATION_UNDERWAY = (0x4)
}routeTableStatus_t;


/******************************************************************************
                                Types section
 ******************************************************************************/
/** Type of routing table size. */
typedef uint8_t NwkRoutingTableSize_t;
/** Type of routing entry index. */
typedef NwkRoutingTableSize_t NwkRoutingEntryIndex_t;
/** Type of routing rate. */
typedef uint8_t NwkRoutingRate_t;

/** Routing table entry. ZigBee spec r17, Table 3.51. */
typedef struct _NwkRoutingTableEntry_t
{
  /** The 16-bit network address or Group ID of this route. If the destination
   * device is a ZigBee router, ZigBee coordinator, or an end device, and
   * nwkAddrAlloc has a value of 0x02, this field shall contain the actual
   * 16-bit address of that device. If the destination device is an end device
   * and nwkAddrAlloc has a value of 0x00, this field shall contain the 16-bit
   * network address of the device's parent. */
  ShortAddr_t dstAddr;
  /** The 16-bit network address of the next hop on the way to the destination.
   **/
  ShortAddr_t nextHopAddr;
  /** A flag indicating that the entry is active. */
  routeTableStatus_t status              : 3;
  /** A flag indicating that the destination indicated by this address does
   * not store source routes. */
  bool noRouteCache        : 1;
  /** A flag indicating that the destination is a concentrator
   * that issued a many-to-one route request. */
  bool manyToOne           : 1;
  /** A flag indicating that a route record command frame should be sent to
   * the destination prior to the next data packet. */
  bool routeRecordRequired : 1;
  /** A flag indicating that the destination address is a Group ID. */
  bool groupId             : 1;
  /** Counter of the many-to-one discovery periods
   * without any source route packets. */
 // NwkBitField_t noSourceRoutePeriods  : 2;
  /** Indicate to upper layer about new concentrator. */
  //bool newConcentrator     : 1;
  bool endDevice_Child_rxOff      : 1;
  NwkRoutingRate_t rate;
  /** Cost of route path to destination node. */
  uint16_t cost;
  uint8_t failures;//Ê§°Ü´ÎÊý
} NwkRoutingTableEntry_t;

/** The routing table. */
typedef struct _NwkRoutingTable_t
{
  /** Pointer to first entry of the routing table. */
  NwkRoutingTableEntry_t *table;
  /** Current size of the routing table. */
  NwkRoutingTableSize_t size;
  /* Number of failure transmission after that a routing entry will be deleted.
   **/
  uint8_t failOrder;
  uint32_t BitMap;
} NwkRoutingTable_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
#if defined NWK_ROUTING_CAPACITY \
  && (defined _NWK_MESH_ROUTING_ || defined _NWK_MANY_TO_ONE_ROUTING_)
/**************************************************************************//**
  \brief Allocate a new entry in the routing table.

  \return pointer to an entry or NULL.
 ******************************************************************************/
NWK_PRIVATE NwkRoutingTableEntry_t* nwkAllocRoutingEntry(void);

/**************************************************************************//**
  \brief Free the entry of the routing table.

  \param[in] dstAddr - a short address of destination node.
  \param[in] isGroupId - 'true' if dstAddr is group id otherwise 'false'.
  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkFreeRoutingEntry(const ShortAddr_t dstAddr,
  const bool isGroupId);

/**************************************************************************//**
  \brief Find a routing table entry by destination address.

  \param[in] dstAddr - a short address of destination node.
  \param[in] isGroupId - 'true' if dstAddr is group id otherwise 'false'.

  \return pointer to a routing table entry with given dstAddr.
 ******************************************************************************/
NWK_PRIVATE
NwkRoutingTableEntry_t* nwkFindRoutingEntry(const ShortAddr_t dstAddr,
  const bool isGroupId);

/******************************************************************************
 \brief Update information of the routing table entry after a packet transmission.

 \param[in] entry - pointer to an entry in the routing table.
 \param status - NWK status of packet transmission.
 ******************************************************************************/
NWK_PRIVATE void nwkUpdateRoutingEntry(NwkRoutingTableEntry_t *const entry,
  const NWK_Status_t status);

/**************************************************************************//**
  \brief Reset the routing table.
 ******************************************************************************/
NWK_PRIVATE void nwkResetRoutingTable(void);

#if defined _NWK_MESH_ROUTING_
/**************************************************************************//**
  \brief Setting the next hop address of the routing table entry.

  \param[in] dstAddr - a short address of destination node.
  \param[in] nextHopAddr - short address of next hop node.
  \param[in] isGroupId - 'true' if dstAddr is group id otherwise 'false'.
  \param[in] cost - cost of path to destination node.
  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkUpdateNextHop(const ShortAddr_t dstAddr,
  const bool isGroupId, const ShortAddr_t nextHopAddr,
  const NwkPathCost_t cost);

#else
#define nwkUpdateNextHop(dstAddr, isGroupId, nextHopAddr, cost) (void)0
#endif /* _NWK_MESH_ROUTING_ */
#else /* NWK_ROUTING_CAPACITY and (_NWK_MESH_ROUTING_ or _NWK_MANY_TO_ONE_ROUTING_) */

#define nwkResetRoutingTable() (void)0
//#define nwkUpdateNextHop(dstAddr, isGroupId, nextHopAddr, cost) (void)0
//#define nwkFreeRoutingEntry(dstAddr, isGroupId) (void)0

#endif /* NWK_ROUTING_CAPACITY and (_NWK_MESH_ROUTING_ or _NWK_MANY_TO_ONE_ROUTING_) */
#endif /* _NWK_ROUTING_TABLE_H */
/** eof nwkRoutingTable.h */

