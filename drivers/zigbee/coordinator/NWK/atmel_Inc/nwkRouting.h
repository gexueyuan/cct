/**************************************************************************//**
  \file nwkRouting.h

  \brief Routing header file.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2007-06-14 V. Panov - Created.
    2008-09-06 M. Gekk  - Modified to support parallel processing.
    2009-09-08 M. Gekk  - Refactoring.
   Last change:
    $Id: nwkRouting.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NWK_ROUTING_H
#define _NWK_ROUTING_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwkConfig.h>
#include <appFramework.h>
#include <nldeData.h>

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Getting MAC short address of next hop by destination network address.

  \param[in] dstAddr - destination network address.
  \param[in] srcAddr - short address of node for that a next hop is searched.
  \param[in] prevHopAddr - address of previous hop.
  \param[in] isMulticast - 'true' if destination address is multicast group.
  \param[in] isMulticastMember - 'true' if destination address is multicast
                                 member address.
  \param[in] nextHop - pointer to short address of next hop node
                       or MAC_NO_SHORT_ADDR.
  \param[in] hops - number of hops that is equal to CS_MAX_NETWORK_DEPTH * 2.
                   It is not read in function to avoid unnecessary reading of
                   CS_MAX_NETWORK_DEPTH when not implementing routing of packet
                   from upper layer.

  \return Routing method used for searching the next hop node.
 ******************************************************************************/
NWK_PRIVATE NwkRouting_t nwkGetNextHop(const ShortAddr_t dstAddr,
  const ShortAddr_t srcAddr, const ShortAddr_t prevHopAddr,
  const bool isMulticast, const bool isMulticastMember,
  ShortAddr_t *const nextHop, NwkRadius_t hops);

/**************************************************************************//**
  \brief Process routing result.

     Update link state, the routing table and indicate to upper layer..

  \param[in] outPkt - pointer to output packet.
  \param[in] nwkStatus - status of transmission.
  \return None.
 ******************************************************************************/
NWK_PRIVATE void nwkRoutingIsCompleted(const NwkOutputPacket_t *const outPkt,
  const NWK_Status_t nwkStatus);

#endif /* _NWK_ROUTING_H */
/** eof nwkRouting.h */

