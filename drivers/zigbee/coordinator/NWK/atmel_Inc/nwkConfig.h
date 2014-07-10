/**************************************************************************//**
  \file nwkConfig.h

  \brief Constant definition header file.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2007-06-20 V. Panov - Created.
    2008-09-29 M. Gekk  - Values of constants for a network from 60
                        nodes are increased.
   Last change:
    $Id: nwkConfig.h 19021 2011-10-26 12:36:59Z mgekk $
******************************************************************************/
#if !defined _NWK_CONFIG_H
#define _NWK_CONFIG_H

//#include "nwkConfigServer.h"
/******************************************************************************
                             Definitions section
 ******************************************************************************/
/** A Boolean flag indicating whether the device is capable of becoming
 * the ZigBee coordinator. ZigBee spec r17, page 338, table 3.43.
 * */
#if defined _COORDINATOR_
  #define NWKC_COORDINATOR_CAPABLE true
#else
  #define NWKC_COORDINATOR_CAPABLE false
#endif /* _COORDINATOR_ */

/** Define for coordinator address */
#define NWKC_COORD_ADDR 0x0000U

/** North America */
/* #define NWK_NO_WIFI_CHANNELS_MASK 0x610800 */
/** Europe */
#define NWK_NO_WIFI_CHANNELS_MASK 0x618000U

#define NWK_FAVOURITE_CHANNELS_MASK (NWK_NO_WIFI_CHANNELS_MASK)

#define NWKC_PROTOCOL_ID 2U

#define NWKC_INITIAL_LINK_STATUS_PERIOD 15000UL /* msec15000UL*/

#define NWK_MAX_CHANNEL 26

#define NWK_MAX_LINK_STATUS_FAILURES 3U
#define NWK_END_DEVICE_MAX_FAILURES 3U

#define NWK_MAX_ED_LEVEL 0xCCU /* 255 * 0.8 */

/** Support network realignment */
/* #define NWK_COORD_REALIGNMENT */

#if defined _NWK_NONSTANDARD_BEACON_FILTER_
  /** Filter by extended pan id */
  #define NWK_BEACON_FILTER_EXTENDED_PANID
  /** If predefined short pan id then discard beacons with other pan id. */
  #define NWK_BEACON_FILTER_PREDEFINED_PANID
  /** Beacons with the end device capacity bit equal zero will discard. */
  #define NWK_BEACON_FILTER_END_DEVICE_CAPACITY
  /** Beacons with the router capacity bit equal zero will discard. */
  #define NWK_BEACON_FILTER_ROUTER_CAPACITY
#endif /* NWK_NONSTANDARD_BEACON_FILTER */

/** Update the neighbor table only if our network address is presented in
 * the received link status command. */
/* #define NWK_LINK_STATUS_ONLY_IN_LIST */

/** Duration of searching other networks on same channel.
 * It is ~(2^NWK_REPORT_SCAN_DURATION)*960*16 (usec) - for 2.4GHz */
#define NWK_SEARCH_NETWORKS_DURATION 8U /* ~4sec - 2.4GHz */

#define NWK_SELECT_RANDOM_PARENT
#define NWK_DELTA_LQI (255/10)

#if defined _NWK_ROUTE_CACHE_
  #define NWKC_NO_ROUTE_CACHE false
#else
  #define NWKC_NO_ROUTE_CACHE true
#endif /* _NWK_ROUTE_CACHE_ */

/** Maximum number of the many-to-one route requests after that the
 * route record command must be transmitted if no source route packet has
 * been received. */
#define NWK_MAX_NO_SOURCE_ROUTE_PERIODS 3U

#if defined _COORDINATOR_ || defined _ROUTER_
  #define NWK_ROUTING_CAPACITY
#endif /* _COORDINATOR_ or _ROUTER_ */

/** The maximum time duration in milliseconds allowed for the parent and
 * all child devices to retransmit a broadcast message. */
#ifndef VANET
#define  NWK_PASSIVE_ACK_TIMEOUT  470000UL /* octets duration ~= 500 ms. in 2.4Ghz */
#else
#define  NWK_PASSIVE_ACK_TIMEOUT  200000UL /* octets duration ~= 500 ms. in 2.4Ghz */
#endif
/** Maximum data packet retransmission. The maximum number of retries allowed
 * after a broadcast transmission failure.*/
#define NWK_MAX_BROADCAST_RETRIES 1U
//3U
//nwkBroadcastDeliveryTime = 2*nwkMaxDepth *
//((0.05+( nwkcMaxBroadcastJitter/2))+
//nwkPassiveAckTimeout* nwkBroadcastRetries/
//1000)
#define NWK_MAX_BROADCAST_JITTER 0x40

/** Maximum value of a link cost metric. */
#define NWK_MAX_LINK_COST 7U

/** Time duration in milliseconds until a route discovery expires.
 * ZigBee spec r18, Table 3.43, page 342. */
#define NWKC_ROUTE_DISCOVERY_TIME 100U

#define NWKC_RREQ_RETRY_INTERVAL 0xFE

#define NWKC_INITIAL_RREQ_RETRIES 0x01//0x03

#define NWKC_MIN_RREQ_JITTER 0x01

#define NWKC_MAX_RREQ_JITTER 0x40

#define NWKC_RELAY_RREQ_RETRIES 0x01

#define NWK_ALWAYS_REJECT_OWN_BROADCAST

#if !defined _NWK_ROUTING_OPTIMIZATION_
#define _NWK_ROUTING_OPTIMIZATION_ 0
#endif

#if defined _CERTIFICATION_
/* Doesn't change relationship of router child. */
#define NWK_TAKES_CARE_OF_CHILD_ROUTER

#define NWK_LIFE_TIME_OF_RX_ON_END_DEVICE (65534UL * 2048UL) /* msec */

#else /* not _CERTIFICATION_ */

#define NWK_LIFE_TIME_OF_RX_ON_END_DEVICE (60U * 2048UL) /* msec */

#endif /* _CERTIFICATION_ */

#define NWK_MAX_DEPTH_IN_BEACON 0xf

/** The cost of an outgoing link as measured by the neighbor. A value
 * of 0 indicates no outgoing cost is available. See ZigBee Spec r19, Table 3.48. */
#define NWK_NO_OUTGOING_COST 0U




#define _NWK_DISTRIBUTED_ADDRESSING_

#define NWK_BROADCAST_DELIVERY_TIME  ((2*3*((50+(64000/2))+NWK_PASSIVE_ACK_TIMEOUT*NWK_MAX_BROADCAST_RETRIES)))

#endif /* _NWK_CONFIG_H */
/** eof nwkConfig.h */
