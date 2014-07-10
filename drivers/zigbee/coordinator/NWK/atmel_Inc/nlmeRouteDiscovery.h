/**************************************************************************//**
  \file nlmeRouteDiscovery.h

  \brief NLME-ROUTE-DISCOVERY interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-13 Max Gekk - Created.
   Last change:
    $Id: nlmeRouteDiscovery.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NLME_ROUTE_DISCOVERY_H
#define _NLME_ROUTE_DISCOVERY_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <appFramework.h>
#include <nlmeNwkStatus.h>
#include <nwkCommon.h>
#include "nwk_msg_const.h"

/******************************************************************************
                                Types section
 ******************************************************************************/
/**//**
 * \brief NLME-ROUTE-DISCOVERY confirm primitive's parameters structure.
 * ZigBee Specification r17, 3.2.2.32 NLME-NETWORK-FORMATION.confirm, page 306.
 */
typedef struct _NWK_RouteDiscoveryConf_t
{
  enum nwk_msg_code cmdcode;
  /** The status of an attempt to initiate route discovery. */
  NWK_Status_t status;
  /** In the case where the Status parameter has a value of ROUTE_ERROR,
   * this code gives further information about the kind of error that occurred.
   * Otherwise, it should be ignored. ZigBee spec r17, Table 3.35. */
  NWK_StatusIndErrorCodes_t networkStatusCode;
  ShortAddr_t nextHop;
} NWK_RouteDiscoveryConf_t;

/**//**
 * \brief NLME-ROUTE-DISCOVERY request primitive's parameters structure.
 * Zigbee Specification r17, 3.2.2.31 NLME-ROUTE-DISCOVERY.request.
 */
typedef struct _NWK_RouteDiscoveryReq_t
{
  enum nwk_msg_code cmdcode;
  /** A parameter specifying the kind of destination address provided. */
  NWK_DstAddrMode_t dstAddrMode;
  /** The destination of the route discovery. */
  ShortAddr_t dstAddr;
  /** This optional parameter describes the number of hops that the route
   * request will travel through the network. */
  NwkRadius_t radius;
  /** In the case where DstAddrMode has a value of zero, indicating many-to-one
   * route discovery, this flag determines whether the NWK should establish
   * a route record table. TRUE = no route record table should be established,
   * FALSE = establish a route record table */
  bool noRouteCache;

  buffer_t *msg;
  uint8_t  *uartApp;

} NWK_RouteDiscoveryReq_t;


typedef struct _NWK_RouteDiscoveryReqRelay_t
{
  enum nwk_msg_code cmdcode;
  uint8_t payloadSize;
  uint8_t rssi;
  uint8_t *payload;
} NWK_RouteDiscoveryReqRelay_t;



/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief NLME-ROUTE-DISCOVERY request primitive's prototype.

  \param[in] req - NLME-ROUTE-DISCOVERY request parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_RouteDiscoveryReq(NWK_RouteDiscoveryReq_t *const req);

#endif /* _NLME_ROUTE_DISCOVERY_H */
/** eof nlmeRouteDiscovery.h */

