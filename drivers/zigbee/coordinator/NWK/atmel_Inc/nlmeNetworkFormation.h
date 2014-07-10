/**************************************************************************//**
  \file nlmeNetworkFormation.h

  \brief NLME-NETWORK-FORMATION interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-11 Max Gekk - Created.
   Last change:
    $Id: nlmeNetworkFormation.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NLME_NETWORK_FORMATION_H
#define _NLME_NETWORK_FORMATION_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <mac.h>
#include <nwkCommon.h>
#include <nlmeEdScan.h>
#include "nwk_msg_types.h"
/******************************************************************************
                                Types section
 ******************************************************************************/
/**//**
 * \brief NLME-NETWORK-FORMATION confirm primitive's parameters structure.
 * ZigBee Specification r17, 3.2.2.3, NLME-NETWORK-FORMATION.confirm
 */
typedef struct _NWK_NetworkFormationConf_t
{
  enum nwk_msg_code cmdcode;
  /** The status of the request. */
  NWK_Status_t status;
} NWK_NetworkFormationConf_t;

/**//**
 * \brief NLME-NETWORK-FORMATION request primitive's parameters structure.
 * Zigbee Specification r17, 3.2.2.3 NLME-NETWORK-FORMATION.request.
 */
typedef struct _NWK_NetworkFormationReq_t
{
    enum nwk_msg_code cmdcode;
    
  /** Service fields - for internal needs. */
//  struct
//  { /** Link to a next network formation request in queue. */
//    QueueElement_t qelem;
//    union
//    {
//      struct
//      {
//        MAC_ScanReq_t req;
//        PHY_EnergyLevel_t energyList[MAX_EDSCAN_CHANNEL_AMOUNT];
//      } scan;
//      MAC_SetReq_t setReq;
//      MAC_StartReq_t startReq;
//      MAC_RxEnableReq_t rxEnableReq;
//    } mac;
//  } service;
  /** Channels are to be scanned in preparation for starting a network. */
  ChannelsMask_t scanChannels;
  /** The time spent scanning each channel is
   *  (aBaseSuperframeDuration * (2n + 1)) symbols,
   * where n is the value of the ScanDuration parameter. */
  ScanDuration_t scanDuration;
  bool coordRealignment;
  /** The beacon order of the network that the higher layers wish to form. */
  /* NwkBeaconOrder_t beaconOrder; */
  /* The superframe order of the network that the higher layers wish to form. */
  /* NwkBeaconOrder_t superFrameOrder; */
  /* Support battery life extension mode. This parameter is ignored if the
   * BeaconOrder parameter has a value of 15. */
  /* bool batteryLifeExtention; */
  /** NLME-NETWORK-FORMATION confirm callback function's pointer. */
 // void (*NWK_NetworkFormationConf)(NWK_NetworkFormationConf_t *conf);
 // NWK_NetworkFormationConf_t confirm;
} NWK_NetworkFormationReq_t;

/******************************************************************************
                               Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief NLME-NETWORK-FORMATION request primitive's prototype.

  \param[in] req - NLME-NETWORK-FORMATION request parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_NetworkFormationReq(NWK_NetworkFormationReq_t *const req);

#endif /* _NLME_NETWORK_FORMATION_H */
/** eof nlmeNetworkFormation.h */

