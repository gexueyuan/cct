/**************************************************************************//**
  \file nlmeJoin.h

  \brief NLME-JOIN interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-11 Max Gekk - Created.
   Last change:
    $Id: nlmeJoin.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NLME_JOIN_H
#define _NLME_JOIN_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <mac.h>
#include <nwkCommon.h>
#include  "nwk_msg_const.h"

/******************************************************************************
                                Types section
 ******************************************************************************/
/**//**
 * \brief NLME-JOIN confirm primitive's parameters structure declaration.
 * ZigBee Specification r17, 3.2.2.13 NLME-JOIN.confirm, page 285.
 */
typedef struct _NWK_JoinConf_t
{
  enum nwk_msg_code cmdcode;
  /** The status of the corresponding request. Valid Range: NWK_SUCCESS_STATUS,
   * NWK_NOT_PERMITTED_STATUS, NWK_NO_NETWORKS_STATUS or any status value
   * returned from MLME-ASSOCIATE.confirm, MLME-SET.confirm, MLME-GET.confirm,
   * MLME-RX-ENABLE.confirm, NLME-NETWORK-DISCOVERY.confirm. */
  NWK_Status_t status;
  /** The 16-bit network address that was allocated to this device. This parameter
   * will be equal to 0xffff if the join attempt was unsuccessful. */
  ShortAddr_t networkAddress;
  /** The 64-bit extended PAN identifier for the network of which the device is
   * now a member. */
  ExtPanId_t extendedPANId;
  /** The value of phyCurrentChannel attribute of the PHY PIB, which is equal to
   * the current channel of the network that has been joined. */
  Channel_t activeChannel;
} NWK_JoinConf_t;

/** The method of joining the network. */
typedef enum _NWK_RejoinNetwork_t
{
  /** The device is requesting to join a network through association. */
  NWK_REJOIN_ASSOCIATION = 0,
  /** The device is joining directly or rejoining the network using
   * the orphaning procedure. */
  NWK_REJOIN_ORPHAN = 1,
  /** The device is joining the network using the NWK rejoining procedure. */
  NWK_REJOIN_NWK_REJOIN = 2,
  /** The device is to change the operational network channel to that
   * identified in the ScanChannels parameter. */
  NWK_REJOIN_CHANGE_CHANNEL = 3,
  /** The device is joining to a network with known parameters. Device's own
   * parameters are also known. This rejoin won't be visible on the air. */
  NWK_REJOIN_SILENT = 4,
} NWK_RejoinNetwork_t;

/**//**
 * \brief NLME-JOIN request primitive's parameters structure declaration.
 * Zigbee Specification r17, 3.2.2.11 NLME-JOIN.request, page 281.
 */
typedef struct _NWK_JoinReq_t
{

  enum nwk_msg_code cmdcode;

  /** Service fields - for internal needs. */
//  struct
//  {
//    QueueElement_t qelem; /**< link used for queuing */
//  } service;
  /** The 64-bit PAN identifier of the network to join.*/
  ExtPanId_t extendedPANId;
  /** This parameter controls the method of joining the network. */
  NWK_RejoinNetwork_t rejoinNetwork;
  /** The 27 least significant bits (b0, b1,... b26) indicate which channels
   * are to be scanned (1=scan, 0=do not scan) for each of the 27 valid channels.
   **/
  ChannelsMask_t scanChannels;
  /** A value used to calculate the length of time to spend scanning each
   * channel. The time spent scanning each channel is
   *        (aBaseSuperframeDuration * (2^n + 1)) symbols,
   * where n is the value of the ScanDuration parameter. */
  ScanDuration_t scanDuration;
  /** The operating capabilities of the device being directly joined. */
  /** This parameter should be set in NWK_ResetReq. */
  /* MAC_CapabilityInf_t capabilityInformation; */
  /** If the value of RejoinNetwork is 0x02 and this is TRUE than the device
   * will try to rejoin securely. Otherwise, this is set to FALSE. */
  bool securityEnable;
  /** NLME-JOIN confirm callback function's pointer. */
 // void (*NWK_JoinConf)(NWK_JoinConf_t *conf);
 // NWK_JoinConf_t confirm;
} NWK_JoinReq_t;

/**//**
 * \brief NLME-JOIN indication primitive's parameters structure declaration.
 * Zigbee Specification r17, 3.2.2.12 NLME-JOIN.indication, page 283.
 */
typedef struct _NWK_JoinInd_t
{

  enum nwk_msg_code cmdcode;

  /** The network address of an entity that has been added to the network. */
  ShortAddr_t networkAddress;
  /** The 64-bit IEEE address of an entity that has been added to the network.*/
  ExtAddr_t extendedAddress;
  /** Specifies the operational capabilities of the joining device. */
  MAC_CapabilityInf_t capabilityInformation;
  /** The RejoinNetwork parameter indicating the method
   * used to join the network. */
  NWK_RejoinNetwork_t rejoinNetwork;
  /** This parameter will be TRUE if the rejoin was performed in a secure manner.
   * Otherwise, this parameter will be FALSE. */
  bool secureJoin;
  bool isRejoin;
} NWK_JoinInd_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief NLME-JOIN request primitive's prototype.

  \param[in] req - NLME-JOIN request parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_JoinReq(NWK_JoinReq_t *const req);

/**************************************************************************//**
  \brief NLME-JOIN indication primitive's prototype.

  \param[in] ind - NLME-JOIN indication parameters' structure pointer.
  \return None.
 ******************************************************************************/
extern void NWK_JoinInd(NWK_JoinInd_t *ind);

#endif /* _NLME_JOIN_H */
/** eof nlmeJoin.h */

