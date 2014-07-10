/**************************************************************************//**
  \file nldeData.h

  \brief NLDE-DATA interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-13 Max Gekk - Created.
   Last change:
    $Id: nldeData.h 18405 2011-08-31 15:42:23Z mgekk $
 ******************************************************************************/
#if !defined _NLDE_DATA_H
#define _NLDE_DATA_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <types1.h>
#include <appFramework.h>
#include <mac.h>
#include <nwkCommon.h>
#include <nwkNeighbor.h>
#include <nlmeRouteDiscovery.h>
#if defined _SECURITY_
#include <sspSfp.h>
#endif /* _SECURITY_ */

#include "nwk_msg_const.h"
/******************************************************************************
                                Types section
 ******************************************************************************/
/**//**
 * \brief NLDE-DATA confirm primitive's parameters structure declaration.
 * ZigBee Specification r17, 3.2.2.2 NLDE-DATA.confirm, page 264.
 */
typedef struct _NWK_DataConf_t
{
  enum nwk_msg_code cmdcode;
  /* Service fields - for internal needs. */
//  struct
//  {
//    QueueElement_t qelem;
//  } service;

  /** The status of the corresponding request. */
  NWK_Status_t status;
  /** The handle associated with the NSDU being confirmed. */
  void *nsduHandle;
  /** A time indication for the transmitted packet based on the local clock. */
  uint32_t txTime;
  /** Quantity (or approximate quantity, it depends on routing method) of hops between source and destination nodes */
  NwkRadius_t hops;
} NWK_DataConf_t;

/** Transmission delay types. */
typedef enum _NwkTxDelayType_t
{
  NWK_TX_DELAY_UNICAST_DATA = 0x00,
  NWK_TX_DELAY_UNICAST_COMMAND = 0x01,
  NWK_TX_DELAY_LINK_STATUS = 0x02,
  NWK_TX_DELAY_INITIAL_ROUTE_REQUEST = 0x03,
  NWK_TX_DELAY_INITIAL_MANYTOONE_ROUTE_REQUEST = 0x04,
  NWK_TX_DELAY_ROUTE_REQUEST = 0x05,
  NWK_TX_DELAY_MANYTOONE_ROUTE_REQUEST = 0x06,
  NWK_TX_DELAY_BROADCAST = 0x07,
  NWK_TX_DELAY_BROADCAST_SUCCESS = 0x08,
  NWK_TX_DELAY_MULTICAST = 0x09,
  NWK_TX_DELAY_MULTICAST_SUCCESS = 0x0A,
  NWK_TX_DELAY_INITIAL_MULTICAST= 0x0B,
  NWK_TX_DELAY_INITIAL_ROUTE_RECORD = 0x0C,
  NWK_TX_DELAY_TRANSIT_DATA = 0x0D,
  NWK_LAST_TX_DELAY
} NwkTxDelayType_t;

/** Further behavior after delay. */
typedef enum _NwkTxDelayStatus_t
{
  NWK_TX_NOW = 0x0,             /**< Immediately to send. */
  NWK_TX_DONE = 0x1,            /**< To complete transmission. */
  NWK_TX_DELAY_REQUIRED = 0x2,  /**< Delay before transmission. */
  NWK_TX_INDIRECT = 0x3         /**< Indirect transmission to all children. */
} NwkTxDelayStatus_t;

/** Delay request's parameters. */
typedef struct _NwkTxDelayReq_t
{
  /* Service fields - for internal needs. */
  struct
  { /** Link to a next output packet in queue. */
    QueueElement_t qelem;
  } service;

  NwkTxDelayType_t type;
  uint8_t attempt;
  uint16_t delay;
  NwkTxDelayStatus_t actionAfterDelay;
} NwkTxDelayReq_t;

/* Routing methods, used for data packet transmission. */
typedef enum _NwkRoutingMethod_t
{
  NWK_ROUTING_UNKNOWN,
  NWK_ROUTING_FAIL,
  NWK_ROUTING_INDIRECT,
  NWK_ROUTING_TO_PARENT,
  NWK_ROUTING_TO_NEIGHBOR,
  NWK_ROUTING_TREE,
  NWK_ROUTING_MESH,
  NWK_ROUTING_MANYTOONE,
  NWK_ROUTING_BROADCAST,
  NWK_ROUTING_DIRECT,
  NWK_ROUTING_MANYTOONE_RECORD
} NwkRoutingMethod_t;

/* Information about routing method and quantity of hops to destination node */
typedef struct _NwkRouting_t
{
  NwkRoutingMethod_t method;
  NwkRadius_t hops;
} NwkRouting_t;

/**//**
 * \brief Meta-information of output packet.
 */
typedef struct _NwkOutputPacket_t
{
  /* Service fields - for internal needs. */
  struct
  { /** Link to a next output packet in queue. */
    QueueElement_t qelem;
  } service;

  /** Identifier of transmission request. */
  uint8_t txId;
  /** Transmission context. */
  void *context;
  /** pointer to the destination neighbor. */
  NwkNeighbor_t *neighbor;
  /** Used routing method for transmission and quantity of hops to destination node. */
  NwkRouting_t routingInfo;
  /** Address of node from which the packet has been received. */
  ShortAddr_t prevHopAddr;
  /** Flag indicates that secured packet must be decrypted or not. */
  bool decryptRequired;
  /** MCPS-DATA request primitive's parameters structure. */
  MAC_DataReq_t macDataReq;
  union
  {
    /** NLME-ROUTE-DISCOVERY request primitive's parameters structure. */
    NWK_RouteDiscoveryReq_t routeDiscovery;
    /** Transmission delay parameters. */
    NwkTxDelayReq_t txDelay;
#if defined _NWK_ROUTE_RECORD_
    /** Internal variables of the route record component. */
    struct
    {/** Extended address of node from which an original packet
      * has been received. */
      ExtAddr_t from;
    } routeRecord;
#endif /* _NWK_ROUTE_RECORD_ */
#if defined _SECURITY_
    /** Encrypt Frame primitive's parameters structure. */
    SSP_EncryptFrameReq_t encrypt;
    /** Decrypt Frame primitive's parameters structure. */
    SSP_DecryptFrameReq_t decrypt;
#endif /* _SECURITY_ */
  } req;
#if defined _SYS_ASSERT_ON_
  uint8_t state;
#endif
} NwkOutputPacket_t;

/**//**
 * \brief NLDE-DATA request primitive's parameters structure declaration.
 * Zigbee Specification r17, 3.2.2.1 NLDE-DATA.request, page 261.
 */
typedef struct _NWK_DataReq_t
{
  enum nwk_msg_code cmdcode;

  /* Service fields - for internal needs. */
//  struct
//  {
//    QueueElement_t qelem; /**< link used for queuing */
//    NwkOutputPacket_t outPkt;
//  } service;

  /* Public fields. */
  /** The type of destination address supplied by the DstAddr parameter. */
  NWK_DstAddrMode_t dstAddrMode;
  /** Destination address. */
  ShortAddr_t dstAddr;
  /** The number of octets comprising the NSDU to be transferred. */
  NwkLength_t nsduLength;
  /** The handle associated with the NSDU to be transmitted by the NWK layer. */
  void *nsduHandle;
  /** The distance, in hops, that a frame will be allowed
   * to travel through the network. */
  NwkRadius_t radius;
  /** The distance, in hops, that a multicast frame will be relayed by nodes
   * not a member of the group. A value of 0x07 is treated as infinity.*/
  NwkRadius_t nonmemberRadius;
  /** The DiscoverRoute parameter may be used to control route discovery
   * operations for the transit of this frame: false = suppress route discovery,
   * true = enable route discovery */
  bool discoverRoute;
  /** Encrypt packet by network key before transmission. */
  uint8_t securityEnable;

  app_route_method_t routeType;
 // NwkFrameHeader_t NwkFrameHeader;
  /** The set of octets comprising the NSDU to be transferred. */
  uint8_t *nsdu;
  
  /** Decrypt packet after transmission or not. This is recommendation for NWK layer.
   * NWK layer may set this flag to 'true' if packet was decrypted. */
  //bool decryptRequired;
  /** Send to self a) broadcast, if destination address is matched to device type,
   * b) multicast, if device is in group or c) unicast, if destination address
   * is short address of this device. If this flag is 'false' then NWK-layer never
   * sends to self otherwise it sends to self only conditions a)-c) are true.
   * Before call NWK_DataConf this flag is set to true if NWK_DataInd will be called.
   * */
 // bool boomerang;
  /** NLDE-DATA confirm callback function's pointer. */
 // void (*NWK_DataConf)(NWK_DataConf_t *conf);
//  NWK_DataConf_t confirm;
} NWK_DataReq_t;

/** Service information for an outgoing external packet. */
typedef NWK_DataReq_t NwkExternPacket_t;

/** Nonstandard NLDE-DATA.response */
typedef struct _NWK_DataResp_t
{
  uint8_t status;
} NWK_DataResp_t;

/**//**
 * \brief NLDE-DATA indication primitive's parameters structure declaration.
 * Zigbee Specification r17, 3.2.2.3 NLDE-DATA.indication, page 266.
 */
typedef struct _NWK_DataInd_t
{
  enum nwk_msg_code cmdcode;
  
  /* Service fields - for internal needs. */
//  struct
//  {
//    QueueElement_t qelem; /**< link used for queuing */
//  } service;

  /** The type of destination address supplied by the DstAddr parameter.
   * This may have one of the following two values:
   * - 0x01 = 16-bit multicast group address,
   * - 0x02 = 16-bit network address of a device or a broadcast address. */
  NWK_DstAddrMode_t dstAddrMode;
  /** The destination address to which the NSDU was sent. */
  ShortAddr_t dstAddr;
  /** The individual device address from which the NSDU originated. */
  ShortAddr_t srcAddr;
  /** The short node address from which the NSDU received. */
  ShortAddr_t prevHopAddr;
  /** The number of octets comprising the NSDU being indicated. */
  NwkLength_t nsduLength;
  /** The set of octets comprising the NSDU being indicated. */
  uint8_t *nsdu;
  /** The link quality indication delivered by the MAC on receipt of this frame
   * as a parameter of the MCPS-DATA.indication primitive. */
  Lqi_t linkQuality;
  /** RSSI delivered by the MAC on receipt of this frame as a parameter of
   * the MCPS-DATA.indication primitive.*/
  Rssi_t rssi;
  /** A time indication for the received packet based on the local clock. */
  uint32_t rxTime;
  /** An indication of whether the received data frame is using security. */
  bool securityUse;
  
  uint8_t radius;
  /** The flag indicates that the packet was sent through the loop. */
 // bool boomerang;
  /** NLDE-DATA response callback function's pointer. */
 // void (*NWK_DataResp)(NWK_DataResp_t *resp);
 // NWK_DataResp_t response;
} NWK_DataInd_t;

/**//**
 * \brief Confirmation parameters of NWK Data allocation.
 */
typedef struct _NWK_AllocDataConf_t
{
  NWK_DataReq_t *nwkDataReq;
} NWK_AllocDataConf_t;

/**//**
 * \brief Parameters of NWK Data allocation request.
 */
typedef struct _NWK_AllocDataReq_t
{
  /* Service fields - for internal needs. */
  struct
  {
    QueueElement_t qelem; /**< link used for queuing */
  } service;

  /** The number of allocated octets in the NSDU. */
  NwkLength_t nsduLength;
  /** Pointer to confirm callback function. */
  void (*NWK_AllocDataConf)(NWK_AllocDataConf_t *conf);
  NWK_AllocDataConf_t confirm;
} NWK_AllocDataReq_t;

/******************************************************************************
                               Define(s) section
 ******************************************************************************/
/** Get length of output packet. */
#define NWK_GET_OUTPKT_LEN(outPkt) ((outPkt)->macDataReq.msduLength)
/** Set length of output packet. */
#define NWK_SET_OUTPKT_LEN(outPkt, len) (outPkt)->macDataReq.msduLength = (len)
#define NWK_SET_INVALID_OUTPKT_LEN(outPkt) \
  NWK_SET_OUTPKT_LEN(outPkt, MAC_MAX_MSDU_SIZE + 1U)
#define NWK_IS_VALID_OUTPKT_LEN(outPkt) \
  (NWK_GET_OUTPKT_LEN(outPkt) <= MAC_MAX_MSDU_SIZE)
/** Add value to length of output packet. */
#define NWK_ADD_TO_OUTPKT_LEN(outPkt, value) \
  (outPkt)->macDataReq.msduLength += (value)
/** Subtract value from length of output packet. */
#define NWK_SUB_FROM_OUTPKT_LEN(outPkt, value) \
  (outPkt)->macDataReq.msduLength -= (value)
/** Get pointer to data (NWK header + payload) of output packet. */
#define NWK_GET_OUTPKT_DATA(outPkt) ((void *)((outPkt)->macDataReq.msdu))
/** Initialize msdu pointer. */
#define NWK_SET_OUTPKT_DATA(outPkt, ptr) \
  (outPkt)->macDataReq.msdu = (uint8_t *)(ptr)

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief NLDE-DATA request primitive's prototype.

  \param[in] req - NLDE-DATA request parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_DataReq(NWK_DataReq_t *const req);

/**************************************************************************//**
  \brief NLDE-DATA indication primitive's prototype.

  \param[in] ind - NLDE-DATA indication parameters' structure pointer.
  \return None.
 ******************************************************************************/
extern void NWK_DataInd(NWK_DataInd_t *ind);

/**************************************************************************//**
  \brief The request to send own data packet to itself.

  \param[in] req - pointer to parameters of NLDE-DATA.request primitive.
  \return None.
 ******************************************************************************/
void NWK_LoopbackTxReq(NWK_DataReq_t *const req);

#if defined _INTERPAN_
/**************************************************************************//**
  \brief INTRP-DATA.ind primitive handler.

  It is intended to notify APS Layer about incoming Inter-PAN packet.

  \param[in] ind - NLDE-DATA indication parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_IntrpDataInd(MAC_DataInd_t *ind);
#endif /*_INTERPAN_*/
#if defined _NWK_ALLOCATOR_
/**************************************************************************//**
  \brief Allocate memory for NWK_DataReq_t structure and MPDU.

  \param[in] req - pointer to allocation parameters' structure.
  \return None.
 ******************************************************************************/
void NWK_AllocDataReq(NWK_AllocDataReq_t *const req);

/**************************************************************************//**
  \brief Free memory which is allocated for NWK_DataReq_t structure and MPDU.

  \param[in] req - NLDE-DATA request parameters' structure pointer.
  \return None.
 ******************************************************************************/
void NWK_FreeDataReq(NWK_DataReq_t *const req);

/**************************************************************************//**
  \brief Transform pointer to NWK_DataInd_t to pointer to NWK_DataReq_t.

    This function initializes nsdu and nsduLength by appropriate field
  from NWK_DataInd_t structure.

  \param[in] ind - NLDE-DATA indication parameters' structure pointer.
  \param[in] secure - if 'true' then encryption on NWK will be used for
                      NLDE-DATA.request otherwise 'false'.
  \return Pointer to NLDE-DATA.request's parameters.
 ******************************************************************************/
NWK_DataReq_t* NWK_DataTransformIndToReq(NWK_DataInd_t *ind, const bool secure);

#endif /* _NWK_ALLOCATOR_ */
#endif /* _NLDE_DATA_H */
/** eof nldeData.h */

