/**************************************************************************//**
  \file apsCommon.h

  \brief Declarations of common APS layer's constants and types.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-07-21 Max Gekk - Created.
    2010-10-18 Max Gekk - MAC and NWK status values are added.
   Last change:
    $Id: apsCommon.h 18682 2011-09-28 12:59:47Z mgekk $
 ******************************************************************************/
#if !defined _APS_COMMON_H
#define _APS_COMMON_H

/******************************************************************************
                                Includes section
 ******************************************************************************/
#include <appFramework.h>
#include <nwk1.h>
#include <macAddr.h>

/******************************************************************************
                               Definitions section
 ******************************************************************************/

#ifndef _LINK_SECURITY_ 
/** Max APS header size. */
#define APS_MAX_DATA_FRAME_HEADER_LENGTH  12U

/** Max APS footer size. */
#define APS_MAX_DATA_FRAME_FOOTER_LENGTH  0U
#else /* _LINK_SECURITY_ */


/**//**
 * \brief APS auxiliary header size.
 *
 * See ZigBee spec. r18, 4.5.1.1, Security Control Field.
 **/
#define APS_AUXILIARY_HEADER_LENGTH  (6U + 8U)

/** Max APS header size. */
#define APS_MAX_DATA_FRAME_HEADER_LENGTH  (12U + APS_AUXILIARY_HEADER_LENGTH)

/** Max APS footer size: 4 bytes - Message Integrity Code (MIC) */
#define APS_MAX_DATA_FRAME_FOOTER_LENGTH  4U

/** Universal extended address. */
#ifdef _UNIVERSAL_EXTENDED_ADDRESS_
    #define APS_UNIVERSAL_EXTENDED_ADDRESS 0xFFFFFFFFFFFFFFFBULL
#endif /* _UNIVERSAL_EXTENDED_ADDRESS_ */

#endif /* _LINK_SECURITY_ */


/** Max size of APS payload. */
#define APS_MAX_ASDU_SIZE \
  (NWK_MAX_NSDU_SIZE - APS_MAX_DATA_FRAME_HEADER_LENGTH - APS_MAX_DATA_FRAME_FOOTER_LENGTH)

/** Size of the reserved header in the data frame */
#define APS_ASDU_OFFSET \
  (NWK_NSDU_OFFSET + APS_MAX_DATA_FRAME_HEADER_LENGTH)

/** Size of the reserved part in the data frame. */
#define APS_AFFIX_LENGTH \
  (NWK_AFFIX_LENGTH + APS_MAX_DATA_FRAME_HEADER_LENGTH + APS_MAX_DATA_FRAME_FOOTER_LENGTH)

/** APS Broadcast Endpoint. */
#define APS_BROADCAST_ENDPOINT      0xff

/** Minimum value of user endpoint number. */
#define APS_MIN_USER_ENDPOINT       0x01

/** Maximum value of user endpoint number */
#define APS_MAX_USER_ENDPOINT       0xf0

/** ZDO Endpoint */
#define APS_ZDO_ENDPOINT            0x00

/** Reserved endpoints */
#define APS_MIN_RESERVED_ENDPOINT   0xf1
#define APS_MAX_RESERVED_ENDPOINT   0xfe

/** APS short address defines */
#define APS_MAX_UNICAST_SHORT_ADDRESS     0xfff7

#define APS_SM_INVALID_FRAME_COUNTER_VALUE  (0xffffffffUL)

#define APS_SM_BROADCAST_EXT_ADDRESS                     0xffffffffffffffffULL
#define APS_SM_BROADCAST_ALL_EXT_ADDRESS                 0xffffffffffffffffULL
#define APS_SM_BROADCAST_RX_ON_WHEN_IDLE_EXT_ADDRESS     0xfffffffffffffffdULL
#define APS_SM_BROADCAST_ALL_ROUTERS_EXT_ADDRESS         0xfffffffffffffffcULL
#define APS_SM_BROADCAST_LOW_POWER_ROUTERS_EXT_ADDRESS   0xfffffffffffffffbULL
#define APS_SM_UNIVERSAL_TRUST_CENTER_EXT_ADDRESS        0xfffffffffffffffaULL /*!< Universal trust center address*/
#define APS_LAST_UNICAST_ADDRESS                         0xfffffffffffffff9ULL 
#define APS_SM_INVALID_EXT_ADDRESS                       0x0000000000000000ULL

/* Special value of apsTrustCenterAddress indicates distrubuted trust center mode. */
#define APS_DISTRIBUTED_TC_ADDRESS APS_SM_BROADCAST_EXT_ADDRESS

/******************************************************************************
                                 Types section
 ******************************************************************************/
/**//**
 * \struct APS_Address_t apsCommon.h "aps.h"
 *
 * \brief Common APS Address type.
 *
 *  Used as the part of standard APS primitives
 **/
typedef struct
{
  union
  {
    ShortAddr_t shortAddress; /*!< 16-bit network address */
#ifdef _APS_MULTICAST_
    GroupAddr_t groupAddress; /*!< 16-bit group (multicast) address */
#endif
  };
  ExtAddr_t extAddress;     /*!< 64-bit extended IEEE address */
} APS_Address_t;

/**//**
 * \brief APS addressing modes.
 *
 *   Defines APS addressing modes used in some standard primitives.
 **/
typedef enum
{
  /** Indirect addressing mode via binding future. */
  APS_NO_ADDRESS = 0x00,
  /** Group addressing mode via multicast. */
  APS_GROUP_ADDRESS = 0x01,
  /** Unicast addressing mode by 16-bit network address. */
  APS_SHORT_ADDRESS = 0x02,
  /** Unicast addressing mode by 64-bit IEEE address. */
  APS_EXT_ADDRESS = 0x03
} APS_AddrMode_t;

/***//**
 * \brief APS status values.
 *
 *    APS status values used in the most part of standard primives except for
 *   the security service primitives where APS_SecurityStatus_t is used instead.
 **/
typedef enum
{
  /** A request has been executed successfully. */
  APS_SUCCESS_STATUS                     = 0x00,
  /** IEEE 802.15.4-2006, Table 83. */
  /** Valid values of the Association Status field. */
  APS_MAC_PAN_AT_CAPACITY_STATUS         = 0x01,
  APS_MAC_PAN_ACCESS_DENIED_STATUS       = 0x02,
  /** A transmit request failed since the ASDU is too large and fragmentation
   * is not supported. */
  APS_ASDU_TOO_LONG_STATUS               = 0xa0,
  /** A received fragmented frame could not be defragmented
   * at the current time. */
  APS_DEFRAG_DEFERRED_STATUS             = 0xa1,
  /** A received fragmented frame could not be defragmented since the device
   * does not support fragmentation. */
  APS_DEFRAG_UNSUPPORTED_STATUS          = 0xa2,
  /** A parameter value was out of range. */
  APS_ILLEGAL_REQUEST_STATUS             = 0xa3,
  /** An APSME-UNBIND.request failed due to the requested binding link not
   * existing in the binding table. */
  APS_INVALID_BINDING_STATUS             = 0xa4,
  /** An APSME-REMOVE-GROUP.request has been issued with a group identifier
   * that does not appear in the group table. */
  APS_INVALID_GROUP_STATUS               = 0xa5,
  /** A parameter value was invalid or out of range. */
  APS_INVALID_PARAMETER_STATUS           = 0xa6,
  /** An APSDE-DATA.request requesting acknowledged transmission failed due
   * to no acknowledgement being received. */
  APS_NO_ACK_STATUS                      = 0xa7,
  /** An APSDE-DATA.request with a destination addressing mode set to 0x00
   * failed due to there being no devices bound to this device. */
  APS_NO_BOUND_DEVICE_STATUS             = 0xa8,
  /** An APSDE-DATA.request with a destination addressing mode
   * set to 0x03 failed due to no corresponding short address found
   * in the address map table. */
  APS_NO_SHORT_ADDRESS_STATUS            = 0xa9,
  /** An APSDE-DATA.request with a destination addressing mode set to 0x00
   * failed due to a binding table not being supported on the device. */
  APS_NOT_SUPPORTED_STATUS               = 0xaa,
  /** An ASDU was received that was secured using a link key. */
  APS_SECURED_LINK_KEY_STATUS            = 0xab,
  /** An ASDU was received that was secured using a network key. */
  APS_SECURED_NWK_KEY_STATUS             = 0xac,
  /** An APSDE-DATA.request requesting security has resulted in an error
   * during the corresponding security processing. */
  APS_SECURITY_FAIL_STATUS               = 0xad,
  /** An APSME-BIND.request or APSME.ADDGROUP.request issued when the binding
   * or group tables, respectively, were full. */
  APS_TABLE_FULL_STATUS                  = 0xae,
  /** An ASDU was received without any security. */
  APS_UNSECURED_STATUS                   = 0xaf,
  /** An APSME-GET.request or APSMESET.request has been issued with an
   * unknown attribute identifier. */
  APS_UNSUPPORTED_ATTRIBUTE_STATUS       = 0xb0,
  /** An ASDU was received that was secured using the trust center link key. */
  APS_SECURED_TRUST_CENTER_LINK_KEY_STATUS          = 0xb1,
  /** An ASDU was received that was secured using
   * hash ofthe trust center link key. */
  APS_SECURED_HASH_OF_TRUST_CENTER_LINK_KEY_STATUS  = 0xb2,
  /** An invalid or out-of-range parameter has been passed to a primitive from
   * the next higher layer. */
  APS_NWK_INVALID_PARAMETERS_STATUS      = 0xC1,
  /** The next higher layer has issued a request that is invalid or cannot be
   * executed given the current state of the NWK layer. */
  APS_NWK_INVALID_REQUEST_STATUS         = 0xC2,
  /** An NLME-JOIN.request has been disallowed. */
  APS_NWK_NOT_PERMITTED_STATUS           = 0xC3,
  /** An NLME-NETWORK-FORMATION.request has failed to start a network. */
  APS_NWK_STARTUP_FAILURE_STATUS         = 0xC4,
  /** A device with the address supplied to the NLMEDIRECT-JOIN.request is
   * already present in the neighbor table of the device on which
   * the NLME-DIRECT-JOIN.request was issued. */
  APS_NWK_ALREADY_PRESENT_STATUS         = 0xC5,
  /** Used to indicate that an NLME-SYNC.request has failed at the MAC layer. */
  APS_NWK_SYNC_FAILURE_STATUS            = 0xC6,
  /** An NLME-JOIN-DIRECTLY.request has failed because there is no more room in
   * the neighbor table. */
  APS_NWK_NEIGHBOR_TABLE_FULL_STATUS     = 0xC7,
  /** An NLME-LEAVE.request has failed because the device addressed in
   * the parameter list is not in the neighbor table of the issuing device. */
  APS_NWK_UNKNOWN_DEVICE_STATUS          = 0xC8,
  /** An NLME-GET.request or NLME-SET.request has been issued with
   * an unknown attribute identifier. */
  APS_NWK_UNSUPPORTED_ATTRIBUTE_STATUS   = 0xC9,
  /** An NLME-JOIN.request has been issued in an environment
   * where no networks are detectable. */
  APS_NWK_NO_NETWORKS_STATUS             = 0xCA,
  /** Security processing has been attempted on an outgoing frame,
   * and has failed because the frame counter has reached its maximum value. */
  APS_NWK_MAX_FRM_COUNTER_STATUS         = 0xCC,
  /** Security processing has been attempted on an outgoing frame,
   * and has failed because no key was available with which to process it. */
  APS_NWK_NO_KEY_STATUS                  = 0xCD,
  /** Security processing has been attempted on an outgoing frame,
   * and has failed because the security engine produced erroneous output. */
  APS_NWK_BAD_CCM_OUTPUT_STATUS          = 0xCE,
  /** An attempt to discover a route has failed due to a lack of routing table
   * or discovery table capacity. */
  APS_NWK_NO_ROUTING_CAPACITY_STATUS     = 0xCF,
  /** An attempt to discover a route has failed due to a reason other
   * than a lack of routing capacity. */
  APS_NWK_ROUTE_DISCOVERY_FAILED_STATUS  = 0xD0,
  /** An NLDE-DATA.request has failed due to a routing
   * failure on the sending device. */
  APS_NWK_ROUTE_ERROR_STATUS             = 0xD1,
  /** An attempt to send a broadcast frame or member mode multicast has failed
   * due to the fact that there is no room in the BTT. */
  APS_NWK_BT_TABLE_FULL_STATUS           = 0xD2,
  /** An NLDE-DATA.request has failed due to insufficient buffering available.
   **/
  APS_NWK_FRAME_NOT_BUFFERED_STATUS      = 0xD3,
  /** IEEE 802.15.4-2006, Table 78 MAC enumerations description. */
  /** The frame counter purportedly applied by the originator of the
   * received frame is invalid. */
  APS_MAC_COUNTER_ERROR_STATUS           = 0xDB,
  /** The key purportedly applied by the originator of the received frame is
   * not allowed to be used with that frame type according to the key usage
   * policy of the recipient. */
  APS_MAC_IMPROPER_KEY_TYPE_STATUS       = 0xDC,
  /** The security level purportedly applied by the originator of the received
   * frame does not meet the minimum security level required/expected by
   * the recipient for that frame type. */
  APS_MAC_IMPROPER_SECURITY_LEVEL_STATUS = 0xDD,
  /** The received frame was purportedly secured using security based on
   * IEEE Std 802.15.4-2003, and such security is not supported by this standard.
   **/
  APS_MAC_UNSUPPORTED_LEGACY_STATUS      = 0xDE,
  /** The security purportedly applied by the originator of the received frame
   * is not supported. */
  APS_MAC_UNSUPPORTED_SECURITY_STATUS    = 0xDF,
  /** The beacon was lost following a synchronization request. */
  APS_MAC_BEACON_LOSS_STATUS             = 0xE0,
  /** A transmission could not take place due to activity on the channel,
   * i.e., the CSMA-CA mechanism has failed. */
  APS_MAC_CHANNEL_ACCESS_FAILURE_STATUS  = 0xE1,
  /** The GTS request has been denied by the PAN coordinator. */
  APS_MAC_DENIED_STATUS                  = 0xE2,
  /** The attempt to disable the transceiver has failed. */
  APS_MAC_DISABLE_TRX_FAILURE_STATUS     = 0xE3,
  /** Either a frame resulting from processing has a length that is
   * greater than aMaxPHYPacketSize or a requested transaction is
   * too large to fit in the CAP or GTS. */
  APS_MAC_FRAME_TOO_LONG_STATUS          = 0xE5,
  /** The requested GTS transmission failed because the specified
   * GTS either did not have a transmit GTS direction or was not defined. */
  APS_MAC_INVALID_GTS_STATUS             = 0xE6,
  /** A request to purge an MSDU from the transaction queue was made using
   * an MSDU handle that was not found in the transaction table. */
  APS_MAC_INVALID_HANDLE_STATUS          = 0xE7,
  /** A parameter in the primitive is either not supported or is out of
   * the valid range. */
  APS_MAC_INVALID_PARAMETER_STATUS       = 0xE8,
  /** No acknowledgment was received after macMaxFrameRetries. */
  APS_MAC_NO_ACK_STATUS                  = 0xE9,
  /** A scan operation failed to find any network beacons. */
  APS_MAC_NO_BEACON_STATUS               = 0xEA,
  /** No response data were available following a request. */
  APS_MAC_NO_DATA_STATUS                 = 0xEB,
  /** The operation failed because a 16-bit short address was not allocated. */
  APS_MAC_NO_SHORT_ADDRESS_STATUS        = 0xEC,
  /** A receiver enable request was unsuccessful because it could not be
   * completed within the CAP. */
  APS_MAC_OUT_OF_CAP_STATUS              = 0xED,
  /** A PAN identifier conflict has been detected and communicated
   * to the PAN coordinator. */
  APS_MAC_PAN_ID_CONFLICT_STATUS         = 0xEE,
  /** A coordinator realignment command has been received. */
  APS_MAC_REALIGNMENT_STATUS             = 0xEF,
  /** The transaction has expired and its information was discarded. */
  APS_MAC_TRANSACTION_EXPIRED_STATUS     = 0xF0,
  /** There is no capacity to store the transaction. */
  APS_MAC_TRANSACTION_OVERFLOW_STATUS    = 0xF1,
  /** The transceiver was in the transmitter enabled state when the receiver
   * was requested to be enabled. */
  APS_MAC_TX_ACTIVE_STATUS               = 0xF2,
  /** The key purportedly used by the originator of the received frame is
   * not available or, if available, the originating device is not known
   * or is blacklisted with that particular key. */
  APS_MAC_UNAVAILABLE_KEY_STATUS         = 0xF3,
  /** A SET/GET request was issued with the identifier of a PIB
   * attribute that is not supported. */
  APS_MAC_UNSUPPORTED_ATTRIBUTE_STATUS   = 0xF4,
  /** A request to send data was unsuccessful because neither the source address
   * parameters nor the destination address parameters were present. */
  APS_MAC_INVALID_ADDRESS_STATUS         = 0xF5,
  /** A receiver enable request was unsuccessful because it specified a number
   * of symbols that was longer than the beacon interval. */
  APS_MAC_ON_TIME_TOO_LONG_STATUS        = 0xF6,
  /** A receiver enable request was unsuccessful because it could not be
   * completed within the current superframe and was not permitted to be
   * deferred until the next superframe. */
  APS_MAC_PAST_TIME_STATUS               = 0xF7,
  /** The device was instructed to start sending beacons based on the
   * timing of the beacon transmissions of its coordinator, but the device
   * is not currently tracking the beacon of its coordinator. */
  APS_MAC_TRACKING_OFF_STATUS            = 0xF8,
  /** An attempt to write to a MAC PIB attribute that is in a table failed
   * because the specified table index was out of range. */
  APS_MAC_INVALID_INDEX_STATUS           = 0xF9,
  /** There are some unscanned channels yet, but there is no memory */
  APS_MAC_LIMIT_REACHED_STATUS           = 0xFA,
  /** A SET/GET request was issued with the identifier of an attribute
   * that is read only. */
  APS_MAC_READ_ONLY_STATUS               = 0xFB,
  /** A request to perform a scan operation failed because the MLME was
   * in the process of performing a previously initiated scan operation. */
  APS_MAC_SCAN_IN_PROGRESS_STATUS        = 0xFC,
  /** The device was instructed to start sending beacons based on the timing of
   * the beacon transmissions of its coordinator, but the instructed start time
   * overlapped the transmission time of the beacon of its coordinator. */
  APS_MAC_SUPERFRAME_OVERLAP_STATUS      = 0xFD
} APS_Status_t;

#if defined _SECURITY_
/** Type of request to transmit APS command. */
typedef struct _ApsCommandReq_t
{
  /** \cond SERVICE_FIELDS **/
  struct
  {
    /** Service field, used for queue support */
    QueueElement_t next;
    /** Partner device short address. */
    ShortAddr_t partnerShortAddr;
    /** Service structure for memory allocation on NWK-layer */
    NWK_AllocDataReq_t allocateReq;
    /** Callback function for preparing an NWK-DATA.request. */
    void (*prepareNwkDataReq)(struct _ApsCommandReq_t *const commandReq);
#if defined _LINK_SECURITY_ 
    /** Service structure for payload encryption */
    SSP_EncryptFrameReq_t encryptReq;
#endif /* _LINK_SECURITY_ */

  } service;
  /** Flag indicates the command frame encryption on NWK-layer. */
  bool nwkSecurity;
  /** Flag indicates the command frame encryption on APS-layer. */
  bool apsSecurity;
  /** Identifies the type of the key which is used to protect the frame. */
  uint8_t keyIdentifier;
  /** If the extendedNonce field is equal to 1 then source address shall
   * be present in Auxilary header. */
  bool extendedNonce;
  /** Actual command frame lenght, APS header with payload */
  uint8_t frameLength;
  /** Pointer to extended address of partner device. */
  ExtAddr_t *partnerExtAddr;
  /** Callback function for preparing an APS command. */
  void (*prepareCommand)(struct _ApsCommandReq_t *const commandReq);
  /** Callback function for confirmation of APS command transmission. */
  void (*confirmCommand)(struct _ApsCommandReq_t *const commandReq,
    const APS_Status_t status);
} ApsCommandReq_t;
#endif /* _SECURITY_ */

#endif /* _APS_COMMON_H */
/** eof apsCommon.h */

