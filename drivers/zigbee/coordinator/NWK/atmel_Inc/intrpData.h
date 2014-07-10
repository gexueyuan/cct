/**************************************************************************//**
  \file  intrpData.h

  \brief Interface of inter-PAN transmission.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-02-18 A. Kostyukov - Created.
    2010-11-20 Max Gekk - Refactoring.
   Last change:
    $Id: intrpData.h 18271 2011-08-18 12:07:56Z nfomin $
 ******************************************************************************/
#if !defined _INTRP_DATA_H
#define _INTRP_DATA_H
/**//**
 *
 *  Inter-PAN is a transmission mechanism whereby ZigBee devices can perform
 * limited, insecure, and possibly anonymous exchanges of information with
 * devices in their local neighborhood without having to form or join the same
 * ZigBee network. See ZigBee SE Profile Spec r15, page 81.
 *
 **/

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <apsCommon.h>

#if defined _INTERPAN_

/******************************************************************************
                    Definitions section
******************************************************************************/
#define INTRP_NO_PANID_COMPRESSION_TXOPTION 1u

/******************************************************************************
                                Types section
 ******************************************************************************/
/** Type of handle associated with the inter-PAN ASDU to be transmitted. */
typedef uint8_t IntrpAsduHandle_t;
/** Type of inter-PAN frame length. */
typedef uint8_t IntrpLength_t;

/**//**
 * \struct INTRP_DataConf_t intrpData.h "aps.h"
 *
 * \brief INTRP-DATA confirm primitive's parameters structure declaration.
 *
 *  ZigBee Smart Energy profile specification r15, B.3.2.
 * The INTRP-DATA.confirm Primitive, page 85.
 **/
typedef struct
{
  /** An integer handle associated with the transmitted frame. */
  IntrpAsduHandle_t  asduHandle;
  /** The status of the ASDU transmission corresponding to ASDUHandle as
   * returned by the MAC.*/
  MAC_Status_t  status;
} INTRP_DataConf_t;

typedef uint8_t INTRP_TxOptions_t;

/**//**
 * \struct INTRP_DataReq_t intrpData.h "aps.h"
 *
 * \brief INTRP-DATA.request primitive's parameters structure declaration.
 *
 *  ZigBee Smart Energy profile specification r15, B.3.1
 * The INTRP-DATA.request Primitive, page 83.
 **/
typedef struct
{
  /** \cond SERVICE_FIELDS **/
  struct
  {
    /** MCPS-DATA request primitive's parameters structure. */
    MAC_DataReq_t  macDataReq;
  } service;
  /** \endcond **/

  /** The addressing mode for the destination address used in this primitive.
   *  This parameter can take one of the values from the following list:
   *    \li 0x01 = 16-bit group address
   *    \li 0x02 = 16-bit NWK address, normally  the broadcast address 0xffff
   *    \li 0x03 = 64-bit extended address
   **/
  APS_AddrMode_t dstAddrMode;
  /** \ref Endian "[LE]" The 16-bit PAN identifier of the entity or entities
   * to which the ASDU is being  transferred or the broadcast PANId 0xffff. */
  PanId_t dstPANID;
  /** \ref Endian "[LE]" The address of the entity or entities to which
   *  the ASDU is being transferred. */
  MAC_Addr_t dstAddress;
  /** \ref Endian "[LE]" The identifier of the profile for which this frame is
   * intended. */
  ProfileId_t profileId;
  /** \ref Endian "[LE]" The identifier of the cluster, within the profile
   * specified by the ProfileId parameter, which defines the application
   * semantics of the ASDU. **/
  ClusterId_t clusterId;
  /** The number of octets in the ASDU to be transmitted. */
  IntrpLength_t asduLength;
  /** The set of octets comprising the ASDU to be transferred. */
  uint8_t *asdu;
  /** An integer handle associated with the ASDU to be transmitted. */
  IntrpAsduHandle_t asduHandle;
  /** Callback function pointer as a handler of corresponding
   *  confirm primitive.
   **/
  void (*INTRP_DataConf)(INTRP_DataConf_t *conf);
  /** Confirm primitive as a parameter of the callback function. */
  INTRP_DataConf_t confirm;
  /** Transmission options for this request **/
  INTRP_TxOptions_t txOptions;
} INTRP_DataReq_t;

/**//**
 * \struct INTRP_DataInd_t intrpData.h "aps.h"
 *
 * \brief INTRP-DATA indication primitive's parameters structure declaration.
 *
 *  ZigBee Smart Energy profile specification r15, B.3.3.
 * The INTRP-DATA.indication Primitive, page 86.
 **/
typedef struct
{
  /** \ref Endian "[LE]" The 16-bit PAN identifier of the entity from which
   *  the ASDU is being transferred. */
  PanId_t srcPANID;
  /** \ref Endian "[LE]" The device address of the entity from which
   *  the ASDU is being transferred. */
  MAC_Addr_t srcAddress;
  /** The addressing mode for the destination address used in this primitive.
   *  This parameter can take one of the values from the following list:
   *   \li 0x01 = 16-bit group address
   *   \li 0x02 = 16-bit NWK address, normally  the broadcast address 0xffff
   *   \li 0x03 = 64-bit extended address.
   **/
  APS_AddrMode_t dstAddrMode;
  /** \ref Endian "[LE]" The 16-bit PAN identifier of the entity or entities
   * to which the ASDU is being  transferred or the broadcast PANId 0xffff. */
  PanId_t dstPANID;
  /** \ref Endian "[LE]" The address of the entity or entities to which
   *  the ASDU is being transferred. */
  MAC_Addr_t dstAddress;
  /** \ref Endian "[LE]"
   * The identifier of the profile for which this frame is intended. */
  ProfileId_t profileId;
  /** \ref Endian "[LE]" The identifier of the cluster, within the profile
   * specified by the ProfileId parameter, which defines the application
   * semantics of the ASDU. */
  ClusterId_t clusterId;
  /** The number of octets in the ASDU to be transmitted. */
  IntrpLength_t asduLength;
  /** The set of octets comprising the ASDU to be transferred. */
  uint8_t *asdu;
  /** The link quality observed during the reception of the ASDU. */
  Lqi_t lqi;
  /** RSSI observed during the reception of the ASDU. */
  Lqi_t rssi;
} INTRP_DataInd_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief INTRP-DATA request primitive's prototype.

  \param[in] req - INTRP-DATA request parameters' structure pointer.

  \return None.
 ******************************************************************************/
void INTRP_DataReq(INTRP_DataReq_t *const req);

/**************************************************************************//**
  \brief INTRP-DATA indication primitive's prototype.

  \param[in] ind - INTRP-DATA indication parameters' structure pointer.

  \return None.
 ******************************************************************************/
void INTRP_DataInd(INTRP_DataInd_t *ind);

#endif /* _INTERPAN_ */
#endif /* _INTRP_DATA_H */
/** eof intrpData.h */

