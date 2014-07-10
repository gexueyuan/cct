/**************************************************************************//**
  \file apsAIB.h

  \brief Interface of APS information base.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-10-19 Max Gekk - Created.
   Last change:
    $Id: apsAIB.h 18613 2011-09-20 13:15:05Z mgekk $
 ******************************************************************************/
/**//**
 *
 *  The APS information base comprises the attributes required to manage the APS
 * layer of a device. The attributes of the AIB are listed in ZigBee spec r19,
 * Table 2.24, page 61. The securityrelated AIB attributes are described in
 * sub-clause 4.4.10, page 489.
 **/
#if !defined _APS_AIB_H
#define _APS_AIB_H

/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <nwk1.h>
#include <configServer.h>

/******************************************************************************
                             Definitions section
******************************************************************************/
#define APS_AIB_NONMEMBER_RADIUS_DEFAULT_VALUE 2

/******************************************************************************
                                Types section
 ******************************************************************************/
/**//**
 * \struct AIB_t apsAIB.h "aps.h"
 *
 * \brief Attributes in APS Information Base.
 **/
typedef struct
{
  /** The value to be used for the NonmemberRadius parameter when using
   * NWK layer multicast. Valid range: 0 - 7, default value - 2. */
  NwkRadius_t nonMemberRadius;
#if defined _SECURITY_
  /** \ref Endian "[LE]" Identifies the address of the device's Trust Center. */
  ExtAddr_t  trustCenterAddress;
#endif /* _SECURITY_ */
#ifdef _APS_FRAGMENTATION_
  uint8_t fragWindowSize;
#ifdef _CERTIFICATION_
  uint8_t fragDropMask;
#endif /* _CERTIFICATION_  */
#endif /* _APS_FRAGMENTATION_ */
} AIB_t;

/**//**
 * \struct APS_TcMode_t apsAIB.h "aps.h"
 *
 * \brief Mode indicate role of this device in the network as trust center.
 **/
typedef enum _APS_TcMode_t
{
  APS_NOT_TRUST_CENTER,
  APS_CENTRALIZED_TRUST_CENTER,
  /** Mode whereby routers in a ZigBee Pro network can each act like a trust 
   * center so that there is no need for a single centralized device. */
  APS_DISTRIBUTED_TRUST_CENTER
} APS_TcMode_t;

/******************************************************************************
                           Global variables section
 ******************************************************************************/
extern AIB_t csAIB; /*!< APS Information Base */

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
#if defined _SECURITY_
/**************************************************************************//**
  \brief Check whether supplied address is a Trust Center address

  \param[in] addr - pointer to required address

  \return true, if required address is TC address, false - otherwise
******************************************************************************/
INLINE bool APS_IsTcAddress(const ExtAddr_t *addr)
{
  return IS_EQ_EXT_ADDR(*addr, csAIB.trustCenterAddress);
}

/**************************************************************************//**
  \brief Get pointer to Trust Center address

  \return pointer to Trust Center extended address
******************************************************************************/
INLINE ExtAddr_t* APS_GetTrustCenterAddress(void)
{
  return &csAIB.trustCenterAddress;
}

/**************************************************************************//**
  \brief Set Trust Center address

  \param[in] addr - pointer to address to set

  \return none
******************************************************************************/
INLINE void APS_SetTrustCenterAddress(const ExtAddr_t *addr)
{
  memcpy(&csAIB.trustCenterAddress, addr, sizeof(ExtAddr_t));
}

/**************************************************************************//**
  \brief Gets the trust center mode of the current device.

  \return Status of the device as Trust Center:
    \li APS_NOT_TRUST_CENTER - this deive is not trust center,
    \li APS_CENTRALIZED_TRUST_CENTER - single trust center in the network,
    \li APS_DISTRIBUTED_TRUST_CENTER  - each routers can be trust center.
******************************************************************************/
INLINE APS_TcMode_t APS_GetOwnTcMode(void)
{
#if defined _DISTRIBUTED_TRUST_CENTER_
  if (APS_DISTRIBUTED_TC_ADDRESS == csAIB.trustCenterAddress)
    return APS_DISTRIBUTED_TRUST_CENTER;
#endif /* _DISTRIBUTED_TRUST_CENTER_ */

  if (IS_EQ_EXT_ADDR(*MAC_GetExtAddr(), csAIB.trustCenterAddress))
    return APS_CENTRALIZED_TRUST_CENTER;
  else
    return APS_NOT_TRUST_CENTER;
}

/**************************************************************************//**
  \brief Check whether this node is the Trust Center or not.

  \return true, if this node is the Distributed or Centralized Trust Center,
                otherwise return false.
******************************************************************************/
INLINE bool APS_IsTrustCenter(void)
{
  return APS_NOT_TRUST_CENTER != APS_GetOwnTcMode();
}

/**************************************************************************//**
  \brief Check whether given address is trusted address.

  \param[in] addr - pointer to required address

  \return true, if argument is the address of the centralized Trust Center 
                               or we are in distributed TC mode,
                otherwise return false.
******************************************************************************/
INLINE bool APS_IsTrustedAddress(const ExtAddr_t *addr)
{
#if defined _DISTRIBUTED_TRUST_CENTER_
  if (APS_DISTRIBUTED_TC_ADDRESS == csAIB.trustCenterAddress)
    return true;
#endif /* _DISTRIBUTED_TRUST_CENTER_ */

  return IS_EQ_EXT_ADDR(*addr, csAIB.trustCenterAddress);
}

#endif /* _SECURITY_ */

#endif /* _APS_AIB_H */
/** eof apsAIB.h */

