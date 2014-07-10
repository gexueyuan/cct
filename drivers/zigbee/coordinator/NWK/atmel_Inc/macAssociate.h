/**************************************************************************//**
  \file macAssociate.h
  
  \brief Types', constants' and functions' declarations for IEEE 802.15.4-2006 
    association primitives.
    
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).
  
  \internal
    History:     
      18/06/07 ALuzhetsky - Created.
******************************************************************************/

#ifndef _MACASSOCIATE_H
#define _MACASSOCIATE_H

/******************************************************************************
                        Includes section.
******************************************************************************/
#include <bcEndian.h>
#include <macAddr.h>
#include <macCommon.h>
#include <macCommStatus.h>

/******************************************************************************
                        Definitions section.
******************************************************************************/

/******************************************************************************
                        Types section.
******************************************************************************/
/**
  \brief MAC capability information field.
  IEEE 802.15.4-2006 7.3.1.2 Capability Information field.
*/
BEGIN_PACK
typedef struct PACK
{
  LITTLE_ENDIAN_OCTET(7, (
    uint8_t alternatePANCoordinator : 1,
    uint8_t deviceType              : 1,
    uint8_t powerSource             : 1,
    uint8_t rxOnWhenIdle            : 1,
    uint8_t reserved                : 2,
    uint8_t securityCapability      : 1,
    uint8_t allocateAddress         : 1
  ))
}  MAC_CapabilityInf_t;
END_PACK

/**
  \brief MLME-ASSOCIATE confirm primitive's parameters structure declaration.
  IEEE 802.15.4-2006 7.1.3.4 MLME-ASSOCIATE.confirm.
*/
typedef struct
{
  //! The short device address allocated by the coordinator on successful association.
  ShortAddr_t  shortAddr;
  //! The extended device address of coordinator wich allocated short address.
  //! It's additional parameter, not included in IEEE 802.15.4.
  ExtAddr_t extAddr;
  //! The status of the association attempt.
  MAC_Status_t status;
}  MAC_AssociateConf_t;

/**
  \brief MLME-ASSOCIATE request primitive's parameters structure declaration.
  IEEE 802.15.4-2006 7.1.3.1 MLME-ASSOCIATE.request.
*/
typedef struct
{
  //! Service field - for internal needs.
  MAC_Service_t       service;
  //! The logical channel on which to attempt association.
  uint8_t             channel;
  //! The channel page on which to attempt association.
  uint8_t             page;
  //! The coordinator addressing mode for this primitive.
  MAC_AddrMode_t      coordAddrMode;
  //! The identifier of the PAN with which to associate.
  PanId_t             coordPANId;
  //! The address of the coordinator with which to associate.
  MAC_Addr_t          coordAddr;
  //! Specifies the operational capabilities of the associating device.
  MAC_CapabilityInf_t capability;
  //! MLME-ASSOCIATE confirm callback fubction's pointer.
  void (*MAC_AssociateConf)(MAC_AssociateConf_t *conf);
  //! MLME-ASSOCIATE confirm parameters' structure.
  MAC_AssociateConf_t confirm;
}  MAC_AssociateReq_t;

/**
  \brief MLME-ASSOCIATE indication primitive's parameters.
  IEEE 802.15.4-2006 7.1.3.2 MLME-ASSOCIATE.indication.
*/
typedef struct
{
  //! The address of the device requesting association.
  ExtAddr_t           extAddr;
  //! The operational capabilities requesting association.
  MAC_CapabilityInf_t capability;
}  MAC_AssociateInd_t;

/**
  \brief MLME-ASSOCIATE response pritive's parameters.
  IEEE 802.15.4-2006 7.1.3.3 MLME-ASSOCIATE.response.
*/
typedef struct
{
  //! Service field - for internal needs.
  MAC_ServiceTransaction_t service;
  //! The address of the device requesting association.
  ExtAddr_t     extAddr;
  //! The 16-bit short device address allocated by the coordinator on successful association.
  ShortAddr_t   shortAddr;
  //! The status of the association attempt.
  MAC_Status_t  status;
  //! MLME-COMM STATUS callback function's pointer.
  void (*MAC_CommStatusInd)(MAC_CommStatusInd_t *commStat);
  //! Comm staus parameters' structure.
  MAC_CommStatusInd_t commStatus;
}  MAC_AssociateResp_t;

/******************************************************************************
                        Prototypes section
******************************************************************************/
/**************************************************************************//**
  \brief MLME-ASSOCIATE request primitive's prototype.
  \param reqParams - MLME-ASSOCIATE request parameters' structure pointer.
  \return none.
******************************************************************************/
void MAC_AssociateReq(MAC_AssociateReq_t *reqParams);

/**************************************************************************//**
  \brief MLME-ASSOCIATE indication primitive's prototype.
  \param indParams - MLME-ASSOCIATE indication parameters' structure pointer.
  \return none.
******************************************************************************/
extern void MAC_AssociateInd(MAC_AssociateInd_t *indParams);

/**************************************************************************//**
  \brief MLME-ASSOCIATE response primitive's prototype.
  \param respParams - MLME-ASSOCIATE response parameters' structure pointer.
  \return none.
******************************************************************************/
void MAC_AssociateResp(MAC_AssociateResp_t *respParams);

#endif //_MACASSOCIATE_H
// eof macAssociate.h
