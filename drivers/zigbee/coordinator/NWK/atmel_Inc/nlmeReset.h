/**************************************************************************//**
  \file nlmeReset.h

  \brief NLME-RESET interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-13 Max Gekk - Created.
   Last change:
    $Id: nlmeReset.h 17448 2011-06-09 13:53:59Z ataradov $
 ******************************************************************************/
#if !defined _NLME_RESET_H
#define _NLME_RESET_H

/******************************************************************************
                                Includes section
 ******************************************************************************/
#include <nwkCommon.h>

/******************************************************************************
                                 Types section
 ******************************************************************************/
/**//**
 * \brief NLME-RESET confirm primitive's parameters structure declaration.
 * ZigBee Specification r17, 3.2.2.20 NLME-RESET.confirm, page 293.
 */
typedef struct _NWK_ResetConf_t
{
  /** The result of the reset operation. */
  NWK_Status_t status;
} NWK_ResetConf_t;

/**//**
 * \brief Power failure status bitfield declaration.
 */
typedef struct _NWK_PowerFailureControl_t
{
  NwkBitField_t active                    : 1;
  NwkBitField_t networkOperationsAccepted : 1;
  NwkBitField_t reserved                  : 6;
} NWK_PowerFailureControl_t;

/**//**
 * \brief NLME-RESET request primitive's parameters structure declaration.
 * Zigbee Specification r17, 3.2.2.19 NLME-RESET.request, page 292.
 */
typedef struct _NWK_ResetReq_t
{
  /** Service fields - for internal needs. */
  struct
  {
    QueueElement_t qelem; /**< link used for queuing */
  } service;
  /** To restore or not the saved parameters? */
  NWK_PowerFailureControl_t  powerFailureControl;
  /** This field shall contain the device capability information.
   * See ZigBee spec r18, Table 3.44, page 345. */
  MAC_CapabilityInf_t capabilityInformation;
  /** NLME-RESET confirm callback function's pointer. */
  void (*NWK_ResetConf)(NWK_ResetConf_t *conf);
  NWK_ResetConf_t confirm;
} NWK_ResetReq_t;

/******************************************************************************
                           Definitions section
 ******************************************************************************/

/******************************************************************************
                               Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief NLME-RESET request primitive's prototype.

  \param[in] req - NLME-RESET request parameters' structure pointer.
  \return None.
******************************************************************************/
void NWK_ResetReq(NWK_ResetReq_t *const req);

/**************************************************************************//**
  \brief  Check, if BitCloud was restored after power failure

  \param[in] powerFailureControl - stack restoring after power failure control bitfield;
                                  affects on initialization procedure.
  \return True, if BitCloud was restored after power failure; false - otherwise
******************************************************************************/
INLINE bool NWK_StackRestoredAfterPowerFailure(NWK_PowerFailureControl_t powerFailureControl)
{
  return powerFailureControl.active && powerFailureControl.networkOperationsAccepted;
}

/**************************************************************************//**
  \brief Retruns Power Failure control information default value.

  \return Power Failure control default value.
 ******************************************************************************/
INLINE NWK_PowerFailureControl_t NWK_GetDefaultPowerFailureControl(void)
{
  NWK_PowerFailureControl_t powerFailureControl =
  {
    .active = 0U,
    .networkOperationsAccepted = 0U,
    .reserved = 0U
  };

  return powerFailureControl;
}

/**************************************************************************//**
  \brief Gather Power Failure control information.

  \return Power Failure control information.
 ******************************************************************************/
NWK_PowerFailureControl_t NWK_GatherPowerFailureInfo(void);

#endif  /* _NLME_RESET_H */
/** eof nlmeReset.h */

