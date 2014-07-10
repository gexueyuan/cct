/**************************************************************************//**
  \file nwkSecurity.h

  \brief Network security interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2009-04-13 Max Gekk - Created.
    2009-08-15 Max Gekk - Refactoring.
    2011-01-29 Max Gekk - Redesign.
   Last change:
    $Id: nwkSecurity.h 18245 2011-08-16 09:23:15Z arazinkov $
 ******************************************************************************/
/**//**
 *  The NWK layer is responsible for the processing steps needed to securely
 * transmit outgoing frames and securely receive incoming frames. Upper layers
 * control the security processing operations by setting up the appropriate keys
 * and frame counters and establishing which security level to use.
 * ZigBee spec r17, 4.3, page 426.
 **/
#if !defined _NWK_SECURITY_H && defined _SECURITY_
#define _NWK_SECURITY_H
/******************************************************************************
                                Includes section
 ******************************************************************************/
#include <sspCommon.h>
#include <macs.h>
#include <nlmeReset.h>

/******************************************************************************
                                Define(s) section
 ******************************************************************************/
#define NWK_PRECONFIGURED_NETWORK_KEY 0U
#define NWK_NOT_PRECONFIGURED_NETWORK_KEY 3U

/** Amount of incoming frame counters. */
#define NWK_IN_FRAMES_MAX 4U
/** Startup active network key sequence number */
#define NWK_STARTUP_ACTIVE_KEY_SEQUENCE_NUMBER 0U
/** Size of security counter in bytes */
#define NWK_SIZE_OF_SECURITY_COUNTER 4U

/******************************************************************************
                                 Types section
 ******************************************************************************/
/** Type of NWK Key Sequence Number. */
typedef uint8_t NWK_KeySeqNum_t;
/** Type of NWK Key Index and amount. */
typedef uint8_t NwkKeyAmount_t;
typedef NwkKeyAmount_t NwkKeyIndex_t;

/** Type of outgoing frame counter used for outgoing frames. */
typedef uint32_t NwkOutFrameCounter_t;
/** Type of incoming frame counter used for incoming frames. */
typedef uint32_t NwkInFrameCounter_t;
/** Type of outgoing frame counter higher bytes. */
typedef uint16_t NwkOutFrameCounterTop_t;
/** Type of outgoing frame counter lower bytes. */
typedef uint16_t NwkOutFrameCounterLow_t;

/** Outgoing security frame counter handle type. */
BEGIN_PACK
typedef union PACK
{
  NwkOutFrameCounter_t value;
  struct
  {
    LITTLE_ENDIAN_OCTET(2,
      (NwkOutFrameCounterLow_t low,
       NwkOutFrameCounterTop_t top
      ))
  } part;
} NwkOutFrameCounterHandle_t;
END_PACK

BEGIN_PACK
typedef union PACK _NwkUnalignedInFrameCounter_t
{
  NwkInFrameCounter_t value;
  uint8_t raw[sizeof(NwkInFrameCounter_t)];
} NwkUnalignedInFrameCounter_t;
END_PACK

/** The Incoming Frame Counter Descriptor. */
typedef struct _NWK_InFrameCounterSet_t
{
  bool isSet; /** Flag indicates busy status of entry. */
  /** Extended device address. */
  ExtAddr_t senderAddress;
  /** Incoming frame counter used for incoming frames. */
  NwkInFrameCounter_t value;
} NWK_InFrameCounterSet_t;

/** The Network Security Material Descriptor - key and key sequence number. */
typedef struct _NWK_SecurityKey_t
{
  bool isSet; /** Flag indicates busy status of entry. */

  /** A sequence number assigned to a network key by the Trust Center and used
   * to distinguish network keys for purposes of key updates, and incoming
   * frame security operations. */
  NWK_KeySeqNum_t seqNum;
  /** Outgoing frame counter higher bytes. */
  NwkOutFrameCounterTop_t outCounterTop;
  /** The actual value of the key. */
  uint8_t value[SECURITY_KEY_SIZE];
} NWK_SecurityKey_t;

/** The Network Mutable Security Material Descriptor - incoming and outgoing
 * frame counters. */
typedef struct _NWK_SecurityFrameCounters_t
{
  /** Outgoing frame counter lower bytes. */
  NwkOutFrameCounterLow_t outCounterLow;
  /** Set of incoming frame counter values and corresponding device addresses. */
  /* NWK_InFrameCounterSet_t inCounters[NWK_IN_FRAMES_MAX]; */
} NWK_SecurityFrameCounters_t;

/** Security-Related NIB Attributes. */
typedef struct _NWK_SecurityIB_t
{
  /** Private field for internal use - the index of active key. */
  NwkKeyIndex_t activeIndex;
  /** The security level for outgoing and incoming NWK frames. */
  uint8_t securityLevel;
  /** Indicates whether security shall be applied to incoming
   * and outgoing NWK data frames. */
  bool secureAllFrames;
  /** Total amount of NWK security keys. */
  NwkKeyAmount_t keyAmount;
  /** Set of network security material descriptors capable of maintaining
   * an active and alternate network key. */
  NWK_SecurityKey_t *key;
  /** Pointer to set of security incoming and outgoing key frame counters.*/
  NWK_SecurityFrameCounters_t *counter;
} NWK_SecurityIB_t;

/******************************************************************************
                               Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Get value of the nwkSecurityLevel attribute.

    ZigBee spec r17, page 344, Table 3.44.

  \return Current value of nwkSecurityLevel.
 ******************************************************************************/
uint8_t NWK_GetSecurityLevel(void);

/**************************************************************************//**
  \brief Getting value of nwkSecureAllFrames attribute.
    ZigBee spec r17, page 344, Table 3.44.

  \return Current value of nwkSecureAllFrames.
 ******************************************************************************/
bool NWK_GetSecureAllFrames(void);

/**************************************************************************//**
  \brief Set a new network key.

  \param[in] key - a new network security key.
  \param[in] keySeqNum - key sequence number.
  \return 'true' if operation was completed successfully otherwise 'false'.
 ******************************************************************************/
bool NWK_SetKey(const uint8_t *const key, const NWK_KeySeqNum_t keySeqNum);

/**************************************************************************//**
  \brief Set NWK key as active.

  \param[in] keySeqNum - key sequence number.
  \return 'true' if operation was completed successfully otherwise 'false'.
 ******************************************************************************/
bool NWK_ActivateKey(const NWK_KeySeqNum_t keySeqNum);

/**************************************************************************//**
  \brief Check active network key.

  \return 'true' if active NWK key is set otherwise 'false'.
 ******************************************************************************/
bool NWK_IsAnyKeyActivated(void);

/**************************************************************************//**
  \brief Get active network key.

  \return Pointer to value of active network key.
 ******************************************************************************/
const uint8_t* NWK_GetActiveKey(void);

/**************************************************************************//**
  \brief Getting value of nwkActiveKeySeqNumber attribute.
     ZigBee spec r17, page 344, Table 3.44.

  \return Current value of nwkActiveKeySeqNumber.
 ******************************************************************************/
NWK_KeySeqNum_t NWK_GetActiveKeySeqNumber(void);

/**************************************************************************//**
  \brief Update outgoing security frame counter of active NWK key and return
         it's value.

  \return Outgoing frame counter in little-endian format.
 ******************************************************************************/
NwkOutFrameCounter_t NWK_GetUpdatedSecOutCounter(void);

/**************************************************************************//**
  \brief Return outgoing security frame counter value of active NWK key

  \return Outgoing frame counter in little-endian format.
 ******************************************************************************/
NwkOutFrameCounter_t NWK_GetSecOutCounter(void);

/**************************************************************************//**
  \brief Update the incoming frame counter.

  \param[in] srcExtAddr - an extended source address.
  \param[in] newValue - a new value of the incoming frame counter.
  \return None.
 ******************************************************************************/
void NWK_SetInFrameCounter(const ExtAddr_t srcExtAddr,
  const NwkInFrameCounter_t counter);

/**************************************************************************//**
  \brief Deactivate current active NWK key.
 ******************************************************************************/
void NWK_DeactivateKey(void);

/**************************************************************************//**
  \brief Set all incoming and outgoing frame counters to zero.
 ******************************************************************************/
void NWK_ResetAllSecurityCounters(void);

/**************************************************************************//**
  \brief Reseting the network security subsystem.

  \param[in] powerFailureControl - stack restoring after power failure control bitfield;
                                  affects on initialization procedure.
  \return None.
 ******************************************************************************/
void NWK_ResetSecurityIB(const NWK_PowerFailureControl_t powerFailureControl);

/**************************************************************************//**
  \brief Checks, if security services are provided by NWK.

  \return True, if security services are provided by NWK, false - otherwise.
 ******************************************************************************/
bool NWK_IsSecurityProvided(void);

#if defined _CERTIFICATION_
/**************************************************************************//**
  \brief Changes the maximum number of stored NWK security keys.

  \param[in] keyAmount - new total amount of NWK keys.

  \return NWK_SUCCESS_STATUS if keyAmount is changed successfuly otherwise
          return NWK_INVALID_PARAMETERS_STATUS.
 ******************************************************************************/
NWK_Status_t NWK_SetKeyAmount(const uint8_t keyAmount);
#endif /* _CERTIFICATION_ */

#endif /* _NWK_SECURITY_H and _SECURITY_ */
/** eof nwkSecurity.h */


