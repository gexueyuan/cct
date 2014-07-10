/**************************************************************************//**
  \file  apsKeyPairSet.h

  \brief Private interface of APS Key-Pair Set.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
   History:
    2010-11-01 Max Gekk - Created.
   Last change:
    $Id: apsKeyPairSet.h 17813 2011-07-04 14:11:12Z dloskutnikov $
 ******************************************************************************/
#if !defined _APS_KEY_PAIR_SET_H
#define _APS_KEY_PAIR_SET_H

#if defined _LINK_SECURITY_ 
/******************************************************************************
                               Includes section
 ******************************************************************************/
#include <apsCryptoKeys.h>
#include <sspCommon.h>

/******************************************************************************
                               Definitions section
 ******************************************************************************/
/* Size of APS security counter in bytes */
#define APS_SECURITY_COUNTER_SIZE 4U

/******************************************************************************
                                Types section
 ******************************************************************************/
/* Type of outgoing frame counter higher bytes. */
typedef uint16_t ApsOutFrameCounterTop_t;
/* Type of outgoing frame counter lower bytes. */
typedef uint16_t ApsOutFrameCounterLow_t;

/** Outgoing security frame counter handle type. */
BEGIN_PACK
typedef union PACK
{
  ApsOutFrameCounter_t value;
  struct
  {
    LITTLE_ENDIAN_OCTET(2,
      (ApsOutFrameCounterLow_t low,
       ApsOutFrameCounterTop_t top))
  } part;
} ApsOutFrameCounterHandle_t;
END_PACK

/* Type of key-pair descriptor. */
typedef struct _ApsKeyPairDescriptor_t
{
  /** A flag indicating that key-pair is used. */
  bool busy;
  /** A flag indicating that key-pair is authorized. */
  bool auth;
  /* Outgoing frame counter higher bytes. */
  ApsOutFrameCounterTop_t outCounterTop;
  /* Identifies the address of the entity with which this key-pair is shared. */
  ExtAddr_t deviceAddress;
  /* The actual value of the initial key:
     master key for High Security and preinstalled link key for SE Security. */
  uint8_t initialKey[SECURITY_KEY_SIZE];
  /* The actual value of the link key. */
  uint8_t linkKey[SECURITY_KEY_SIZE];
} ApsKeyPairDescriptor_t;

/* Type of incoming and outgoing counters. */
typedef struct _ApsKeyCounters_t
{
  /* Outgoing frame counter lower bytes. */
  ApsOutFrameCounterLow_t outCounterLow;
  /* Incoming frame counter value corresponding to DeviceAddress. */
  ApsInFrameCounter_t in;
} ApsKeyCounters_t;

/* Type of size of APS Key-Pair Set. */
typedef uint8_t ApsKeyPairAmount_t;
/* Type of Key-Pair Index. */
typedef ApsKeyPairAmount_t ApsKeyPairIndex_t;

/******************************************************************************
                              Prototypes section
 ******************************************************************************/
/**************************************************************************//**
  \brief Get current value of outgoing frame counter and increment it.

  \param[in] keyHandle - valid key handle.

  \return Pointer to current value of outgoing frame counter.
 ******************************************************************************/
APS_PRIVATE
ApsOutFrameCounter_t apsGetUpdatedOutFrameCounter(const APS_KeyHandle_t keyHandle);

/**************************************************************************//**
  \brief Get current value of incoming frame counter.

  \param[in] handle - valid key handle.

  \return Pointer to current value of incoming frame counter.
 ******************************************************************************/
APS_PRIVATE
ApsInFrameCounter_t apsGetInFrameCounter(const APS_KeyHandle_t handle);

#endif /* _LINK_SECURITY_ */

#endif /* _APS_KEY_PAIR_SET_H */
/** eof apsKeyPairSet.h */

