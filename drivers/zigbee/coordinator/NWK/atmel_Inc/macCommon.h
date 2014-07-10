/**************************************************************************//**
  \file macCommon.h
  
  \brief Declarations of common MAC layer fields and types.
  
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).
  
  \internal
    History:     
      18/06/07 ALuzhetsky - Created.
******************************************************************************/

#ifndef _MACCOMMON_H
#define _MACCOMMON_H

/******************************************************************************
                         Includes section.
******************************************************************************/
#include <types1.h>

/******************************************************************************
                        Definitions section.
******************************************************************************/
//! MAC primitives' return codes.
typedef enum
{
  // IEEE802.15.4 table - 83.
  MAC_PAN_AT_CAPACITY_STATUS        = 0x01, 
  MAC_PAN_ACCESS_DENIED_STATUS      = 0x02,
  // IEEE802.15.4 table - 78.
  MAC_SUCCESS_STATUS                = 0x00,
  MAC_BEACON_LOSS_STATUS            = 0xE0,
  MAC_CHANNEL_ACCESS_FAILURE_STATUS = 0xE1,
  MAC_DENIED_STATUS                 = 0xE2,
  MAC_DISABLE_TRX_FAILURE_STATUS    = 0xE3,
  MAC_FRAME_TOO_LONG_STATUS         = 0xE5,
  MAC_INVALID_GTS_STATUS            = 0xE6,
  MAC_INVALID_HANDLE_STATUS         = 0xE7,
  MAC_INVALID_PARAMETER_STATUS      = 0xE8,
  MAC_NO_ACK_STATUS                 = 0xE9,
  MAC_NO_BEACON_STATUS              = 0xEA,
  MAC_NO_DATA_STATUS                = 0xEB,
  MAC_NO_SHORT_ADDRESS_STATUS       = 0xEC,
  MAC_PAN_ID_CONFLICT_STATUS        = 0xEE,
  MAC_REALIGNMENT_STATUS            = 0xEF,
  MAC_TRANSACTION_EXPIRED_STATUS    = 0xF0,
  MAC_TRANSACTION_OVERFLOW_STATUS   = 0xF1,
  MAC_TX_ACTIVE_STATUS              = 0xF2,
  MAC_UNSUPPORTED_ATTRIBUTE_STATUS  = 0xF4,
  MAC_INVALID_ADDRESS_STATUS        = 0xF5,
  MAC_INVALID_INDEX_STATUS          = 0xF9,
  MAC_LIMIT_REACHED_STATUS          = 0xFA, ///< There are some unscanned channels yet, but there is no memory.
  MAC_ON_TIME_TOO_LONG_STATUS       = 0xF6,
  MAC_PAST_TIME_STATUS              = 0xF7,
  MAC_READ_ONLY_STATUS              = 0xFB,
  MAC_SCAN_IN_PROGRESS_STATUS       = 0xFC,
  MAC_SUPERFRAME_OVERLAP_STATUS     = 0xFD,
  MAC_TRACKING_OFF_STATUS           = 0xF8
} MAC_Status_t;

/******************************************************************************
                        Types section.
******************************************************************************/
//! Internal service fields which give us opportunity to compose requests' queue.
typedef struct                         
{
  void    *next;
  uint8_t requestId;
}  MAC_Service_t;   

/** 
 * \brief Internal service fields which give us opportunity to compose and to serve
 * transactions frame queue.
*/
typedef struct                         
{
  void     *next;
  uint8_t  requestId;
  uint32_t ttl;
  bool     activated;
}  MAC_ServiceTransaction_t;   

#endif //_MACCOMMON_H

// eof macCommon.h
