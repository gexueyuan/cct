/**************************************************************************//**
  \file rfRandom.h
  
  \brief Prototype of random generation function.
  
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).
  
  \internal
    History:     
      08/04/08 A. Mandychev - Created.
******************************************************************************/

#ifndef _RFRANDOM_H
#define _RFRANDOM_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <types1.h>
#include <macCommon.h>

/******************************************************************************
                    Define(s) section
******************************************************************************/
 
/******************************************************************************
                    Types section
******************************************************************************/
//! Random confirm structure. 
typedef struct 
{ 
  uint8_t *buff;
  uint16_t value;
} RF_RandomConf_t;

//! Random request structure.
typedef struct
{
  //! Service field - for internal needs.
  MAC_Service_t  service;
  //! Number of bytes to be generated
  uint8_t numberOfBytes;
  //! Confirm structure on RF_RandomReq.
  RF_RandomConf_t confirm;
  //! Callback on RF_RandomReq.
  void (*RF_RandomConf)(RF_RandomConf_t *conf);
} RF_RandomReq_t;

/******************************************************************************
                    Prototypes section
******************************************************************************/
/**************************************************************************//**
  \brief Requests random value from RF chip. Valid for RF231 and RF212.
  \param reqParams - request parameters structure pointer. 
  \return none.
******************************************************************************/
void RF_RandomReq(RF_RandomReq_t *reqParams);

/**************************************************************************//**
  \brief Requests random value from RF chip. Valid for RF231 and RF212.
  \param reqParams - request parameters structure pointer. 
  \return none.
******************************************************************************/
void RF_RandomSeqReq(RF_RandomReq_t *reqParams);

#endif /*_RFRANDOM_H*/

// eof rfRandom.h
