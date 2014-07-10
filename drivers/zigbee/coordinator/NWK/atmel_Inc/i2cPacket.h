/**************************************************************************//**
  \file  i2cPacket.h

  \brief The header file describes the i2cPacket interface.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    5/12/07 A. Khromykh - Created
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _I2CPACKET_H
#define _I2CPACKET_H

/******************************************************************************
                   Includes section
******************************************************************************/
// \cond
#include <halClkCtrl.h>
#include <types.h>
// \endcond

/******************************************************************************
                   Define(s) section
******************************************************************************/
/** \brief i2c bus prescaler */
#define HAL_I2C_PRESCALER 0ul
// \cond
/** \brief internal address size */
#if defined(AT91SAM7X256)
  /* AT91C_TWI_IADRSZ_NO */
  #define  HAL_NO_INTERNAL_ADDRESS   (0x0 << 8)
  /* AT91C_TWI_IADRSZ_1_BYTE */
  #define  HAL_ONE_BYTE_SIZE         (0x1 << 8)
  /* AT91C_TWI_IADRSZ_2_BYTE */
  #define  HAL_TWO_BYTE_SIZE         (0x2 << 8)
  /* AT91C_TWI_IADRSZ_3_BYTE */
  #define  HAL_THREE_BYTE_SIZE       (0x3 << 8)
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  #define  HAL_NO_INTERNAL_ADDRESS   0x0
  #define  HAL_ONE_BYTE_SIZE         0x1
  #define  HAL_TWO_BYTE_SIZE         0x2
  #define  HAL_THREE_BYTE_SIZE       0x3
#endif
// \endcond

/******************************************************************************
                   Types section
******************************************************************************/
/** \brief i2c baud rate */
#if defined(AT91SAM7X256)
  typedef enum
  {
    I2C_CLOCK_RATE_250 = F_CPU/(2*250000) - 3, // 200 Kb/s clock rate
    I2C_CLOCK_RATE_125 = F_CPU/(2*125000) - 3, // 125 Kb/s clock rate
    I2C_CLOCK_RATE_100 = F_CPU/(2*100000) - 3, // 100 Kb/s clock rate
    I2C_CLOCK_RATE_62  = F_CPU/(2*62500) - 3   // 62.5 Kb/s clock rate
  } I2cClockRate_t;
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  typedef enum
  {
    I2C_CLOCK_RATE_250 = ((F_CPU/250000ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER)), // 250 Kb/s clock rate
    I2C_CLOCK_RATE_125 = ((F_CPU/125000ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER)), // 125 Kb/s clock rate
    I2C_CLOCK_RATE_62  = ((F_CPU/62500ul) - 16ul)/(2ul * (1ul << HAL_I2C_PRESCALER) * (1ul << HAL_I2C_PRESCALER))   // 62.5 Kb/s clock rate
  } I2cClockRate_t;
#endif

/** \brief TWI clock rate */
typedef struct
{
  I2cClockRate_t clockrate; // clock rate
} HAL_i2cMode_t;

/**************************************************************************//**
\brief i2c access control structure
******************************************************************************/
typedef struct
{
  /** \brief slave address */
  uint8_t id;
  /** \brief number of bytes to be written to the bus */
  uint16_t length;
  /** \brief pointer to the data */
  uint8_t *data;
  /** \brief internal device address size conditions:
                  HAL_NO_INTERNAL_ADDRESS - byte command condition  \n
                  HAL_ONE_BYTE_SIZE       \n
                  HAL_TWO_BYTE_SIZE       \n
                  HAL_THREE_BYTE_SIZE */
#if defined(AT91SAM7X256)
  uint32_t lengthAddr;
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  uint8_t lengthAddr;
#endif
  /** \brief internal device address */
  uint32_t internalAddr;
  /** \brief callback function */
  void (*f)(bool result);
} HAL_I2cParams_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Opens twi resource.\n
field of i2cMode structure \n
i2cClockRate_t clockrate (set by user). Must be chosen from:\n
  I2C_CLOCK_RATE_250 - 250 Kb/s clock rate \n
  I2C_CLOCK_RATE_125 - 125 Kb/s clock rate \n
  I2C_CLOCK_RATE_62  - 62.5 Kb/s clock rate \n

\param[in]
  i2cMode - pointer to the mode structure.

\return
  -1 - resource was already open or pointer is NULL. \n
   0 - success.
******************************************************************************/
int HAL_OpenI2cPacket(HAL_i2cMode_t *i2cMode);

/**************************************************************************//**
\brief Closes resource.

\return
  -1 - resource was not open. \n
   0 - success.
******************************************************************************/
int HAL_CloseI2cPacket(void);

/**************************************************************************//**
\brief Writes a series of bytes out to the TWI bus. Operation result will be
sent to the callback function of the HAL_I2cParams_t structure.

\param[in]
  param - pointer to HAL_I2cParams_t structure
\return
   0 - the bus is free and the request is accepted. \n
  -1 - otherwise.
******************************************************************************/
int HAL_WriteI2cPacket(HAL_I2cParams_t *param);

/**************************************************************************//**
\brief Reads the series of bytes from the TWI bus. Operation result will be
sent to the callback function of the HAL_I2cParams_t structure.

\param[in]
  param - pointer to HAL_I2cParams_t structure
\return
  0 - the bus is free and the request is accepted. \n
 -1 - otherwise.
******************************************************************************/
int HAL_ReadI2cPacket(HAL_I2cParams_t *param);

#endif /* _I2CPACKET_H */
// eof i2cPacket.h
