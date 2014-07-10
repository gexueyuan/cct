/**************************************************************************//**
  \file pwm.h

  \brief Description of the PWM interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
   10/11/08 A. Taradov - Created
   5/04/11 A.Razinkov - Refactored
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _PWM_H
#define _PWM_H

/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <atomic.h>
#include <mnUtils.h>

/******************************************************************************
                   Defines section
******************************************************************************/
#define TCCRnA(pwmUnit) MMIO_BYTE(pwmUnit)
#define TCCRnB(pwmUnit) MMIO_BYTE(pwmUnit + 1U)
#define TCCRnC(pwmUnit) MMIO_BYTE(pwmUnit + 2U)
#define TCNTn(pwmUnit)  MMIO_WORD(pwmUnit + 4U)
#define ICRn(pwmUnit)   MMIO_WORD(pwmUnit + 6U)
#define OCRnA(pwmUnit)  MMIO_WORD(pwmUnit + 8U)
#define OCRnB(pwmUnit)  MMIO_WORD(pwmUnit + 10U)
#define OCRnC(pwmUnit)  MMIO_WORD(pwmUnit + 12U)

#define PWM_SUCCESS_STATUS            0
#define PWM_INVALID_UNIT_STATUS      -1
#define PWM_INVALID_CHANNEL_STATUS   -2
#define PWM_INVALID_PRESCALER_STATUS -3

/******************************************************************************
                   Types section
******************************************************************************/
/**************************************************************************//**
\brief PWM prescaler
******************************************************************************/
typedef enum
{
  PWM_STOPPED,
  PWM_PRESCALER_1,
  PWM_PRESCALER_8,
  PWM_PRESCALER_64,
  PWM_PRESCALER_256,
  PWM_PRESCALER_1024,
  PWM_PRESCALER_EXT_CLOCK_ON_FALLING_EDGE,
  PWM_PRESCALER_EXT_CLOCK_ON_RISING_EDGE,
  PWM_PRESCALER_INVALID
} HAL_PwmPrescaler_t;

/**************************************************************************//**
\brief PWM channel
******************************************************************************/
typedef enum
{
  PWM_CHANNEL_0,
  PWM_CHANNEL_1,
#ifndef ATMEGA1284
  PWM_CHANNEL_2,
#endif /* ATMEGA1284 */
  PWM_INVALID_CHANNEL
} HAL_PwmChannel_t;

/**************************************************************************//**
\brief PWM impulse polarity
******************************************************************************/
typedef enum
{
  /** \brief PWM output is low when duty cycle = 0% */
  PWM_POLARITY_NON_INVERTED,
  /** \brief PWM output is high when duty cycle = 0% */
  PWM_POLARITY_INVERTED
} HAL_PwmPolarity_t;

/**************************************************************************//**
\brief PWM unit number. Relative to corresponding Timer/Counter.
******************************************************************************/
typedef enum
{
  /* Base address of T/C1 PWM related registers */
  PWM_UNIT_1 = 0x80,
  /* Base address of T/C3 PWM related registers */
  PWM_UNIT_3 = 0x90
} HAL_PwmUnit_t;

/**************************************************************************//**
\brief PWM module descriptor
******************************************************************************/
typedef struct _HAL_PwmDescriptor_t
{
  /* Service fields for internal use. */
  struct
  {
    /* Pointer to Data Direction Register. PWM port dependent. */
    volatile uint8_t *DDRn;
    /* PWN channel pin number. PWM port dependent. */
    uint8_t pwmBaseChannelPin;
    /* Compare Output Mode low bit position. PWM channel dependent. */
    uint8_t COMnx0;
    /* Output Compare Register. PWM channel dependent. */
    volatile uint16_t *OCRnx;
  } service;
  /* PWM unit number. Equal to ID of Timer/Counter witch serves PWM module. */
  HAL_PwmUnit_t unit;
  /* PWM channel */
  HAL_PwmChannel_t channel;
  /* PWM polarity */
  HAL_PwmPolarity_t polarity;
} HAL_PwmDescriptor_t;

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Initializes the PWM.

\param [in] pwmUnit - PWM unit number.
                      Equal to ID of Timer/Counter witch serves PWM module.

\return operation status
******************************************************************************/
int HAL_OpenPwm(HAL_PwmUnit_t pwmUnit);

/**************************************************************************//**
\brief Starts PWM on specified channel.

\param [in] descriptor - PWM channel descriptor.

\return operation status
******************************************************************************/
int HAL_StartPwm(HAL_PwmDescriptor_t *descriptor);

/**************************************************************************//**
\brief Stops PWM on specified channel.

\param [in] descriptor - PWM channel descriptor.

\return operation status
******************************************************************************/
int HAL_StopPwm(HAL_PwmDescriptor_t *descriptor);

/**************************************************************************//**
\brief Sets base frequency of module. Common for all module channels.

\param [in] pwmUnit - PWM unit number. Equal to corresponding Timer/Counter ID.
\param [in] top - value for the TOP register.
\param [in] prescaler - clock prescaler.

\return operation status
******************************************************************************/
int HAL_SetPwmFrequency(HAL_PwmUnit_t pwmUnit, uint16_t top, HAL_PwmPrescaler_t prescaler);


/**************************************************************************//**
\brief Sets compare value for the PWM channel.

\param [in] descriptor - PWM channel descriptor.

\return operation status
******************************************************************************/
int HAL_SetPwmCompareValue(HAL_PwmDescriptor_t *descriptor, uint16_t cmpValue);

/**************************************************************************//**
\brief Closes the PWM.

\param [in] pwmUnit - PWM unit number.
                      Equal to ID of Timer/Counter witch serves PWM module.

\return operation status
******************************************************************************/
int HAL_ClosePwm(HAL_PwmUnit_t pwmUnit);

#endif /* _PWM_H */

// eof pwm.h
