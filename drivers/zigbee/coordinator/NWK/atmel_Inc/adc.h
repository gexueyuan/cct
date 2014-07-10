/**************************************************************************//**
  \file adc.h

  \brief The header file describes the ADC interface

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    22.11.07 A. Khromykh - Created.
 ******************************************************************************/
/******************************************************************************
 *   WARNING: CHANGING THIS FILE MAY AFFECT CORE FUNCTIONALITY OF THE STACK.  *
 *   EXPERT USERS SHOULD PROCEED WITH CAUTION.                                *
 ******************************************************************************/

#ifndef _ADC_H
#define _ADC_H

// \cond
/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
// \endcond

/** \brief adc resolution */
#define RESOLUTION_8_BIT  0
#define RESOLUTION_10_BIT 1

/******************************************************************************
                   Types section
******************************************************************************/
/**************************************************************************//**
\brief adc sample rate
******************************************************************************/
#if defined(AT91SAM7X256)
  typedef enum
  {
    ADC_533KSPS,
    ADC_429KSPS,
    ADC_369KSPS,
    ADC_250KSPS,
    ADC_136KSPS,
    ADC_68KSPS,
    ADC_34KSPS
  } HAL_AdcSampleRate_t;
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  typedef enum
  {
    ADC_77KSPS,
    ADC_39KSPS,
    ADC_19200SPS,
    ADC_9600SPS,
    ADC_4800SPS
  } HAL_AdcSampleRate_t;
#endif

/**************************************************************************//**
\brief adc voltage reference
******************************************************************************/
typedef enum
{
  /** \brief AREF, Internal Vref turned off. \n
  If 10x or 200x gain is selected, only 2.56 V should be used as Internal Voltage Reference. \n
  For differential conversion, only 1.1V cannot be used as internal voltage reference. */
  AREF = (0 << 6),
  /** \brief AVCC with external capacitor at AREF pin */
  AVCC = (1 << 6),
  /** \brief Internal 1.1V Voltage Reference with external capacitor at AREF pin */
  INTERNAL_1d1V = (2 << 6),
  /** \brief Internal 2.56V Voltage Reference with external capacitor at AREF pin */
  INTERNAL_2d56V = (3 << 6)
} HAL_AdcVoltageReference_t;

/**************************************************************************//**
\brief adc structure of parameters
******************************************************************************/
typedef struct
{
  /** \brief conversion resolution */
  uint8_t resolution;
  /** \brief adc sample rate */
  HAL_AdcSampleRate_t sampleRate;
  /** \brief adc voltage reference selections (only for avr) */
  HAL_AdcVoltageReference_t voltageReference;
  /** \brief pointer to the application data */
  void *bufferPointer;
  /** \brief amount of samples (buffer cells) */
  uint16_t selectionsAmount;
  /** \brief pointer to callback method */
  void (*callback)();
} HAL_AdcParams_t;

/**************************************************************************//**
\brief channel number. \n
Note for avr:
If 10x gain is used, 8 bit resolution can be expected. \n
If 200x gain is used, 7 bit resolution can be expected. \n
If the user wants to perform a quick polarity check of the result, \n
it is sufficient to read the MSB of the result. If the bit is one, \n
the result is negative, and if this bit is zero, the result is positive. \n
To reach the given accuracy, 10x or 200x Gain should not be used \n
for operating voltage below 2.7V.
******************************************************************************/
#if defined(AT91SAM7X256)
  typedef enum
  {
    HAL_ADC_CHANNEL0 = (1 <<  0),
    HAL_ADC_CHANNEL1 = (1 <<  1),
    HAL_ADC_CHANNEL2 = (1 <<  2),
    HAL_ADC_CHANNEL3 = (1 <<  3),
    HAL_ADC_CHANNEL4 = (1 <<  4),
    HAL_ADC_CHANNEL5 = (1 <<  5),
    HAL_ADC_CHANNEL6 = (1 <<  6),
    HAL_ADC_CHANNEL7 = (1 <<  7)
  } HAL_AdcChannelNumber_t;
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || defined(ATMEGA128RFA1)
  typedef enum
  {
    /** \brief channel ADC0 */
    HAL_ADC_CHANNEL0 = 0,
    /** \brief channel ADC1 */
    HAL_ADC_CHANNEL1 = 1,
    /** \brief channel ADC2 */
    HAL_ADC_CHANNEL2 = 2,
    /** \brief channel ADC3 */
    HAL_ADC_CHANNEL3 = 3,
    /** \brief ADC0 - ADC0 with gain 10x */
    HAL_ADC_DIFF_CHANNEL0 = 8,
    /** \brief ADC1 - ADC0 with gain 10x */
    HAL_ADC_DIFF_CHANNEL1 = 9,
    /** \brief ADC0 - ADC0 with gain 200x */
    HAL_ADC_DIFF_CHANNEL2 = 10,
    /** \brief ADC1 - ADC0 with gain 200x */
    HAL_ADC_DIFF_CHANNEL3 = 11,
    /** \brief ADC2 - ADC2 with gain 10x */
    HAL_ADC_DIFF_CHANNEL4 = 12,
    /** \brief ADC3 - ADC2 with gain 10x */
    HAL_ADC_DIFF_CHANNEL5 = 13,
    /** \brief ADC2 - ADC2 with gain 200x */
    HAL_ADC_DIFF_CHANNEL6 = 14,
    /** \brief ADC3 - ADC2 with gain 200x */
    HAL_ADC_DIFF_CHANNEL7 = 15
  } HAL_AdcChannelNumber_t;
#endif

/******************************************************************************
                   Prototypes section
******************************************************************************/
/**************************************************************************//**
\brief Opens the ADC to make the measurement on the ADC channel.

  \param[in] param - address of HAL_AdcParams_t structure. \n
  fields of structure set by user: \n
  resolution - conversion resolution. Must be chosen from: \n
    RESOLUTION_8_BIT  \n
    RESOLUTION_10_BIT \n
  sampleRate - sample rate. Must be chosen from: \n
   for avr hardware platform \n
    ADC_77KSPS \n
    ADC_39KSPS \n
    ADC_19200SPS \n
    ADC_9600SPS \n
    ADC_4800SPS \n
   for arm hardware platform \n
    ADC_533KSPS \n
    ADC_429KSPS \n
    ADC_369KSPS \n
    ADC_250KSPS \n
    ADC_136KSPS \n
    ADC_68KSPS  \n
    ADC_34KSPS  \n
  bufferPointer - pointer to the application data buffer. \n
  Buffer cell is one byte for 8-bit resolution. \n
  Buffer cell is two bytes for 10-bit resolution. \n
  selectionsAmount - amount of samples (buffer cells).  \n
  callback - pointer to the function that will notify about measurement completion.
\return
  -1 - unsupported parameter or ADC is busy. \n
   0 - success.
******************************************************************************/
int HAL_OpenAdc(HAL_AdcParams_t *param);

/**************************************************************************//**
\brief Starts ADC with the parameters defined in HAL_OpenAdc.

  \param[in] channel - number of ADC channel. Must be chosen from:\n
             HAL_ADC_CHANNEL0 \n
             HAL_ADC_CHANNEL1 \n
             HAL_ADC_CHANNEL2 \n
             HAL_ADC_CHANNEL3 \n
             next four channels only for arm hardware platform \n
             HAL_ADC_CHANNEL4 \n
             HAL_ADC_CHANNEL5 \n
             HAL_ADC_CHANNEL6 \n
             HAL_ADC_CHANNEL7 \n
             next eight channels only for avr hardware platform \n
             HAL_ADC_DIFF_CHANNEL0 \n
             HAL_ADC_DIFF_CHANNEL1 \n
             HAL_ADC_DIFF_CHANNEL2 \n
             HAL_ADC_DIFF_CHANNEL3 \n
             HAL_ADC_DIFF_CHANNEL4 \n
             HAL_ADC_DIFF_CHANNEL5 \n
             HAL_ADC_DIFF_CHANNEL6 \n
             HAL_ADC_DIFF_CHANNEL7 \n

\return
  -1 - unable to open ADC (unsupported channel number). \n
   0 - success.
******************************************************************************/
int HAL_ReadAdc(HAL_AdcChannelNumber_t channel);

/**************************************************************************//**
\brief Closes the ADC.

\return
  -1  - the module was not open. \n
   0  - success.
******************************************************************************/
int HAL_CloseAdc(void);

#endif /* _ADC_H */
// eof adc.h
