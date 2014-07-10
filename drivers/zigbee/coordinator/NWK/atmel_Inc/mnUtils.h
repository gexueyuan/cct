/**************************************************************************//**
  \file  mnUtils.h
  \brief Header file describes stack utilities functions.

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
     History:
      07/10/08 - Created.
******************************************************************************/
#ifndef _MNUTILS_H
#define _MNUTILS_H

/******************************************************************************
                        Includes section.
******************************************************************************/
#include <types1.h>
#include <string.h>
#include <stdlib.h>

/******************************************************************************
                        Definitions section.
******************************************************************************/
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C)
  #define SYS_BYTE_MEMCPY SYS_ByteMemcpy
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) \
   || defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3) || defined(ATMEGA128RFA1)
  #define SYS_BYTE_MEMCPY memcpy
#else
  #define SYS_BYTE_MEMCPY memcpy
#endif

#if defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) || \
   defined(ATMEGA128RFA1)
/* Macroses to accept memory I/O registers for AVR Mega family */
#define MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define MMIO_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
#endif /* AVR Mega family */

#define  GET_FIELD_PTR(structPtr, typeName, fieldName) \
  ((uint8_t *)(structPtr) + offsetof(typeName, fieldName))

#define GET_PARENT_BY_FIELD(TYPE, FIELD, FIELD_POINTER)  \
  ((TYPE *)(((uint8_t *)FIELD_POINTER) - offsetof(TYPE, FIELD)))
#define GET_CONST_PARENT_BY_FIELD(TYPE, FIELD, FIELD_POINTER)  \
  ((const TYPE *)(((const uint8_t *)FIELD_POINTER) - offsetof(TYPE, FIELD)))

#define GET_STRUCT_BY_FIELD_POINTER(struct_type, field_name, field_pointer)\
  ((struct_type *) (((uint8_t *) field_pointer) - FIELD_OFFSET(struct_type, field_name)))
#define GET_INDEX_FROM_OFFSET(PTR1, PTR2)  (PTR1 - PTR2)

// Size of slice between firstField end lastField of struct (including lastField)
#define SLICE_SIZE(type, firstField, lastField)\
  (offsetof(type, lastField) - offsetof(type, firstField) + sizeof(((type *)0)->lastField))

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#undef FIELD_OFFSET
#define FIELD_OFFSET(struct_type, field_name)\
  (((uint8_t*) &((struct_type *)(NULL))->field_name) - (uint8_t*)NULL)

#define MIN(a, b)                       ( ((a) < (b)) ? (a) : (b) )
#define MAX(x,y) ((x)<(y)?(y):(x))

/******************************************************************************
                        Inline functions' section.
******************************************************************************/
/**************************************************************************//**
  \brief Performs bytes memory copying operation.

  \param dst  - points destination memory start address.
         src  - points source memory start address.
         size - number of bytes to copy.
  \return pointer to dst.
******************************************************************************/
INLINE void *SYS_ByteMemcpy(void *dst, const void *src, uint16_t size)
{
  uint8_t *dst_ = (uint8_t *) dst;
  const uint8_t *src_ = (const uint8_t *) src;

  while(size--)
    *(dst_++) = *(src_++);
  return dst;
}

/**************************************************************************//**
  \brief Performs swap bytes in array of length

  \param array  - pointer to array.
         length  - array length
  \return no return.
******************************************************************************/
void SYS_Swap(uint8_t *array, uint8_t length);

/**************************************************************************//**
  \brief Random number generator. The range of the generator is 0-0xFFFF.The SID
   is generated in ZDO from UID by srand()

  \param: none.
  \returns: a random number.
******************************************************************************/
static inline uint16_t SYS_GetRandomNumber(void)
{
  uint16_t result = (rand() << 4) & 0xFF00;

  result |= rand() & 0x00FF;
  return result;
}

/**************************************************************************//**
  \brief This function copies size bytes of random data into buffer.

  \param: buffer - This is an unsigned char array of size at least sz to hold
                   the random data.
          size   - The number of bytes of random data to compute and store.

  \return:  0 Indicates successful completion.
******************************************************************************/
int SYS_GetRandomSequence(uint8_t *buffer, unsigned long size);

#endif // _MNUTILS_H
// eof mnUtils.h
