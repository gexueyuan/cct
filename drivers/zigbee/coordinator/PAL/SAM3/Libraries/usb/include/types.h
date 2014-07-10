/************************************************************************************************//**
  \file types.h

  \brief The header file describes global system types and pre-processor words
         which depends on compiler or platform

  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008 , Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
*****************************************************************************************************/

#ifndef _TYPES_H
#define _TYPES_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <pal.h>

#if defined(__ICCAVR__) || defined(__ICCARM__) || defined(__ICCAVR32__)

//\cond
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C)
#include <intrinsics.h>
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) \
  || defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3) || defined(ATMEGA128RFA1)
#include <inavr.h>
#include <ioavr.h>
#include <intrinsics.h>
#include <pgmspace.h>
#elif defined(AT32UC3A0512)
#include <intrinsics.h>
#endif
//\endcond

/**
 * Some preprocessor magic to allow for a header file abstraction of
 * interrupt service routine declarations for the IAR compiler.  This
 * requires the use of the C99 _Pragma() directive (rather than the
 * old #pragma one that could not be used as a macro replacement), as
 * well as two different levels of preprocessor concetanations in
 * order to do both, assign the correct interrupt vector name, as well
 * as construct a unique function name for the ISR.
 *
 * Do *NOT* try to reorder the macros below, or you'll suddenly find
 * out about all kinds of IAR bugs...
 */
#define PRAGMA(x) _Pragma(#x)

// \cond
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C) || defined(AT91SAM3S4B)

#define PROGMEM_DECLARE(x) x
#define FLASH_VAR
#define memcpy_P   memcpy
#define hw_platform_address_size_t    uint32_t
#define BEGIN_PACK PRAGMA(pack(push, 1))
#define END_PACK   PRAGMA(pack(pop))
#define INLINE static inline

#elif defined(AT32UC3A0512)

#define PROGMEM_DECLARE(x) x
#define FLASH_VAR
#define memcpy_P   memcpy
#define hw_platform_address_size_t    uint32_t
#define BEGIN_PACK PRAGMA(pack(push, 1))
#define END_PACK   PRAGMA(pack(pop))
#define INLINE static inline
#define INTERRUPT   __interrupt

#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) \
  || defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3) || defined(ATMEGA128RFA1)

#ifndef __HAS_ELPM__
#define _MEMATTR  __flash
#else /* __HAS_ELPM__ */
#define _MEMATTR  __farflash
#endif /* __HAS_ELPM__ */

#define PROGMEM_DECLARE(x) _MEMATTR x
#define FLASH_VAR _MEMATTR
#define BEGIN_PACK
#define END_PACK
#define INLINE PRAGMA(inline=forced) static

#define ASM     asm
#define __SLEEP   __sleep()

#if defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3)
#define ENABLE_CHANGE_PROTECTION_REGISTER   CCP = 0xD8
#endif

#define ISR(vec) PRAGMA(vector=vec) __interrupt void handler_##vec(void)
#define sei() (__enable_interrupt())
#define cli() (__disable_interrupt())
/** Wait until bit \c bit in IO register \c sfr is clear. */
#define loop_until_bit_is_clear(sfr, bit) do {;} while (sfr & (1 << bit))

#define wdt_reset() (__watchdog_reset())

#define SF_GET_LOW_FUSES() __AddrToZByteToSPMCR_LPM((void __flash *)0x0000, 0x09)
#endif

#if defined(ATMEGA1284)
#define EEMPE  2
#define EEPE   1
#define PSRASY 1
#endif

#if defined(AT90USB1287)
#define UPE1 2
#define USB_GEN_vect USB_General_vect
#define USB_COM_vect USB_Endpoint_Pipe_vect
#endif

// \endcond

#define PACK
#define MAY_ALIAS
#define NOP       __no_operation()
//#define nop() (__no_operation())

#elif defined(__GNUC__)

// \cond
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C)
#elif defined(X86)
#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) \
   || defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3) || defined(ATMEGA128RFA1)
#include <avr/io.h>
#include <avr/pgmspace.h>
#if !defined(ATXMEGA128A1) && !defined(ATXMEGA256A3) && !defined(ATXMEGA256D3)
#include <avr/boot.h>
#endif
#include <avr/interrupt.h>
#include <avr/wdt.h>
#endif
// \endcond

// \cond
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C)

#define PROGMEM_DECLARE(x) x
#define FLASH_VAR
#define memcpy_P   memcpy
#define hw_platform_address_size_t    uint32_t
#define PRAGMA(x) _Pragma(#x)	
#define BEGIN_PACK PRAGMA(pack(push, 1))
#define END_PACK   PRAGMA(pack(pop))
#define PACK __attribute__ ((packed))

#elif defined(ATMEGA1281) || defined(ATMEGA2561) || defined(ATMEGA1284) || defined(AT90USB1287) \
   || defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3) || defined(ATMEGA128RFA1)

#define SF_GET_LOW_FUSES() boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS)
#define PROGMEM_DECLARE(x) x __attribute__((__progmem__))
#define FLASH_VAR PROGMEM
#define hw_platform_address_size_t    uint16_t
#define BEGIN_PACK
#define END_PACK
#define PACK

#define ASM     asm volatile
#define __SLEEP   asm volatile ("sleep")
#define UINT64_MEMCMP

#if defined(ATXMEGA128A1) || defined(ATXMEGA256A3) || defined(ATXMEGA256D3)
#define ENABLE_CHANGE_PROTECTION_REGISTER   CCP = 0xD8
#endif

#elif defined(X86)
#define PROGMEM_DECLARE(x) x
#define FLASH_VAR
#define memcpy_P   memcpy
#define hw_platform_address_size_t    uint32_t
#define BEGIN_PACK
#define END_PACK
#define PACK __attribute__ ((packed))

#endif
// \endcond

#define INLINE static inline __attribute__ ((always_inline))
#define MAY_ALIAS __attribute__((__may_alias__))
#define NOP       asm volatile ("nop")

#else
#define PRAGMA(x) _Pragma(#x)

// \cond
#if defined(AT91SAM7X256) || defined(AT91SAM3S4C) || defined(AT91SAM3S4B)

#define PROGMEM_DECLARE(x) x
#define FLASH_VAR
#define memcpy_P   memcpy
#define hw_platform_address_size_t    uint32_t
#define BEGIN_PACK PRAGMA(pack(push, 1))
#define END_PACK   PRAGMA(pack(pop))
#define INLINE static inline
#endif
#endif

//typedef bool result_t;
//typedef uint64_t BcTime_t;

//BEGIN_PACK
//typedef struct PACK
//{
//    uint16_t val;
//} u16Packed_t;
//typedef struct PACK
//{
//    uint32_t val;
//} u32Packed_t;
//typedef struct PACK
//{
//    uint64_t val;
//} u64Packed_t;
//typedef struct PACK
//{
//    int16_t val;
//}  s16Packed_t;
//typedef struct PACK
//{
//    int32_t val;
//}  s32Packed_t;
//typedef struct PACK
//{
//    int64_t val;
//}  s64Packed_t;
//END_PACK
#if defined __ICCAVR__ || defined __ICCARM__
 typedef uint8_t BitField_t;//keil
#else
typedef unsigned int BitField_t;
#endif
#define BC_SUCCESS false
#define BC_FAIL    true
	



#endif
// eof types.h
