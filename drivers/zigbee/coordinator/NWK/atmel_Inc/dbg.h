/*******************************************************************************//**
  \file dbg.h

  \brief
    A module should use the SYS_writeLog define and should not use writeLog() directly
    Module should define own #define on SYS_WriteLog() to have an opportunity
    to turn on/off logging by setting special define during compilation

    The LOG is turned on by the _SYS_LOG_ON_ define defined in Makefile
  \author
    Atmel Corporation: http://www.atmel.com \n
    Support email: avr@atmel.com

  Copyright (c) 2008-2011, Atmel Corporation. All rights reserved.
  Licensed under Atmel's Limited License Agreement (BitCloudTM).

  \internal
  History:
    29/05/07 D. Ovechkin - Created
    17/01/11 M. Gekk - Guards are added
**********************************************************************************/

#ifndef _DBG_H
#define _DBG_H

/*********************************************************************************
                          Includes section.
**********************************************************************************/
#include <types.h>
#ifdef _SYS_ASSERT_ON_
  #include <halAssert.h>
#endif // _SYS_ASSERT_ON_

/******************************************************************************
                   Define(s) section
******************************************************************************/
/* log type of message  */
typedef enum
{
  HAL_LOG_MESSAGE     = 0x80,
  MAC_LOG_MESSAGE     = 0x81,
  NWK_LOG_MESSAGE     = 0x82,
  APS_LOG_MESSAGE     = 0x83,
  ZDO_LOG_MESSAGE     = 0x84,
  SSP_TC_LOG_MESSAGE  = 0x85,
  ZCL_LOG_MESSAGE     = 0x86,
  APL_LOG_MESSAGE     = 0x87
} LogMessageLevel_t;

#if defined(_SYS_LOG_ON_) && defined(_HAL_LOG_ON_)
  #define HAL_WRITE_LOG(A)    SYS_WriteLog((uint8_t)HAL_LOG_MESSAGE, (uint8_t)A);
#else
  #define HAL_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_MAC_LOG_ON_)
  #define MAC_WRITE_LOG(A)    SYS_WriteLog((uint8_t)MAC_LOG_MESSAGE, (uint8_t)A);
#else
  #define MAC_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_NWK_LOG_ON_)
  #define NWK_WRITE_LOG(A)    SYS_WriteLog((uint8_t)NWK_LOG_MESSAGE, (uint8_t)A);
#else
  #define NWK_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_APS_LOG_ON_)
  #define APS_WRITE_LOG(A)    SYS_WriteLog((uint8_t)APS_LOG_MESSAGE, (uint8_t)A);
#else
  #define APS_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_ZDO_LOG_ON_)
  #define ZDO_WRITE_LOG(A)    SYS_WriteLog((uint8_t)ZDO_LOG_MESSAGE, (uint8_t)A);
#else
  #define ZDO_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_SSP_TC_LOG_ON_)
  #define SSP_TC_WRITE_LOG(A)    SYS_WriteLog((uint8_t)SSP_TC_LOG_MESSAGE, (uint8_t)A);
#else
  #define SSP_TC_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_ZCL_LOG_ON_)
  #define ZCL_WRITE_LOG(A)    SYS_WriteLog((uint8_t)ZCL_LOG_MESSAGE, (uint8_t)A);
#else
  #define ZCL_WRITE_LOG(A)
#endif

#if defined(_SYS_LOG_ON_) && defined(_APL_LOG_ON_)
  #define APL_WRITE_LOG(A)    SYS_WriteLog((uint8_t)APL_LOG_MESSAGE, (uint8_t)A);
#else
  #define APL_WRITE_LOG(A)
#endif

#define assert_static(e)   {enum {_SA_ = 1/(e)};}

/*********************************************************************************
                          Function prototypes section.
**********************************************************************************/
/*  ________________________________ SYS_LOG __________________________________ */
#ifdef _SYS_LOG_ON_
  /******************************************************************************
                     Define(s) section
  ******************************************************************************/
  #define  SYS_INFINITY_LOOP_MONITORING     sysInfinityLoopMonitoring = 0;
  /* 1 = 10 ms */
  #define  TASK_LENGTH        100

  #if defined(_HAL_LOG_INTERFACE_UART0_)
    #define INFORMATION_HANDLER HAL_TaskHandler();
  #elif defined(_HAL_LOG_INTERFACE_UART1_)
    #define INFORMATION_HANDLER HAL_TaskHandler();
  #else
    #define INFORMATION_HANDLER
  #endif

  /******************************************************************************
                     External variables section
  ******************************************************************************/
  extern volatile uint8_t sysInfinityLoopMonitoring;

  /*********************************************************************************
                          Function prototypes section.
  **********************************************************************************/
  /*****************************************************************************//**
   \brief Write log information to defined destination. The destination can be
    UART, EEPROM, Ethernet... The destination is determined by the define
    during compilation
    \param[in]
      leyerID - identical definition of application layer;
    \param[in]
      message - information byte (must be less then 0x80);
  **********************************************************************************/
  void SYS_WriteLog(uint8_t leyerID, uint8_t message);

  /*****************************************************************************//**
   \brief Initialization of logging system.
  **********************************************************************************/
  void SYS_InitLog(void);

  #if defined(_HAL_LOG_INTERFACE_UART0_) || defined(_HAL_LOG_INTERFACE_UART1_)
    /**************************************************************************//**
    \brief HAL task handler.
    ******************************************************************************/
    void HAL_TaskHandler(void);
  #endif

  /*********************************************************************************
                     Inline static functions section
  **********************************************************************************/
  /*****************************************************************************//**
   \brief Monitoring infinity loop in the soft.
  **********************************************************************************/
  INLINE void SYS_InfinityLoopMonitoring(void)
  {
    sysInfinityLoopMonitoring++;
    if (sysInfinityLoopMonitoring > TASK_LENGTH)
    {
      INFORMATION_HANDLER
    }
  }
#else
  INLINE void SYS_InitLog(void){}
  INLINE void SYS_InfinityLoopMonitoring(void){}
  INLINE void SYS_WriteLog(uint8_t leyerID, uint8_t message){(void)leyerID; (void)message;}
  #define  SYS_INFINITY_LOOP_MONITORING
#endif // _SYS_LOG_ON_



/*  ________________________________ SYS_ASSERT _______________________________ */
#ifdef _SYS_ASSERT_ON_
  /*********************************************************************************
     ASSERT is used for debugging wrong conditions
     Dbg codes are in each layers.
  *********************************************************************************/
  /*********************************************************************************
    Function catches unexpected conditions in the logic.
    Parameters:
      condition - TRUE or FALSE. FALSE means problems in the logic.
      dbgCode   - assert's unique code.
                  dbgCode ranges: 0x1000 - 0x1fff - MAC
                                  0x2000 - 0x2fff - HAL
                                  0x3000 - 0x3fff - NWK
                                  0x4000 - 0x4fff - APS
                                  0x5000 - 0x5fff - ZDO
                                  0x6000 - 0x6fff - Configuration Server
                                  0x7000 - 0x7fff - SSP/TC
                                  0x8000 - 0x8fff - System Environment
                                  0x9000 - 0x9fff - BSP
                                  0xf000 - 0xfffe - APL
    Returns:
      none.
  *********************************************************************************/
  void assert(bool condition, uint16_t dbgCode);
  // Inline Assert
  #define ASSERT(condition, dbgCode)     halAssert(condition, dbgCode);

#else // !_SYS_ASSERT_ON_
 // #define ASSERT(condition, dbgCode) {if(condition){}}
#ifndef assert
  #define assert(condition, dbgCode) {if(condition){}}
#endif
#endif // _SYS_ASSERT_ON_

#define assert_static(e)   {enum {_SA_ = 1/(e)};}

#if defined _SYS_ASSERT_ON_
#define TOP_GUARD_VALUE 0x55U
#define BOTTOM_GUARD_VALUE 0xAAU

#define TOP_GUARD uint8_t topGuard;
#define BOTTOM_GUARD uint8_t bottomGuard;
#define INIT_GUARDS(obj) \
  {\
    (obj)->topGuard = TOP_GUARD_VALUE;\
    (obj)->bottomGuard = BOTTOM_GUARD_VALUE;\
  }

#define CHECK_GUARDS(obj, assertNum) \
  assert((TOP_GUARD_VALUE == (obj)->topGuard) \
    && (BOTTOM_GUARD_VALUE == (obj)->bottomGuard), assertNum)

#define GUARDED_STRUCT(obj) \
  (obj) = \
  { \
    .topGuard = TOP_GUARD_VALUE, \
    .bottomGuard = BOTTOM_GUARD_VALUE \
  }

#else
#define TOP_GUARD
#define BOTTOM_GUARD
#define INIT_GUARDS(obj) ((void)0)
#define CHECK_GUARDS(obj, assertNum) ((void)0)
#define GUARDED_STRUCT(obj) (obj)
#endif

#endif // _DBG_H
//end of dbg.h

