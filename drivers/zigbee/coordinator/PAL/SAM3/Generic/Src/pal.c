/**
 * @file pal.c
 *
 * @brief General PAL functions for SAM3 MCUs
 *
 * This file implements generic PAL function for SAM3 MCUs.
 *
 * $Id: pal.c 30341 2012-01-20 07:31:44Z mahendran.p $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */
/* === Includes ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "pal.h"
#include "pal_config.h"
#include "pal_timer.h"
#include "pal_internal.h"
#ifdef INTERN_FLASH_ENABLED
#include "pal_flash.h"
#endif
#include "app_uart.h"
#include "tal.h"
#include "tal_internal.h"
#ifndef ATMEL_RF
#include "Uz2400D.h"
#endif
/* === Macros =============================================================== */

/* Mask to act on all the interrupts at once. */
#define ALL_INTERRUPTS_MASK         (0xFFFFFFFF)

/* Maximum number of interrupt sources that can be defined. This
constant can be increased, but the current value is the smallest possible
that will be compatible with all existing projects */
#define MAX_INTERRUPT_SOURCES       (1)//7

/* Watchdog control register key password*/
#define WDT_CR_PASSWORD             (0xA5)
/* === Globals ============================================================= */

#ifdef TIMESTAMP_UPPER_16BIT_SW
/*
 * This is the most significant part of the system time.
 */
volatile uint16_t sys_time;
#endif

/* List of interrupt sources */
static interruptSource_t pSources[MAX_INTERRUPT_SOURCES];

/* Number of currently defined interrupt sources */
static uint32_t numSources = 0;

/* === Prototypes ========================================================== */
//static void watchdog_disable(void);
#ifdef STM32F10X_MD
static void pal_pio_setperiphA(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute);
//static void pal_pio_setperiphB(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute);
#elif defined (STM32F051) || defined (STM32F4XX)
static void pal_pio_setperiphA(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute, GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType);
#endif


//static void pal_pio_setperiphC(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute);
//static void pal_pio_setperiphD(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute);

static void pal_pio_setinput(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute);
static void pal_pio_setoutput(GPIO_TypeDef *pio, uint16_t mask, BitAction defaultValue,
                              GPIOMode_TypeDef enableMultiDrive);
static void pal_pio_interrupt_handler(uint32_t id, GPIO_TypeDef *pPio);
//void PIOA_IrqHandler(void);
void PIOB_IrqHandler(void);
void PIOC_IrqHandler(void);

#ifdef WATCHDOG
static void pal_wdt_init(void);
static void pal_wdt_reset(void);
#else
static void pal_wdt_disable(void);
#endif

/* === Implementation ====================================================== */

/**
 * @brief Initialization of PAL
 *
 * This function initializes the PAL.
 *
 * @return MAC_SUCCESS  if PAL initialization is successful, FAILURE otherwise
 */
retval_t pal_init(void)
{

#ifdef WATCHDOG
    /* Watchdog Initialization. Enables the watchdog and sets the timeout period */
    pal_wdt_init();
#else
    /* Disabling watch dog*/
    pal_wdt_disable();
#endif

    /* Initialising PIO pins */
    gpio_init();

    /* Initialising PIO interrupts with Zero prority*/
    pal_pio_initialize_interrupts(1);

    /* Initialising tranceiver interface */
    trx_interface_init();

    /* Initialising timer for PAL */
    timer_init();

//#ifdef RTC
//    RTC_EXTI_INITIAL(ENABLE);
//#endif

    return MAC_SUCCESS;
}


/**
 * @brief Services timer
 *
 * This function calls timer handling functions.
 */
void pal_task(void)
{
#if (TOTAL_NUMBER_OF_TIMERS > 0)
    timer_service();
#endif
#ifdef WATCHDOG
    pal_wdt_reset();
#endif
}


/**
 * @brief Get data from persistence storage
 *
 * @param[in]  ps_type Persistence storage type
 * @param[in]  start_addr Start offset within EEPROM
 * @param[in]  length Number of bytes to read from EEPROM
 * @param[out] value Data from persistence storage
 *
 * @return MAC_SUCCESS  if everything went OK else FAILURE
 */
retval_t pal_ps_get(ps_type_t ps_type, uint16_t start_addr, uint16_t length, void *value)
{
    /* Board does not has EEPROM */
#if (EXTERN_EEPROM_AVAILABLE == 1)
    if (ps_type == EXTERN_EEPROM)
    {
        /* eeprom get function */
    }
    else
#endif

        if (ps_type == INTERN_EEPROM)
        {
#ifdef INTERN_FLASH_ENABLED
            pal_flash_read(start_addr, length, value);
#endif
        }
        else
        {
            return MAC_INVALID_PARAMETER;
        }
    /* Keep compiler happy */
    //start_addr = start_addr;
    //length = length;
    //value = value;

    return MAC_SUCCESS;
}


/**
 * @brief Write data to persistence storage
 *
 * @param[in]  start_addr Start address offset within EEPROM
 * @param[in]  length Number of bytes to be written to EEPROM
 * @param[in]  value Data to persistence storage
 *
 * @return MAC_SUCCESS  if everything went OK else FAILURE
 */
retval_t pal_ps_set(uint16_t start_addr, uint16_t length, void *value)
{
    /* Board does not has EEPROM */
#ifdef INTERN_FLASH_ENABLED
    uint8_t status;
    status = pal_flash_write(start_addr, value, length);
    ASSERT(!status);
    if (!status)
    {
        return MAC_SUCCESS;
    }
#endif
    /* Keep compiler happy. */
    //start_addr = start_addr;
    //length = length;
    //value = value;

    return FAILURE;
}


/**
 * @brief Alert indication
 *
 * This Function can be used by any application to indicate an error condition.
 * The function is blocking and never returns.
 */
void pal_alert(void)
{
#if (DEBUG > 0)
    bool debug_flag = false;
#endif
   // ALERT_INIT();

    while (1)
    {
     //   pal_timer_delay(0xFFFF);
     //   ALERT_INDICATE();

#if (DEBUG > 0)
        /* Used for debugging purposes only */
        if (debug_flag == true)
        {
            break;
        }
#endif
    }
}


/**
 * @brief Get Periph Status for the given peripheral ID.
 *
 * @param peripheral_id  Peripheral ID (ID_xxx).
 * @return 32bit value - status of the clock
 */
uint32_t pal_get_peripheral_clock_status(RCC_ClocksTypeDef  *RCC_Clocks)
{
#if (DEBUG > 0)
    ASSERT(peripheral_id < 35);
#endif
    RCC_GetClocksFreq(RCC_Clocks);
    return 1;
}


/**
 * @brief Enables the clock of a peripheral
 *
 * This function enables the clock of peripheral. The peripheral ID is used
 * to identify which peripheral is targetted.
 *
 * @param peripheral_id  Peripheral ID (ID_xxx).
 */
void pal_peripheral_clock_enable(uint8_t peripheral_id)
{
#if (DEBUG > 0)
    ASSERT(peripheral_id < 35);
#endif
    switch (peripheral_id)
    {
    case ID_PIOA:
#ifdef STM32F10X_MD      
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#elif defined (STM32F051)
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);      
#elif defined (STM32F4XX)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);        
#endif      
      break;
    case ID_PIOB:
#ifdef STM32F10X_MD      
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#elif defined (STM32F051)
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);      
#elif defined (STM32F4XX)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);        
#endif        

      break;
    case ID_PIOC:
#ifdef STM32F10X_MD      
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
#elif defined (STM32F051)
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);   
#elif defined (STM32F4XX)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);        
#endif          

      break;
    case ID_PIOD:
#ifdef STM32F10X_MD      
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
#elif defined (STM32F051)
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);   
#elif defined (STM32F4XX)
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);       
#endif          

      break;
    case ID_SPI:
#ifndef STM32F405xx    
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
#else
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
#endif
      break;
    case ID_TC2:
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    break;
    
    default:
    break;
      
      
    }
}


/**
 * @brief Disables the clock of a peripheral. The peripheral ID is used
 * to identify which peripheral is targetted.
 *
 * @param peripheral_id  Peripheral ID (ID_xxx).
 */
void pal_peripheral_clock_disable(uint32_t RCC_APB2Periph)
{
#if (DEBUG > 0)
    ASSERT(peripheral_id < 35);
#endif
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, DISABLE);
}


/**
 * @brief Configures a list of Pin instances
 *
 * This function configures list of pins each of which can either hold a single
 * pin or a group of pins, depending on the mask value; all pins are configured
 * by this function. The size of the array must also be provided and is easily
 * computed using PIO_LISTSIZE whenever its length is not known in advance.
 *
 * @param size  Size of the Pin list (calculated using PIO_LISTSIZE)
 * @param list  Pointer to a list of Pin instances.
 *
 * @return 1 if the pins have been configured properly; otherwise 0
 */
#ifdef STM32F10X_MD
bool pal_pio_configure(const pin_t *list, uint32_t size)
{
    while (size > 0)
    {
        switch (list->type)
        {
            case PIO_PERIPH_A:
            case PIO_PERIPH_B:              
            case PIO_PERIPH_C:
            case PIO_PERIPH_D:  
                pal_pio_setperiphA(list->pio,
                                   list->mask,
                                   (list->attribute));
                break;


            case PIO_INPUT:
                pal_pio_setinput(list->pio,
                                 list->mask,
                                 list->attribute);
                break;

            case PIO_OUTPUT_0:
                pal_pio_setoutput(list->pio,
                                  list->mask,
                                  (Bit_RESET),
                                  (list->attribute ));
                break;
            case PIO_OUTPUT_1:
                pal_pio_setoutput(list->pio,
                                  list->mask,
                                  (Bit_SET),
                                  (list->attribute ));
                break;

            default:
                return 0;
        }

        list++;
        size--;
    }

    return 1;
}
#elif defined (STM32F051) || defined (STM32F4XX)
bool pal_pio_configure(const pin_t *list, uint32_t size)
{
    while (size > 0)
    {
        switch (list->type)
        {
            case PIO_PERIPH_A:
            case PIO_PERIPH_B:
            case PIO_PERIPH_C:
            case PIO_PERIPH_D:
                pal_pio_setperiphA(list->pio,
                                   list->mask,
                                   list->attribute,
                                   list->outType,
                                   list->pupdType);
                break;
   

            case PIO_INPUT:
                pal_pio_setinput(list->pio,
                                 list->mask,
                                 list->attribute);
                break;

            case PIO_OUTPUT_0:
                pal_pio_setoutput(list->pio,
                                  list->mask,
                                  (Bit_RESET),
                                  (list->attribute ));
                break;
            case PIO_OUTPUT_1:
                pal_pio_setoutput(list->pio,
                                  list->mask,
                                  (Bit_SET),
                                  (list->attribute ));
                break;

            default:
                return 0;
        }

        list++;
        size--;
    }

    return 1;
}
#endif

/**
 * @brief Configures one or more pin(s) of PIOA
 *
 * This function configures one or more pin(s) of a PIO controller as
 * being controlled by peripheral A. Optionally, the corresponding internal
 * pull-up(s) can be enabled.
 *
 * @param pio  Pointer to a PIO controller.
 * @param mask  Bitmask of one or more pin(s) to configure.
 * @param enablePullUp  Indicates if the pin(s) internal pull-up shall be
 *                      configured.
 */
#ifdef STM32F10X_MD
static void pal_pio_setperiphA(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = mask;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   

    GPIO_InitStructure.GPIO_Mode = attribute;

    GPIO_Init(pio, &GPIO_InitStructure);
}
#elif defined (STM32F051) || defined (STM32F4XX)
static void pal_pio_setperiphA(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute, GPIOOType_TypeDef outType, GPIOPuPd_TypeDef pupdType)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = mask;
#ifdef STM32F4XX 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
#else    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
#endif    
    GPIO_InitStructure.GPIO_Mode = attribute;
    GPIO_InitStructure.GPIO_OType = outType;
    GPIO_InitStructure.GPIO_PuPd  = pupdType;
    GPIO_Init(pio, &GPIO_InitStructure);
}
#endif

/**
 * @brief Configures one or more pin(s) of PIOB
 *
 * This function configures one or more pin(s) of a PIO controller as
 * being controlled by peripheral B. Optionally, the corresponding internal
 * pull-up(s) can be enabled.
 *
 * @param pio  Pointer to a PIO controller.
 * @param mask  Bitmask of one or more pin(s) to configure.
 * @param enablePullUp  Indicates if the pin(s) internal pull-up shall be
 *                      configured.
 */
//#ifdef STM32F10X_MD
//static void pal_pio_setperiphB(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    
//
//    GPIO_InitStructure.GPIO_Pin = mask;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
//
//    GPIO_InitStructure.GPIO_Mode = attribute;
//
//
//    GPIO_Init(pio, &GPIO_InitStructure);
//}
//#endif

/**
 * @brief Configures one or more pin(s) of PIOC
 *
 * This function configures one or more pin(s) of a PIO controller as
 * being controlled by peripheral C. Optionally, the corresponding internal
 * pull-up(s) can be enabled.
 *
 * @param pio  Pointer to a PIO controller.
 * @param mask  Bitmask of one or more pin(s) to configure.
 * @param enablePullUp  Indicates if the pin(s) internal pull-up shall be
 *                      configured.
 */
//static void pal_pio_setperiphC(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//        GPIO_InitStructure.GPIO_Pin = mask;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
//
//        GPIO_InitStructure.GPIO_Mode = attribute;
//
//    
//        GPIO_Init(pio, &GPIO_InitStructure);
//}
//
//
///**
// * @brief Configures one or more pin(s) of PIOD
// *
// * This function configures one or more pin(s) of a PIO controller as
// * being controlled by peripheral D. Optionally, the corresponding internal
// * pull-up(s) can be enabled.
// *
// * @param pio  Pointer to a PIO controller.
// * @param mask  Bitmask of one or more pin(s) to configure.
// * @param enablePullUp  Indicates if the pin(s) internal pull-up shall be
// *                      configured.
// */
//static void pal_pio_setperiphD(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    GPIO_InitStructure.GPIO_Pin = mask;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
//
//    GPIO_InitStructure.GPIO_Mode = attribute;
//
//
//    GPIO_Init(pio, &GPIO_InitStructure);
//}


/**
 * @brief Configures pin(s) of a PIO controller as inputs.
 *
 * This function configures one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s)
 * can be enabled.
 *
 * @param pio  Pointer to a PIO controller.
 * @param mask  Bitmask of one or more pin(s) to configure.
 * @param attribute  Indicates if the pin(s) de-glitch/de-bounce shall be
 *                      configured.
 */
static void pal_pio_setinput(GPIO_TypeDef *pio, uint16_t mask, GPIOMode_TypeDef attribute)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = mask;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   

    GPIO_InitStructure.GPIO_Mode = attribute;   
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(pio, &GPIO_InitStructure);
}


/**
 * @brief Configures pin(s) of a PIO controller as outputs
 *
 * This function configures one or more pin(s) or a PIO controller as outputs
 * Optionally,the corresponding internal pull-up(s) and glitch filter(s)
 * can be enabled.
 *
 * @param pio  Pointer to a PIO controller.
 * @param mask  Bitmask indicating which pin(s) to configure.
 * @param defaultValue  Default level on the pin(s).
 * @param enableMultiDrive  Indicates if the pin(s) shall be configured as
 *                        open-drain.
 * @param enablePullUp  Indicates if the pin shall have its pull-up activated.
 */
static void pal_pio_setoutput(GPIO_TypeDef *pio, uint16_t mask, BitAction defaultValue,
                              GPIOMode_TypeDef enableMultiDrive)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = mask;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   

    GPIO_InitStructure.GPIO_Mode = enableMultiDrive;   

    GPIO_Init(pio, &GPIO_InitStructure);
    
    GPIO_WriteBit(pio, mask, defaultValue);
}


/**
 * @brief Check the Pin instance currently have a high or not
 *
 * This function returns 1 if one or more PIO of the given Pin instance
 * currently have a high level; otherwise returns 0. This method returns
 * the actual value that is being read on the pin.
 *
 * @param pin  Pointer to a Pin instance describing one or more pins.
 * @return 1 if the Pin instance contains at least one PIO that currently has
 *  a high level; otherwise 0.
 */
bool pal_pio_get(const pin_t *pin)
{
    uint32_t reg;
    /* Check the pin type is an output and get the Data register*/
    if ((pin->type == PIO_OUTPUT_0) || (pin->type == PIO_OUTPUT_1))
    {
        reg = pin->pio->ODR;
    }
    else
    {
        reg = pin->pio->IDR;
    }
    /* Check the level of the pin */
    if ((reg & pin->mask) == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


/**
 * @brief Initializes the PIO interrupt management logic
 *
 * @param priority  PIO controller interrupts priority.
 */
void pal_pio_initialize_interrupts(uint32_t priority)
{
    /* Reset sources */
    //numSources = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
    
    NVIC_InitTypeDef NVIC_InitStructure;
    // 使能EXTI0中断 
#ifndef PA
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; 
#else
#ifdef STM32F10X_MD      
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
#elif defined (STM32F051)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;   
#elif defined (STM32F4XX)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;         
#endif      
#endif
#if defined (STM32F10X_MD) || defined (STM32F4XX)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority; // 指定抢占式优先级别1 
    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 指定响应优先级别0
#elif defined (STM32F051)
    NVIC_InitStructure.NVIC_IRQChannelPriority = priority; // 指定抢占式优先级别1     
#endif    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    
    // 使能EXTI9_5中断 
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 指定抢占式优先级别0 
//   // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 指定响应优先级别1 
//   // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//    NVIC_Init(&NVIC_InitStructure); 


    /* Configure PIO interrupt sources of PIOB */
//    pal_peripheral_clock_enable(ID_PIOB);
//    PIOB->PIO_ISR;
//    PIOB->PIO_IDR = ALL_INTERRUPTS_MASK;
//    NVIC_DisableIRQ(PIOB_IRQn);
//    NVIC_ClearPendingIRQ(PIOB_IRQn);
//    NVIC_SetPriority(PIOB_IRQn, priority);
//    NVIC_EnableIRQ(PIOB_IRQn);
//
//    /* Configure PIO interrupt sources of PIOC */
//    pal_peripheral_clock_enable(ID_PIOC);
//    PIOC->PIO_ISR;
//    PIOC->PIO_IDR = ALL_INTERRUPTS_MASK;
//    NVIC_DisableIRQ(PIOC_IRQn);
//    NVIC_ClearPendingIRQ(PIOC_IRQn);
//    NVIC_SetPriority(PIOC_IRQn, priority);
//    NVIC_EnableIRQ(PIOC_IRQn);
}


/**
 * @Configures a PIO interrupt
 *
 * This function configures a PIO or a group of PIO to generate an interrupt
 * on status change. The provided interrupt handler will be called with the
 * triggering pin as its parameter (enabling different pin instances
 * to share the same handler
 *
 * @param pPin  Pointer to a Pin instance.
 * @param handler  Interrupt handler function pointer.
 */
void pal_pio_configure_interrupt(const pin_t *pPin, void (*handler)(const pin_t *))
{
    uint8_t index;
    bool configured = false;
   // GPIO_TypeDef *pio = pPin->pio;
    interruptSource_t *pSource;

    /* Define new source */
    for (index = 0; index < numSources; index++)
    {
        if (pSources[index].pPin == pPin)
        {
            configured = true;
            pSources[index].pio_handler = handler;
        }
    }
    /* Check the pin already configured or not*/
    if (!configured)
    {
        pSource = &(pSources[numSources]);
        pSource->pPin = pPin;
        pSource->pio_handler = handler;
        numSources++;
    }
    EXTI_InitTypeDef EXTI_InitStruct;
#ifdef PA
#ifdef STM32F10X_CL
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource2);
#else
#ifdef    STM32F10X_MD
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource2);
#elif defined     (STM32F051) || defined (STM32F4XX)
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);    
#endif  
#endif
#endif
    EXTI_InitStruct.EXTI_Line = pPin->mask;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
#ifdef ATMEL_RF
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
#else
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
#endif
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    /* PIO3 with additional interrupt support */
    /* Configure additional interrupt mode registers */
//    if (pPin->attribute & PIO_IT_AIME)
//    {
//        /* Enable additional interrupt mode */
//        pio->PIO_AIMER       = pPin->mask;
//
//        /* if bit field of selected pin is 1, set as Rising Edge/High level
//          detection event */
//        if (pPin->attribute & PIO_IT_RE_OR_HL)
//        {
//            pio->PIO_REHLSR    = pPin->mask;
//        }
//        else
//        {
//            pio->PIO_FELLSR     = pPin->mask;
//        }
//
//        /* if bit field of selected pin is 1, set as edge detection source*/
//        if (pPin->attribute & PIO_IT_EDGE)
//        {
//            pio->PIO_ESR     = pPin->mask;
//        }
//        else
//        {
//            pio->PIO_LSR     = pPin->mask;
//        }
//    }
//    else
//    {
//        /* disable additional interrupt mode */
//        pio->PIO_AIMDR       = pPin->mask;
//    }
}


/**
 * @brief Disables a given interrupt source, with no added side effects.
 * @param pPin  Interrupt source to disable
 */
void pal_pio_disable_interrupt(const pin_t *pPin)
{
   EXTI->IMR &= ~pPin->mask;;
}


/**
 * @brief Enables the given interrupt source
 *
 * This function enables the given interrupt source if it has been configured.
 * The status register of the corresponding PIO controller is cleared prior
 * to enabling the interrupt
 *
 * @param pPin  Interrupt source to enable
 */
void pal_pio_enable_interrupt(const pin_t *pPin)
{
     EXTI->IMR |= pPin->mask;;
}


/**
 * @brief Parallel IO Controller A interrupt handler
 */
#ifndef PA
void EXTI3_IRQHandler(void)
{
    pal_pio_interrupt_handler(ID_PIOA, GPIOA);
}
#else
#ifdef STM32F10X_MD 
void EXTI2_IRQHandler(void)
{
#ifdef STM32F10X_CL
    pal_pio_interrupt_handler(ID_PIOD, GPIOD);
#else
    pal_pio_interrupt_handler(ID_PIOB, GPIOB);
#endif
}
#elif defined (STM32F051)
    void EXTI0_1_IRQHandler(void)
    {
          pal_pio_interrupt_handler(ID_PIOB, GPIOB);
#ifndef ATMEL_RF
            spi_sw(RXFLUSH, 0x01);
#endif
    }
#elif defined (STM32F4XX)
    void EXTI0_IRQHandler(void)
    {
          pal_pio_interrupt_handler(ID_PIOB, GPIOB);
#ifndef ATMEL_RF
            spi_sw(RXFLUSH, 0x01);
#endif
    }
#endif   
#endif


/**
 * @brief Parallel IO Controller B interrupt handler
 */
void PIOB_IrqHandler(void)
{
    pal_pio_interrupt_handler(ID_PIOB, GPIOB);
}


/**
 * @brief Parallel IO Controller C interrupt handler
 */
void PIOC_IrqHandler(void)
{
    pal_pio_interrupt_handler(ID_PIOC, GPIOC);
}


/**
 * @brief Handles all interrupts on the given PIO controller.
 * @param id  PIO controller ID.
 * @param pPio  PIO controller base address.
 */
void pal_pio_interrupt_handler(uint32_t id, GPIO_TypeDef *pPio)
{
    uint32_t status;
    uint32_t index;

    /* Read PIO controller status */
    status =  EXTI->PR & EXTI_Line_All16;
    status &= EXTI->IMR;
#ifndef PA
    EXTI->PR = EXTI_Line3;
#else
#ifdef STM32F10X_MD    
    EXTI->PR = EXTI_Line2;
#elif defined (STM32F051) || defined (STM32F4XX)    
    EXTI->PR = EXTI_Line0;
#endif    
#endif
 //void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)     
      

    /* Check pending events */
    if (status != 0)
    {
        /* Find triggering source */
        index = 0;
        while (status != 0)
        {
            /* There cannot be an unconfigured source enabled */
            /* Source is configured on the same controller */
            if (pSources[index].pPin->id == id)//TODO last problem cloudm
            {
                /* Source has PIOs whose status have changed */
                if ((status & pSources[index].pPin->mask) != 0)
                {
                    pSources[index].pio_handler(pSources[index].pPin);
                    status &= ~(pSources[index].pPin->mask);
                }
            }
            index++;
        }
    }
}


/**
 * @brief Initialize the watchdog timer
 */
#if defined(WATCHDOG)
static void pal_wdt_init(void)
{
	//启动独立看门狗
	 IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //访问之前要首先使能寄存器写


	 IWDG_SetPrescaler(IWDG_Prescaler_64);//64分频 一个周期1.6ms
	 IWDG_SetReload(800);//最长12位 [0,4096] 800*1.6=1.28S
	 /* Reload IWDG counter */
	 IWDG_ReloadCounter();
	 IWDG_Enable();// Enable IWDG (the LSI oscillator will be enabled by hardware)
	 IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable); //访问之前要首先使能寄存器写

	//因为独立看门狗使用的是LSI，所以最好程序启动的时候，使时钟源稳定:

	/* LSI的启动*/
	 RCC_LSICmd(ENABLE);//打开LSI
	 while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);//等待直到LSI稳定
}

/**
 * @brief Resets the watchdog timer
 */
static void pal_wdt_reset(void)
{
	IWDG_ReloadCounter();
}

#else /* #if defined(WATCHDOG) || defined(DOXYGEN) */

/**
 * @brief Disable watchdog.
 */
static void pal_wdt_disable(void)
{
//    Wdt *pWDT = WDT;
//
//    pWDT->WDT_MR = WDT_MR_WDDIS;
}
#endif
/* EOF */
