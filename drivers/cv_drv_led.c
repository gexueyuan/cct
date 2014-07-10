/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_led.c
 @brief  : this file include the LED display functions
 @author : wangyifeng
 @history:
           2014-6-30    wangyifeng    Created file
           ...
******************************************************************************/
#include <string.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "led.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

void led_init(void)
{
    STM_EVAL_LEDInit(LED_RED);
    STM_EVAL_LEDInit(LED_BLUE);
    STM_EVAL_LEDInit(LED_GREEN);
    STM_EVAL_LEDOff(LED_RED);
    STM_EVAL_LEDOff(LED_BLUE);
    STM_EVAL_LEDOff(LED_GREEN);
}


void led_on(uint32_t led)
{
    if (led < LEDn){
        STM_EVAL_LEDOn((Led_TypeDef)led);
    }
}

void led_off(uint32_t led)
{
    if (led < LEDn){
        STM_EVAL_LEDOff((Led_TypeDef)led);
    }
}

void led_blink(uint32_t led)
{
    if (led < LEDn){
        STM_EVAL_LEDBlink((Led_TypeDef)led);
    }
}

