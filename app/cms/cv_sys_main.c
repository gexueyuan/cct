/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

extern void rt_platform_init(void);
extern void param_init(void);
extern void gps_init(void);
extern void voc_init(void);
extern void led_init(void);
extern void vam_init(void);
extern void vsa_init(void);
extern void sys_init(void);
extern void gsnr_init(void);
extern int rt_key_init(void);

cms_global_t cms_envar, *p_cms_envar;


void global_init(void)
{
    p_cms_envar = &cms_envar;
    memset(p_cms_envar, 0, sizeof(cms_global_t));
}


void rt_init_thread_entry(void *parameter)
{
    global_init();
    param_init();

    gps_init();
  	nmea_init();
    voc_init();
    led_init();
	rt_key_init();
    
    vam_init();
    vsa_init();    
    sys_init();
    gsnr_init();
    //quit...
}

ALIGN(RT_ALIGN_SIZE)
static char thread_zigbee_stack[1024];
struct rt_thread thread_zigbee;
static void rt_thread_entry_zigbee(void* parameter)
{

      extern void zigbee_main(void);
      zigbee_main();	
}

int rt_application_init(void)
{
    rt_thread_t tid;

	rt_components_init();
	rt_platform_init();

    rt_kprintf("system clock: %d\n", SystemCoreClock);

    tid = rt_thread_create("t-init",
                           rt_init_thread_entry, RT_NULL,
                           RT_INIT_THREAD_STACK_SIZE, RT_INIT_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL)
    rt_thread_startup(tid);
	
    rt_thread_init(&thread_zigbee,
                   "zigbee",
                   rt_thread_entry_zigbee,
                   RT_NULL,
                   &thread_zigbee_stack[0],
                   sizeof(thread_zigbee_stack),11,5);
	
    rt_thread_startup(&thread_zigbee);
    return 0;
}


/*@}*/
