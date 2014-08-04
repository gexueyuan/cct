/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_manager.c
 @brief  : this file include the system manage functions
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           ...
******************************************************************************/
#include <stdio.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "led.h"

#define HUMAN_ITERFACE_DEFAULT         SECOND_TO_TICK(1)

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
extern const unsigned char notice_16k_8bits[];
extern const unsigned char voice_16k_8bits[];
extern const unsigned int notice_16k_8bitsLen;
extern const unsigned int voice_16k_8bitsLen;
extern void voc_play(uint32_t sample_rate, uint8_t *p_data, uint32_t length);
extern void led_on(uint32_t led);
extern void led_off(uint32_t led);
extern void led_blink(uint32_t led);

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

rt_err_t sys_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv)
{
    rt_err_t err = RT_ENOMEM;
    sys_msg_t msg;

    msg.id = msg_id;
    msg.len = msg_len;
    msg.argc = msg_argc;
    msg.argv = msg_argv;
    err = rt_mq_send(p_sys->queue_sys_mng, &msg, sizeof(sys_msg_t));

    if (err != RT_EOK){
        rt_kprintf("%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_id);
    }

    return err;
}

rt_err_t hi_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv)
{
    rt_err_t err = RT_ENOMEM;
    sys_msg_t msg;

    msg.id = msg_id;
    msg.len = msg_len;
    msg.argc = msg_argc;
    msg.argv = msg_argv;
    err = rt_mq_send(p_sys->queue_sys_hi, &msg, sizeof(sys_msg_t));

    if (err != RT_EOK){
        rt_kprintf("%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_id);
    }

    return err;
}


void sys_manage_proc(sys_envar_t *p_sys, sys_msg_t *p_msg)
{
    switch(p_msg->id){
        case SYS_MSG_INITED:
            rt_kprintf("%s: initialize complete\n", __FUNCTION__);

            vam_start();
            vsa_start();

            hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,HI_OUT_GPS_LOST, 0);

            break;

        case SYS_MSG_START_ALERT:
            rt_kprintf("%s:alert start!!!.\n", __FUNCTION__);
            rt_mq_send(p_sys->queue_sys_hi, p_msg, sizeof(sys_msg_t));
            {
                uint32_t type = 0;

                if (p_msg->argc == VSA_ID_CRD){
                    type = HI_OUT_CRD_ALERT;
                }
                else if (p_msg->argc == VSA_ID_VBD){
                    type = HI_OUT_VBD_ALERT;
                }
                else if (p_msg->argc == VSA_ID_EBD){
                    type = HI_OUT_EBD_ALERT;
                }
                
                hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,type, 0);
            }

            break;
            
        case SYS_MSG_STOP_ALERT:
            rt_kprintf("%s:alert stop.\n", __FUNCTION__);
            hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,HI_OUT_CANCEL_ALERT, 0);
            break;

        case SYS_MSG_GPS_UPDATE:
            hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,\
                ((p_msg->argc == 0)? HI_OUT_GPS_LOST:HI_OUT_GPS_CAPTURED) , 0);
            break;

        default:
            break;
    }
}

void rt_sys_thread_entry(void *parameter)
{
    rt_err_t err;
    sys_msg_t msg, *p_msg = &msg;
    sys_envar_t *p_sys = (sys_envar_t *)parameter;

    rt_kprintf("%s: ---->\n", __FUNCTION__);

    p_msg->id = SYS_MSG_INITED;
    p_msg->len = 0;
    err = rt_mq_send(p_sys->queue_sys_mng, p_msg, sizeof(sys_msg_t));
    RT_ASSERT(err == RT_EOK);

	while(1){
        err = rt_mq_recv(p_sys->queue_sys_mng, p_msg, sizeof(sys_msg_t), RT_WAITING_FOREVER);
        if (err == RT_EOK){
            sys_manage_proc(p_sys, p_msg);
        }
        else{
            rt_kprintf("%s: rt_mq_recv error [%d]\n", __FUNCTION__, err);
        }
	}
}


void timer_human_interface_callback(void* parameter)
{
    sys_envar_t *p_sys = (sys_envar_t *)parameter;
    
#if 0    /* for debug only */
    rt_err_t err = RT_ENOMEM;
    sys_msg_t msg = {0,0,0,0};


    p_sys->hi_timer_cnt++;

    if (p_sys->hi_timer_cnt%5 == 0){
        msg.id = SYS_MSG_START_ALERT;
        msg.argc = VSA_ID_VBD;
    }

    if (msg.id != 0){
        err = rt_mq_send(p_sys->queue_sys_hi, &msg, sizeof(sys_msg_t));

        if (err != RT_EOK){
            rt_kprintf("%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg.id);
        }
    }
#else
    p_sys = p_sys;
#endif
}

void sys_human_interface_proc(sys_envar_t *p_sys, sys_msg_t *p_msg)
{
    if (p_msg->id == SYS_MSG_HI_OUT_UPDATE){
        switch(p_msg->argc){
            case HI_OUT_CRD_ALERT:
                voc_play(16000, (uint8_t *)notice_16k_8bits, notice_16k_8bitsLen);
                p_sys->led_blink_duration[LED_RED] = 0xFFFF;
                p_sys->led_blink_period[LED_RED] = 15;
                p_sys->led_blink_cnt[LED_RED] = 0;

                p_sys->led_blink_period[LED_GREEN] = 0; /* turn off gps led */

                break;
            case HI_OUT_VBD_ALERT:
                voc_play(16000, (uint8_t *)voice_16k_8bits, voice_16k_8bitsLen);
                p_sys->led_blink_duration[LED_RED] = 10;
                p_sys->led_blink_period[LED_RED] = 15;
                p_sys->led_blink_cnt[LED_RED] = 0;
                p_sys->led_blink_period[LED_GREEN] = 0; /* turn off gps led */
                break;

			case HI_OUT_EBD_ALERT:
                voc_play(16000, (uint8_t *)voice_16k_8bits, voice_16k_8bitsLen);
                p_sys->led_blink_duration[LED_RED] = 10;
                p_sys->led_blink_period[LED_RED] = 15;
                p_sys->led_blink_cnt[LED_RED] = 0;
                p_sys->led_blink_period[LED_GREEN] = 0; /* turn off gps led */
                break;	

            case HI_OUT_CANCEL_ALERT:
                p_sys->led_blink_duration[LED_RED] = 0;
                p_sys->led_blink_period[LED_RED] = 0;
                p_sys->led_blink_cnt[LED_RED] = 0;

                /* recover gps led */
                if (p_cms_envar->vsa.gps_status){
                    p_sys->led_blink_period[LED_GREEN] = 0xFFFF; 
                }
                else{
                    p_sys->led_blink_period[LED_GREEN] = 20;
                }
                break;

            case HI_OUT_GPS_LOST:
                voc_play(16000, (uint8_t *)notice_16k_8bits, 6400);
                p_sys->led_blink_duration[LED_GREEN] = 0xFFFF;
                p_sys->led_blink_period[LED_GREEN] = 20;
                p_sys->led_blink_cnt[LED_GREEN] = 0;
                break;

            case HI_OUT_GPS_CAPTURED:
                p_sys->led_blink_duration[LED_GREEN] = 0xFFFF;
                p_sys->led_blink_period[LED_GREEN] = 0xFFFF;
                p_sys->led_blink_cnt[LED_GREEN] = 0;
                break;

            default:
                break;
        }
    }
    else if (p_msg->id == SYS_MSG_HI_IN_UPDATE){

    }
}

void rt_hi_thread_entry(void *parameter)
{
    rt_err_t err;
    sys_msg_t msg, *p_msg = &msg;
    sys_envar_t *p_sys = (sys_envar_t *)parameter;
    int i;

    rt_kprintf("%s: ---->\n", __FUNCTION__);

    rt_timer_start(p_sys->timer_hi);

	while(1){
        err = rt_mq_recv(p_sys->queue_sys_hi, p_msg, sizeof(sys_msg_t), RT_WAITING_NO);
        if (err == RT_EOK){
            sys_human_interface_proc(p_sys, p_msg);
        }

        /* update led status */    
        for(i=0;i<LEDn;i++){
            if (p_sys->led_blink_period[i] == 0){/* always off */
                led_off(i);
            }
            else if (p_sys->led_blink_period[i] == 0xFFFF){ /* always on */
                led_on(i);
            }
            else{ /* blink periodly */
                if (++p_sys->led_blink_cnt[i] >= p_sys->led_blink_period[i]){
                    led_blink(i);
                    p_sys->led_blink_cnt[i] = 0;
                    if(p_sys->led_blink_duration[i] != 0xFFFF){
                        p_sys->led_blink_duration[i]--;
                        if(p_sys->led_blink_duration[i] <= 0){
                            p_sys->led_blink_period[i] = 0;
                        }
                    }
                }
            }
        }

        rt_thread_delay(1);
	}
}

void sys_init(void)
{
    sys_envar_t *p_sys = &p_cms_envar->sys;

    /* object for sys */
    p_sys->queue_sys_mng = rt_mq_create("q-sysm", sizeof(sys_msg_t), SYS_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
    RT_ASSERT(p_sys->queue_sys_mng != RT_NULL);

    p_sys->task_sys_mng = rt_thread_create("t-sysm",
                           rt_sys_thread_entry, p_sys,
                           RT_SYS_THREAD_STACK_SIZE, RT_SYS_THREAD_PRIORITY, 20);
    RT_ASSERT(p_sys->task_sys_mng != RT_NULL)
    rt_thread_startup(p_sys->task_sys_mng);

    /* object for human interface */
    p_sys->queue_sys_hi = rt_mq_create("q-hi", sizeof(sys_msg_t), SYS_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
    RT_ASSERT(p_sys->queue_sys_hi != RT_NULL);

    p_sys->task_sys_hi = rt_thread_create("t-hi",
                           rt_hi_thread_entry, p_sys,
                           RT_HI_THREAD_STACK_SIZE, RT_HI_THREAD_PRIORITY, 20);
    RT_ASSERT(p_sys->task_sys_hi != RT_NULL)
    rt_thread_startup(p_sys->task_sys_hi);

    p_sys->timer_hi = rt_timer_create("tm-hi",timer_human_interface_callback,p_sys,\
        HUMAN_ITERFACE_DEFAULT,RT_TIMER_FLAG_PERIODIC); 					
    RT_ASSERT(p_sys->timer_hi != RT_NULL);

    rt_kprintf("sysc module initial\n");
}


/* below are for debug */
void test_alert(void)
{
    hi_add_event_queue(&p_cms_envar->sys, SYS_MSG_HI_OUT_UPDATE,0,HI_OUT_VBD_ALERT, 0);
}
FINSH_FUNCTION_EXPORT(test_alert, debug: testing alert voice and led);



