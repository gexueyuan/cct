/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_cms_def.h
 @brief  : This file include the cms global definition 
 @author : wangyifeng
 @history:
           2014-6-16    wangyifeng    Created file
           ...
******************************************************************************/
#ifndef __CV_CMS_DEF_H__
#define __CV_CMS_DEF_H__

#include "cv_vam.h"
#include "cv_vsa.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

/**
    prority of all the tasks in system 
*/
#define RT_INIT_THREAD_PRIORITY		10   /* highest, to ensure all initial 
                                            will be complete before running*/
#define RT_SYS_THREAD_PRIORITY		14
#define RT_VAM_THREAD_PRIORITY		23
#define RT_VSA_THREAD_PRIORITY		22

#define RT_GPS_THREAD_PRIORITY		21
#define RT_MEMS_THREAD_PRIORITY		22
#define RT_HI_THREAD_PRIORITY		25


#define RT_INIT_THREAD_STACK_SIZE   (1024*2)
#define RT_SYS_THREAD_STACK_SIZE   (1024*2)
#define RT_GPS_THREAD_STACK_SIZE   (1024*2)
#define RT_MEMS_THREAD_STACK_SIZE   (1024*2)
#define RT_VAM_THREAD_STACK_SIZE   (1024*2)
#define RT_VSA_THREAD_STACK_SIZE   (1024*2)
#define RT_HI_THREAD_STACK_SIZE   (1024*2)


/**
    size of all the queue in system 
*/
#define SYS_QUEUE_SIZE 16
#define VAM_QUEUE_SIZE 16
#define VSA_QUEUE_SIZE 16


enum SYSTEM_MSG_TYPE{
    SYS_MSG_BASE = 0x0000,
    SYS_MSG_INITED,
    SYS_MSG_START_ALERT,
    SYS_MSG_STOP_ALERT,
    SYS_MSG_GPS_UPDATE,
    SYS_MSG_HI_IN_UPDATE,
    SYS_MSG_HI_OUT_UPDATE,
    SYS_MSG_XXX,

    VAM_MSG_BASE = 0x0200,
    VAM_MSG_START,
    VAM_MSG_STOP,
    VAM_MSG_RCPTX,
    VAM_MSG_RCPRX,
    VAM_MSG_NEIGH_TIMEOUT,
    VAM_MSG_GPSDATA,


    VSA_MSG_BASE = 0x0300,
    VSA_MSG_LOCAL_UPDATE,
    VSA_MSG_PEER_UPDATE,
    VSA_MSG_GPS_UPDATE,
    VSA_MSG_ALARM_UPDATE,
};

enum HI_OUT_TYPE{
    HI_OUT_NONE = 0,
    HI_OUT_GPS_CAPTURED,
    HI_OUT_GPS_LOST,
    HI_OUT_CRD_ALERT,
    HI_OUT_VBD_ALERT,
    HI_OUT_EBD_ALERT,
    HI_OUT_CANCEL_ALERT,
};


/**
    misc definitions 
*/
#define MS_TO_TICK(n)     ((n)*RT_TICK_PER_SECOND/1000)
#define SECOND_TO_TICK(n) ((n)*RT_TICK_PER_SECOND)


/*****************************************************************************
 * declaration of structs                                                    *
*****************************************************************************/

/**
    structure of system global message 
*/
typedef struct _sys_msg{
    uint16_t id;
    uint16_t len;
    uint32_t argc; 
    void    *argv;
}sys_msg_t;

/**
    structure of system configure parameters 
*/
typedef struct _cfg_param{

	/*********************ID******************/	
    uint8_t pid[RCP_TEMP_ID_LEN];  // ID 

    /******************** VAM *********************/
    vam_config_t vam;

    /******************** VSA *********************/
    vsa_config_t vsa;

    /******************** DBG *********************/
    uint8_t print_xxx;  /* 0 - disable, 1 - enable */

                 
}cfg_param_t;


/** 
    structure of system manager module's environment variable 
*/
typedef struct _sys_envar{
    /* working_param */
    vsa_config_t working_param;

    uint32_t status;

    uint32_t hi_timer_cnt;

    uint16_t led_blink_duration[3];
    uint16_t led_blink_period[3];
    uint16_t led_blink_cnt[3];

    /* os related */
    rt_thread_t task_sys_mng;
    rt_mq_t queue_sys_mng;

    rt_thread_t task_sys_hi;
    rt_mq_t queue_sys_hi;

    rt_timer_t timer_hi;
    
}sys_envar_t;


/**
    structure of system global environment variable 
*/
typedef struct _cms_global{
    vam_envar_t vam;
    vsa_envar_t vsa;

    sys_envar_t sys;
}cms_global_t;



/*****************************************************************************
 * declare of global functions and variables                                 *
*****************************************************************************/
extern cms_global_t cms_envar, *p_cms_envar;
extern cfg_param_t cms_param, *p_cms_param;

rt_err_t sys_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv);

extern double vsm_get_relative_pos_immediate(vam_stastatus_t *p_src, uint8_t *payload);
#endif /* __CV_CMS_DEF_H__ */

