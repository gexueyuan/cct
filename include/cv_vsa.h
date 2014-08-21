/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vsa.h
 @brief  : this file include the definition of vsa module
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           ...
******************************************************************************/
#ifndef __CV_VSA_H__
#define __CV_VSA_H__

/*****************************************************************************
 * definition of micro                                                       *
*****************************************************************************/
enum VSA_APP_ID{
    VSA_ID_CRD = 0,
    VSA_ID_VBD,    
    VSA_ID_EBD,
    VSM_ID_END
};


/*****************************************************************************
 * definition of struct                                                      *
*****************************************************************************/

typedef struct _vsa_config{
    /*
        General
    */
    

    uint8_t danger_detect_speed_threshold;  /* unit: km/h */
    uint16_t danger_alert_period;  /* 50~1000, unit:ms, min accuracy :10ms */
    /*
        Close Range Danger function:
    */
    uint8_t crd_saftyfactor;  /* 1~10 */

    /*
        Emergency Braking Danger function:
    */
    uint8_t ebd_mode;  /* 0 - disable, 1 - enable */
    uint8_t ebd_acceleration_threshold; /* unit:m/s2 */
    uint8_t ebd_alert_hold_time;  /* unit:s */
	
}vsa_config_t;


typedef struct _vsa_envar{
    /* working_param */
    vsa_config_t working_param;

    uint32_t gps_status;

    uint32_t alert_mask;
    uint32_t alert_pend;

    vam_stastatus_t local;
    vam_stastatus_t remote;

	/*List head*/	
    list_head_t crd_list;	
    list_head_t ebd_list;
    list_head_t vbd_list;

    /* os related */
    rt_thread_t task_vsa;
    rt_mq_t queue_vsa;

    rt_timer_t timer_ebd_send;
                 
}vsa_envar_t;

typedef struct _vsa_crd_node{
    /* !!!DON'T modify it!!! */
    list_head_t list;

    uint8_t pid[RCP_TEMP_ID_LEN];  //temporary ID

    /* private */
    uint16_t life;
    

}vsa_crd_node_t;


typedef int (*vsa_app_handler)(vsa_envar_t *p_vsa, void *p_msg);


/*****************************************************************************
 * declaration of global variables and functions                             *
*****************************************************************************/
void vsa_start(void);

rt_err_t vsa_add_event_queue(vsa_envar_t *p_vsa, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv);


#endif /* __CV_VSA_H__ */

