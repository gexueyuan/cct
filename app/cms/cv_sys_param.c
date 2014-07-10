/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_param.c
 @brief  : this file include the system parameter management
 @author : wangyifeng
 @history:
           2014-6-19    wangyifeng    Created file
           ...
******************************************************************************/
#include <stdio.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_vsa.h"
#include "cv_cms_def.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
cfg_param_t cms_param, *p_cms_param;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

void load_default_param(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 1;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */

    param->vam.evam_hops = 3; 



    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 30;  /* unit: km/h */
    param->vsa.danger_alert_period = 50;  /* 50~1000, unit:ms, min accuracy :10ms */
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

}



void param_init(void)
{
    p_cms_param = &cms_param;
    load_default_param(p_cms_param);
}


void param_get(void)
{
    rt_kprintf("-------------------parameters------------------\n");
    rt_kprintf("vam.bsm_hops=%d\n", p_cms_param->vam.bsm_hops);
    rt_kprintf("vam.bsm_boardcast_mode=%d\n", p_cms_param->vam.bsm_boardcast_mode);
    rt_kprintf("vam.bsm_boardcast_saftyfactor=%d\n", p_cms_param->vam.bsm_boardcast_saftyfactor);
    rt_kprintf("vam.bsm_boardcast_period=%d (ms)\n", p_cms_param->vam.bsm_boardcast_period);
    rt_kprintf("vam.bsm_pause_mode=%d\n", p_cms_param->vam.bsm_pause_mode);
    rt_kprintf("vam.bsm_pause_hold_time=%d (s)\n", p_cms_param->vam.bsm_pause_hold_time);
    rt_kprintf("vam.evam_hops=%d\n", p_cms_param->vam.evam_hops);

    rt_kprintf("vsa.danger_detect_speed_threshold=%d (m/s)\n", p_cms_param->vsa.danger_detect_speed_threshold);
    rt_kprintf("vsa.danger_alert_period=%d (ms)\n", p_cms_param->vsa.danger_alert_period);
    rt_kprintf("vsa.crd_saftyfactor=%d\n", p_cms_param->vsa.crd_saftyfactor);
    rt_kprintf("vsa.ebd_mode=%d\n", p_cms_param->vsa.ebd_mode);
    rt_kprintf("vsa.ebd_acceleration_threshold=%d (m/s2)\n", p_cms_param->vsa.ebd_acceleration_threshold);
    rt_kprintf("vsa.ebd_alert_hold_time=%d (s)\n", p_cms_param->vsa.ebd_alert_hold_time);

    rt_kprintf("...\n");

    rt_kprintf("----------------------end---------------------\n");
}
FINSH_FUNCTION_EXPORT(param_get, get system parameters);

void param_set(uint8_t *param, uint8_t *value)
{


}
FINSH_FUNCTION_EXPORT(param_set, set system parameters);












