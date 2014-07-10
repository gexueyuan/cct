/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_api.c
 @brief  : this file include the api interface of vehicle application middleware
 @author : wangyifeng
 @history:
           2014-6-15    wangyifeng    Created file
           ...
******************************************************************************/

#include <stm32f4xx.h>
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/



int32_t vam_start(void)
{
    rt_kprintf("%s: --->\n", __FUNCTION__);

    vam_add_event_queue(&p_cms_envar->vam, VAM_MSG_START, 0, 0, 0);
    
    return 0;
}
FINSH_FUNCTION_EXPORT(vam_start, vam module start);

int32_t vam_stop(void)
{
    rt_kprintf("%s: --->\n", __FUNCTION__);
    return 0;
}
FINSH_FUNCTION_EXPORT(vam_stop, vam module stop);



int32_t vam_get_config(vam_config_t *config)
{
    return 0;
}


int32_t vam_set_config(vam_config_t *config)
{
    return 0;
}

int32_t vam_set_event_handler(uint32_t evt, vam_evt_handler callback)
{
    if ((evt >= VAM_EVT_MAX)||(!callback)){
        return -1;
    }

    p_vam_envar->evt_handler[evt] = callback;

    return 0;
}

int32_t vam_get_local_status(vam_stastatus_t *local)
{
    return 0;
}



int32_t vam_set_local_status(vam_stastatus_t *local)
{
    return 0;
}

int32_t vam_get_peerlist(vam_stastatus_t **local, uint32_t maxitem, uint32_t *actual)
{
    return 0;
}


int32_t vam_get_peer_status(uint8_t *pid, vam_stastatus_t *local)
{
    return 0;
}



int32_t vam_get_peer_relative_pos(uint8_t *pid)
{
    vam_envar_t *p_vam = p_vam_envar;
    vam_sta_node_t *p_sta = NULL;
    vam_stastatus_t sta;

    rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER);

	list_for_each_entry(p_sta, vam_sta_node_t, &p_vam->neighbour_list, list){
        if (memcmp(p_sta->s.pid, pid, RCP_TEMP_ID_LEN)==0){
            memcpy(&sta, &p_sta->s, sizeof(vam_stastatus_t));
            break;
        }
	}
    rt_sem_release(p_vam->sem_sta);

    return (int32_t)vsm_get_relative_pos(&p_vam->local,&sta);
}



int32_t vam_get_peer_relative_dir(uint8_t *pid)
{
    vam_envar_t *p_vam = p_vam_envar;
    vam_sta_node_t *p_sta = NULL;
    vam_stastatus_t sta;
    int32_t delta, r;

    rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER);

	list_for_each_entry(p_sta, vam_sta_node_t, &p_vam->neighbour_list, list){
        if (memcmp(p_sta->s.pid, pid, RCP_TEMP_ID_LEN)==0){
            memcpy(&sta, &p_sta->s, sizeof(vam_stastatus_t));
            break;
        }
	}
    rt_sem_release(p_vam->sem_sta);

    delta = (int32_t)vsm_get_relative_dir(&p_vam->local,&sta);

    if ((delta <= 10)||(p_vam->local.dir == 0)||(sta.dir == 0)){
        r = 1;
    }
    else{
        r = -1;
    }

    return r;
}


int32_t vam_get_peer_relative_speed(uint8_t *pid)
{
    return 0;
}

int32_t vam_get_peer_absolute_speed(uint8_t *pid)
{
    return 0;
}



int32_t vam_active_alert(uint32_t alerttype)
{
    return 0;
}



int32_t vam_cancel_alert(uint32_t alerttype)
{
    return 0;
}




    
