/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_vsm.c
 @brief  : this file realize the function of vehicle status management
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
#include "cv_cms_def.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
//rt_timer_t timer_send_bsm;
//vam_envar_t *p_vam;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

static uint16_t _cal_peroid_from_speed(uint32_t speed)
{
    return 1000;
}

void vsm_start_bsm_broadcast(vam_envar_t *p_vam)
{
    uint16_t period;
    
    if(p_vam->working_param.bsm_boardcast_mode == BSM_BC_MODE_AUTO){
        //calcute peroid from speed

        period = _cal_peroid_from_speed(0);
    }
    else if (p_vam->working_param.bsm_boardcast_mode == BSM_BC_MODE_FIXED){
        period = p_vam->working_param.bsm_boardcast_period;
    }

    p_vam->bsm_send_period_ticks = MS_TO_TICK(period);

    rt_timer_control(p_vam->timer_send_bsm, RT_TIMER_CTRL_SET_TIME, &p_vam->bsm_send_period_ticks);
    rt_timer_start(p_vam->timer_send_bsm);
}

void vsm_stop_bsm_broadcast(vam_envar_t *p_vam)
{
    rt_timer_stop(p_vam->timer_send_bsm);
}

void timer_send_bsm_callback(void* parameter)
{
    vam_envar_t *p_vam = (vam_envar_t *)parameter;

    if ((p_vam->flag&VAM_FLAG_TX_BSM)&&(!(p_vam->flag&VAM_FLAG_TX_BSM_PAUSE))){
        vam_add_event_queue(p_vam, VAM_MSG_RCPTX, 0, RCP_MSG_ID_BSM, NULL);
    }
}

void timer_bsm_pause_callback(void* parameter)
{
    vam_envar_t *p_vam = (vam_envar_t *)parameter;

    if (p_vam->flag & VAM_FLAG_TX_BSM){
        p_vam->flag &= ~VAM_FLAG_TX_BSM_PAUSE;
    }
}

void timer_gps_life_callback(void* parameter)
{
    vam_envar_t *p_vam = (vam_envar_t *)parameter;

    if (p_vam->flag & VAM_FLAG_GPS_FIXED){
        p_vam->flag &= ~VAM_FLAG_GPS_FIXED;
        rt_kprintf("gps is lost!\n");
        if (p_vam->evt_handler[VAM_EVT_GPS_STATUS]){
            (p_vam->evt_handler[VAM_EVT_GPS_STATUS])((void *)0);
        }
    }
}

vam_sta_node_t *vam_find_sta(vam_envar_t *p_vam, uint8_t *temporary_id)
{
    vam_sta_node_t *p_sta = NULL, *pos;

	list_for_each_entry(pos, vam_sta_node_t, &p_vam->neighbour_list, list){
        if (memcmp(pos->s.pid, temporary_id, RCP_TEMP_ID_LEN) == 0){
            p_sta = pos;
            break;
        }
	}

    /* not found, allocate new one */
    if (p_sta == NULL){

        rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER);
    	if (!list_empty(&p_vam->sta_free_list)) {
    		p_sta = list_first_entry(&p_vam->sta_free_list, vam_sta_node_t, list);
    		list_move(&p_sta->list, &p_vam->neighbour_list);
    	}
        rt_sem_release(p_vam->sem_sta);

        if (p_sta){
            rt_kprintf("one neighbour join\n");
            memcpy(p_sta->s.pid, temporary_id, RCP_TEMP_ID_LEN);        
        }
        else{
            rt_kprintf("%s: no free sta.\n", __FUNCTION__);
        }
    }

    return p_sta;
}


void vam_update_sta(vam_envar_t *p_vam)
{
    vam_sta_node_t *p_sta = NULL;
    list_head_t *pos;
    list_head_t *head = &p_vam->neighbour_list;

//    rt_kprintf("%s--->\n", __FUNCTION__);

    if (rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER) != RT_EOK){
        rt_kprintf("%s: semaphor return failed\n");
        return;
    }

    for (pos = head->next; pos != (head); ){
        /* must prefatch the next pointer */
        p_sta = (vam_sta_node_t *)pos;
        pos = pos->next;

        p_sta->life--;
        if (p_sta->life <= 0){
            rt_kprintf("one neighbour is kick out\n");
            list_move_tail(&p_sta->list, &p_vam->sta_free_list);
        }
    }
    
    rt_sem_release(p_vam->sem_sta);
}

void timer_neigh_time_callback(void* parameter)
{
    vam_envar_t *p_vam = (vam_envar_t *)parameter;

    vam_add_event_queue(p_vam, VAM_MSG_NEIGH_TIMEOUT, 0, 0, NULL);
}


void vam_list_sta(void)
{
    vam_envar_t *p_vam = &p_cms_envar->vam;
    vam_sta_node_t *p_sta = NULL;
    int i = 0;

    if (rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER) != RT_EOK){
        rt_kprintf("%s: semaphor return failed\n");
        return;
    }

	list_for_each_entry(p_sta, vam_sta_node_t, &p_vam->sta_free_list, list){
        i++;
	}

    rt_kprintf("free sta node:%d\n", i);
    rt_kprintf("neighbor node:%d\n", VAM_NEIGHBOUR_MAXNUM - i);

	list_for_each_entry(p_sta, vam_sta_node_t, &p_vam->neighbour_list, list){
        rt_kprintf("STA:[%02x-%02x-%02x-%02x], life:%d\n",\
            p_sta->s.pid[0],p_sta->s.pid[1],p_sta->s.pid[2],p_sta->s.pid[3],\
            p_sta->life);
	}
    rt_sem_release(p_vam->sem_sta);
}
FINSH_FUNCTION_EXPORT(vam_list_sta, list all neighbour sta);


