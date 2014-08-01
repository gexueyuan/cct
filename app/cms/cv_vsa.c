/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vsa.c
 @brief  : this file realize the function of vehicle safty application
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
#include "cv_vsa.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
#define VSA_TIMER_PERIOD         SECOND_TO_TICK(1)


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

void vsa_local_status_update(void* parameter)
{
    vam_stastatus_t *p_sta = (vam_stastatus_t *)parameter;
    vsa_envar_t *p_vsa = &p_cms_envar->vsa;

    memcpy(&p_vsa->local, p_sta, sizeof(vam_stastatus_t));
    vsa_add_event_queue(p_vsa, VSA_MSG_LOCAL_UPDATE, 0,0,NULL);
}

void vsa_peer_status_update(void *parameter)
{
    vam_stastatus_t *p_sta = (vam_stastatus_t *)parameter;
    vsa_envar_t *p_vsa = &p_cms_envar->vsa;

    memcpy(&p_vsa->remote, p_sta, sizeof(vam_stastatus_t));
    vsa_add_event_queue(p_vsa, VSA_MSG_PEER_UPDATE, 0,0,NULL);
}

void vsa_gps_status_update(void *parameter)
{
    vsa_envar_t *p_vsa = &p_cms_envar->vsa;
    vsa_add_event_queue(p_vsa, VSA_MSG_GPS_UPDATE, 0,(uint32_t)parameter,NULL);
}

void vsa_start(void)
{
    vam_set_event_handler(VAM_EVT_LOCAL_UPDATE, vsa_local_status_update);
    vam_set_event_handler(VAM_EVT_PEER_UPDATE, vsa_peer_status_update);
    vam_set_event_handler(VAM_EVT_GPS_STATUS, vsa_gps_status_update);
}

/*****************************************************************************
 @funcname: crd_judge
 @brief   : check and judge the close range danger of vehicles
 @param   : vsa_envar_t *p_vsa  
 @return  : 
            0 - no alert
            1 - alert
*****************************************************************************/
static int crd_judge(vsa_envar_t *p_vsa)
{
    int32_t dis_actual, dis_alert;

    /* put the beginning only in order to output debug infomations */
    dis_actual = vam_get_peer_relative_pos(p_vsa->remote.pid);
    dis_alert = (int32_t)((p_vsa->local.speed*2.0f - p_vsa->remote.speed)*p_vsa->working_param.crd_saftyfactor*1000.0f/3600.0f);
    /* end */

    if (p_vsa->local.speed <= p_vsa->working_param.danger_detect_speed_threshold){
        return 0;
    }

    if (p_vsa->local.speed <= p_vsa->remote.speed){
        return 0;
    }

    if (vam_get_peer_relative_dir(p_vsa->remote.pid) < 0){
        return 0;
    }

    /* remote is behind of local */
    if (dis_actual <= 0){
        return 0;
    }

    if (dis_actual > dis_alert){
        return 0;
    }

	//rt_kprintf("Close range danger alert(safty:%d, actual:%d)!!!\n", dis_alert, dis_actual);

	rt_kprintf("Close range danger alert(safty:%d, actual:%d)!!! Id:%d%d%d%d\n", dis_alert, dis_actual,p_vsa->remote.pid[0],p_vsa->remote.pid[1],p_vsa->remote.pid[2],p_vsa->remote.pid[3]);

    return 1;
}

static int crd_proc(vsa_envar_t *p_vsa, void *arg)
{
    int err = 1;  /* '1' represent is not handled. */
    sys_msg_t *p_msg = (sys_msg_t *)arg;	
    vam_envar_t *p_vam = &p_cms_envar->vam;
	vsa_crd_node_t *p_crd = NULL,*pos = NULL;
	rt_bool_t  crd_flag;
    
    switch(p_msg->id){
        case VSA_MSG_LOCAL_UPDATE:
            err = 0;
            break;
            
        case VSA_MSG_PEER_UPDATE:

            if (p_vsa->alert_mask & (1<<VSA_ID_CRD)){
                if (crd_judge(p_vsa) > 0){
					
					rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER);

					if(list_empty(&p_vam->crd_list))
						{
							p_crd = (vsa_crd_node_t*)rt_malloc(sizeof(vsa_crd_node_t));
							memcpy(p_crd->pid,p_vsa->remote.pid,RCP_TEMP_ID_LEN);			
							list_add(&p_crd->list,&p_vam->crd_list);
						}
					else
					  list_for_each_entry(pos,vsa_crd_node_t,&p_vam->crd_list,list)
							{
								if(memcmp(p_vsa->remote.pid,pos->pid,RCP_TEMP_ID_LEN) == 0)
									{
										crd_flag = RT_FALSE;
										break;
									}	
								else
									crd_flag = RT_TRUE;
							}
					if(crd_flag )
						{
							p_crd = (vsa_crd_node_t*)rt_malloc(sizeof(vsa_crd_node_t));
							memcpy(p_crd->pid,p_vsa->remote.pid,RCP_TEMP_ID_LEN);			
							list_add(&p_crd->list,&p_vam->crd_list);
						}	
					rt_sem_release(p_vam->sem_sta);					
                /* danger is detected */    
                    if (p_vsa->alert_pend & (1<<VSA_ID_CRD)){
                    /* do nothing */    
                    }
                    else{
                    /* inform system to start alert */
                        p_vsa->alert_pend |= (1<<VSA_ID_CRD);
                        sys_add_event_queue(&p_cms_envar->sys, \
                                            SYS_MSG_START_ALERT, 0, VSA_ID_CRD, NULL);
                    }
                }
                else{
					
					rt_sem_take(p_vam->sem_sta, RT_WAITING_FOREVER);
					
					  list_for_each_entry(pos,vsa_crd_node_t,&p_vam->crd_list,list)					
							{
								if(memcmp(p_vsa->remote.pid,pos->pid,RCP_TEMP_ID_LEN) == 0)
									{
								   		list_del(&pos->list);
										rt_free((vsa_crd_node_t*)list_entry(&pos->list,vsa_crd_node_t,list));
									}	
									
					  		}	
					  
					 rt_sem_release(p_vam->sem_sta);				  
						
                    if ((p_vsa->alert_pend & (1<<VSA_ID_CRD))&&(list_empty(&p_vam->crd_list))){
                    /* inform system to stop alert */
                        p_vsa->alert_pend &= ~(1<<VSA_ID_CRD);
                        sys_add_event_queue(&p_cms_envar->sys, \
                                            SYS_MSG_STOP_ALERT, 0, VSA_ID_CRD, NULL);
                    }
                }
            }

            err = 0;
   
            break;
            
        default:
            break;
    }

    return err;
}


static int ebd_judge(vsa_envar_t *p_vsa)
{

	return 1;

}
static int ebd_proc(vsa_envar_t *p_vsa, void *arg)
{
    return 1;
}

static int vbd_proc(vsa_envar_t *p_vsa, void *arg)
{
    return 1;
}

static void vsa_default_proc(vsa_envar_t *p_vsa, void *arg)
{
    sys_msg_t *p_msg = (sys_msg_t *)arg;
    
    switch(p_msg->id){
        case VSA_MSG_GPS_UPDATE:
            p_vsa->gps_status = p_msg->argc;
            sys_add_event_queue(&p_cms_envar->sys, \
                            SYS_MSG_GPS_UPDATE, 0, p_msg->argc, NULL);
        break;

        default:
        break;
    }
}

vsa_app_handler vsa_app_handler_tbl[] = {
    crd_proc,
    vbd_proc,
    ebd_proc,
    NULL
};

void rt_vsa_thread_entry(void *parameter)
{
    rt_err_t err;
    sys_msg_t msg, *p_msg = &msg;
    vsa_envar_t *p_vsa = (vsa_envar_t *)parameter;
    vsa_app_handler *handler;

    rt_kprintf("%s: ---->\n", __FUNCTION__);

	while(1){
        err = rt_mq_recv(p_vsa->queue_vsa, p_msg, sizeof(sys_msg_t), RT_WAITING_FOREVER);
        if (err == RT_EOK){
            for(handler = &vsa_app_handler_tbl[0];*handler != NULL;handler++){
                err = (*handler)(p_vsa, p_msg);
            }

            if (err == 1){
                vsa_default_proc(p_vsa, p_msg);
            }
        }
        else{
            rt_kprintf("%s: rt_mq_recv error [%d]\n", __FUNCTION__, err);
        }
	}
}

rt_err_t vsa_add_event_queue(vsa_envar_t *p_vsa, 
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
    err = rt_mq_send(p_vsa->queue_vsa, &msg, sizeof(sys_msg_t));

    if (err != RT_EOK){
        rt_kprintf("%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_id);
    }

    return err;
}


/*****************************************************************************
 @funcname: vsa_init
 @brief   : vsa module initial
 @param   : None
 @return  : 
*****************************************************************************/
void vsa_init()
{
    vsa_envar_t *p_vsa = &p_cms_envar->vsa;

    memset(p_vsa, 0, sizeof(vsa_envar_t));
    memcpy(&p_vsa->working_param, &p_cms_param->vsa, sizeof(vsa_config_t));

    p_vsa->alert_mask = (1<<VSA_ID_CRD)|(1<<VSA_ID_VBD)|(1<<VSA_ID_EBD);

    /* os object for vsa */
    p_vsa->queue_vsa = rt_mq_create("q-vsa", sizeof(sys_msg_t), VSA_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
    RT_ASSERT(p_vsa->queue_vsa != RT_NULL);
    
	 p_vsa->task_vsa = rt_thread_create("t-vsa",
                           rt_vsa_thread_entry, p_vsa,
                           RT_VSA_THREAD_STACK_SIZE, RT_VSA_THREAD_PRIORITY, 20);
    RT_ASSERT(p_vsa->task_vsa != RT_NULL)
    rt_thread_startup(p_vsa->task_vsa);

	rt_kprintf("vsa module initial\n");
}

/*****************************************************************************
 @funcname: vsa_deinit
 @brief   : vsa module unstall
 @param   : None
 @return  : 
*****************************************************************************/
void vsa_deinit()
{
	rt_kprintf("vsa module initial\n");
}

