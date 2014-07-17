/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_rcp.c
 @brief  : this file realize the vehicle Remote Communicat Protocol
 @author : wangyifeng
 @history:
           2014-6-17    wangyifeng    Created file
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
extern int32_t wnet_dataframe_send(rcp_txinfo_t *txinfo, 
                           uint8_t *databuf, 
                           uint32_t datalen);

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/
__COMPILE_INLINE__ uint16_t cv_ntohs(uint16_t s16)
{
	uint16_t ret;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = s16;
	#else
	s = (uint8_t *)(&s16);
	d = (uint8_t *)(&ret) + 1;
	#endif

	*d-- = *s++;
	*d-- = *s++;

	return ret;
}

__COMPILE_INLINE__ uint32_t cv_ntohl(uint32_t l32)
{
	uint32_t ret;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = l32;
	#else
	s = (uint8_t *)(&l32);
	d = (uint8_t *)(&ret) + 3;
	#endif

	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;

	return ret;
}

__COMPILE_INLINE__ float cv_ntohf(float f32)
{
	float ret;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = f32;
	#else
	s = (uint8_t *)(&f32);
	d = (uint8_t *)(&ret) + 3;
	#endif

	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;

	return ret;
}


__COMPILE_INLINE__ int32_t encode_longtitude(float x)
{
    int32_t r;
    r = (int32_t)(x*10000000.0f);
    return cv_ntohl(r);
}


__COMPILE_INLINE__ float decode_longtitude(uint32_t x)
{
    float r;
    r = ((int32_t)cv_ntohl(x)) / 10000000.0f;
    return (float)r;
}

#define encode_latitude(x) encode_longtitude(x) 
#define decode_latitude(x) decode_longtitude(x) 

#define encode_accuracy(x) encode_longtitude(x) 
#define decode_accuracy(x) decode_longtitude(x) 

__COMPILE_INLINE__ uint16_t encode_elevation(float x)
{
    return cv_ntohs((uint16_t)x);
}

__COMPILE_INLINE__ float decode_elevation(uint16_t x)
{
    return (float)cv_ntohs(x);
}

__COMPILE_INLINE__ uint16_t encode_speed(float x)
{
    uint16_t r;
    r = (uint16_t)(x*100000.0f/3600);
    return cv_ntohs(r);
}


__COMPILE_INLINE__ float decode_speed(uint16_t x)
{
    float r;
    r = cv_ntohs(x)*3600 / 100000.0f;
    return (float)r;
}

__COMPILE_INLINE__ uint16_t encode_heading(float x)
{
    double r;
    r = x*80.0f;
    return cv_ntohs((uint16_t)r);
}

__COMPILE_INLINE__ float decode_heading(uint16_t x)
{
    double r;
    r = cv_ntohs(x)/80.0;
    return (float)r;
}

__COMPILE_INLINE__ uint16_t encode_acce_lon(float x)
{
    return cv_ntohs((uint16_t)x);
}

__COMPILE_INLINE__ float decode_acce_lon(uint16_t x)
{
    return (float)cv_ntohs(x);
}

__COMPILE_INLINE__ uint16_t encode_acce_lat(float x)
{
    return cv_ntohs((uint16_t)x);
}

__COMPILE_INLINE__ float decode_acce_lat(uint16_t x)
{
    return (float)cv_ntohs(x);
}

__COMPILE_INLINE__ uint16_t encode_acce_vert(float x)
{
    return cv_ntohs((uint16_t)x);
}

__COMPILE_INLINE__ float decode_acce_vert(uint16_t x)
{
    return (float)cv_ntohs(x);
}

__COMPILE_INLINE__ uint8_t encode_acce_yaw(float x)
{
    return (uint8_t)(x);
}

__COMPILE_INLINE__ float decode_acce_yaw(uint8_t x)
{
    return (float)(x);
}



int16_t rcp_get_system_time(void)
{
    return 0;
}

int rcp_parse_bsm(vam_envar_t *p_vam,
                  rcp_rxinfo_t *rxinfo,
                  uint8_t *databuf, 
                  uint32_t datalen)
{
    vam_sta_node_t *p_sta;
    rcp_msg_basic_safty_t *p_bsm;

    if (datalen < sizeof(rcp_msg_basic_safty_t)){
        return -1;
    }

    p_bsm = (rcp_msg_basic_safty_t *)databuf;

    p_sta = vam_find_sta(p_vam, p_bsm->header.temporary_id);

    if (p_sta){
        p_sta->life = VAM_NEIGHBOUR_MAXLIFE;
        p_sta->s.timestamp = p_bsm->header.dsecond;

        p_sta->s.pos.lon = decode_longtitude(p_bsm->position.lon);
        p_sta->s.pos.lat = decode_latitude(p_bsm->position.lat);
        p_sta->s.pos.elev = decode_elevation(p_bsm->position.elev);
        p_sta->s.pos.accu = decode_accuracy(p_bsm->position.accu);

        p_sta->s.dir = decode_heading(p_bsm->motion.heading);
        p_sta->s.speed = decode_speed(p_bsm->motion.speed);
        p_sta->s.acce.lon = decode_acce_lon(p_bsm->motion.acce.lon);
        p_sta->s.acce.lat = decode_acce_lat(p_bsm->motion.acce.lat);
        p_sta->s.acce.vert = decode_acce_vert(p_bsm->motion.acce.vert);
        p_sta->s.acce.yaw = decode_acce_yaw(p_bsm->motion.acce.yaw);

        //dump_pos(&p_sta->s);

        if (p_vam->evt_handler[VAM_EVT_PEER_UPDATE]){
            (p_vam->evt_handler[VAM_EVT_PEER_UPDATE])(&p_sta->s);
        }
    }

    return 0;
}

int rcp_parse_msg(vam_envar_t *p_vam,
                  rcp_rxinfo_t *rxinfo, 
                  uint8_t *databuf, 
                  uint32_t datalen)
{
    rcp_msg_head_t *p_head;

    if (datalen < sizeof(rcp_msg_head_t)){
        return -1;
    }

    p_head = (rcp_msg_head_t *)databuf;

    switch(p_head->msg_id){
        case RCP_MSG_ID_BSM:
            rcp_parse_bsm(p_vam, rxinfo, databuf, datalen);
            break;

        case RCP_MSG_ID_EVAM:

            break;

        default:
            break;
    }

    return p_head->msg_id;
}


/*****************************************************************************
 @funcname: vam_rcp_recv
 @brief   : RCP receive data frame from network layer
 @param   : rcp_rxinfo_t *rxinfo  
 @param   : uint8_t *databuf      
 @param   : uint32_t datalen      
 @return  : 
*****************************************************************************/
int vam_rcp_recv(rcp_rxinfo_t *rxinfo, uint8_t *databuf, uint32_t datalen)
{
    vam_envar_t *p_vam = &p_cms_envar->vam;

#if 1
    vam_add_event_queue(p_vam, VAM_MSG_RCPRX, datalen, (uint32_t)databuf, rxinfo);
#else
    rcp_parse_msg(p_vam, rxinfo, databuf, datalen);
#endif
    return 0;
}







int32_t rcp_send_bsm(vam_envar_t *p_vam)
{
    rcp_msg_basic_safty_t *p_bsm = &p_vam->bsm;
    vam_stastatus_t *p_local = &p_vam->local;
    rcp_txinfo_t txbd;

    p_bsm->header.msg_id = RCP_MSG_ID_BSM;
    p_bsm->header.msg_count = p_vam->tx_msg_cnt++;
    memcpy(p_bsm->header.temporary_id, p_local->pid, RCP_TEMP_ID_LEN);
    p_bsm->header.dsecond = rcp_get_system_time();

    p_bsm->position.lon = encode_longtitude(p_local->pos.lon);
    p_bsm->position.lat = encode_latitude(p_local->pos.lat);
    p_bsm->position.elev = encode_elevation(p_local->pos.elev);
    p_bsm->position.accu = encode_accuracy(p_local->pos.accu);

    p_bsm->motion.heading = encode_heading(p_local->dir);
    p_bsm->motion.speed = encode_speed(p_local->speed);
    p_bsm->motion.acce.lon = encode_acce_lon(p_local->acce.lon);
    p_bsm->motion.acce.lat = encode_acce_lat(p_local->acce.lat);
    p_bsm->motion.acce.vert = encode_acce_vert(p_local->acce.vert);
    p_bsm->motion.acce.yaw = encode_acce_yaw(p_local->acce.yaw);

    memset(&txbd, 0, sizeof(rcp_txinfo_t));
    memset(txbd.dest, 0xFF, RCP_MACADR_LEN);
    txbd.hops = 1;

    return wnet_dataframe_send(&txbd, (uint8_t *)p_bsm, sizeof(rcp_msg_basic_safty_t));
}




//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

rcp_msg_basic_safty_t test_bsm_rx;
rcp_rxinfo_t test_rxbd;
rt_timer_t timer_test_bsm_rx;

rcp_msg_basic_safty_t test_bsm_rx_2;
rcp_rxinfo_t test_rxbd_2;
rt_timer_t timer_test_bsm_rx_2;

rcp_msg_basic_safty_t test_bsm_rx_3;
rcp_rxinfo_t test_rxbd_3;
rt_timer_t timer_test_bsm_rx_3;

void timer_test_bsm_rx_callback(void* parameter)
{
    vam_rcp_recv(&test_rxbd, (uint8_t *)&test_bsm_rx, sizeof(rcp_msg_basic_safty_t));
}

void timer_test_bsm_rx_callback_2(void* parameter)
{
    vam_rcp_recv(&test_rxbd_2, (uint8_t *)&test_bsm_rx_2, sizeof(rcp_msg_basic_safty_t));
}

void timer_test_bsm_rx_callback_3(void* parameter)
{
    vam_rcp_recv(&test_rxbd_3, (uint8_t *)&test_bsm_rx_3, sizeof(rcp_msg_basic_safty_t));
}

void test_bsm(void)
{
    rcp_msg_basic_safty_t *p_bsm = &test_bsm_rx;
    vam_stastatus_t sta;
    vam_stastatus_t *p_local = &sta;

    #if 1 
    memset(p_local, 0, sizeof(vam_stastatus_t));
    p_local->pos.lat = 40.0; //39.5427f;
    p_local->pos.lon = 120.0;//116.2317f;
    p_local->dir = 90.0;//
    #else
    memcpy(p_local, &p_cms_envar->vam.local, sizeof(vam_stastatus_t));
    #endif
    p_local->pid[0] = 0x02;
    p_local->pid[1] = 0x04;
    p_local->pid[2] = 0x06;
    p_local->pid[3] = 0x08;
    
    /* construct a fake message */
    p_bsm->header.msg_id = RCP_MSG_ID_BSM;
    p_bsm->header.msg_count = 0;
    memcpy(p_bsm->header.temporary_id, p_local->pid, RCP_TEMP_ID_LEN);
    p_bsm->header.dsecond = rcp_get_system_time();

    p_bsm->position.lon = encode_longtitude(p_local->pos.lon);
    p_bsm->position.lat = encode_latitude(p_local->pos.lat);
    p_bsm->position.elev = encode_elevation(p_local->pos.elev);
    p_bsm->position.accu = encode_accuracy(p_local->pos.accu);

    p_bsm->motion.heading = encode_heading(p_local->dir);
    p_bsm->motion.speed = encode_speed(p_local->speed);
    p_bsm->motion.acce.lon = encode_acce_lon(p_local->acce.lon);
    p_bsm->motion.acce.lat = encode_acce_lat(p_local->acce.lat);
    p_bsm->motion.acce.vert = encode_acce_vert(p_local->acce.vert);
    p_bsm->motion.acce.yaw = encode_acce_yaw(p_local->acce.yaw);

    //dump((uint8_t *)p_bsm, sizeof(rcp_msg_basic_safty_t));

    timer_test_bsm_rx = rt_timer_create("tm-tb",timer_test_bsm_rx_callback,NULL,\
        MS_TO_TICK(2400),RT_TIMER_FLAG_PERIODIC); 					

    rt_timer_start(timer_test_bsm_rx);
}


void tb2(void)
{
    rcp_msg_basic_safty_t *p_bsm = &test_bsm_rx_2;
    vam_stastatus_t sta;
    vam_stastatus_t *p_local = &sta;

    #if 1 
    memset(p_local, 0, sizeof(vam_stastatus_t));
    p_local->pos.lat = 40.0; //39.5427f;
    p_local->pos.lon = 120.1;//116.2317f;
    p_local->dir = 90.0;//
    #else
    memcpy(p_local, &p_cms_envar->vam.local, sizeof(vam_stastatus_t));
    #endif
    p_local->pid[0] = 0x01;
    p_local->pid[1] = 0x03;
    p_local->pid[2] = 0x05;
    p_local->pid[3] = 0x07;
    
    /* construct a fake message */
    p_bsm->header.msg_id = RCP_MSG_ID_BSM;
    p_bsm->header.msg_count = 0;
    memcpy(p_bsm->header.temporary_id, p_local->pid, RCP_TEMP_ID_LEN);
    p_bsm->header.dsecond = rcp_get_system_time();

    p_bsm->position.lon = encode_longtitude(p_local->pos.lon);
    p_bsm->position.lat = encode_latitude(p_local->pos.lat);
    p_bsm->position.elev = encode_elevation(p_local->pos.elev);
    p_bsm->position.accu = encode_accuracy(p_local->pos.accu);

    p_bsm->motion.heading = encode_heading(p_local->dir);
    p_bsm->motion.speed = encode_speed(p_local->speed);
    p_bsm->motion.acce.lon = encode_acce_lon(p_local->acce.lon);
    p_bsm->motion.acce.lat = encode_acce_lat(p_local->acce.lat);
    p_bsm->motion.acce.vert = encode_acce_vert(p_local->acce.vert);
    p_bsm->motion.acce.yaw = encode_acce_yaw(p_local->acce.yaw);

    //dump((uint8_t *)p_bsm, sizeof(rcp_msg_basic_safty_t));

    timer_test_bsm_rx_2 = rt_timer_create("tm-tb",timer_test_bsm_rx_callback_2,NULL,\
        MS_TO_TICK(2400),RT_TIMER_FLAG_PERIODIC); 					

    rt_timer_start(timer_test_bsm_rx_2);
}

void tb3(void)
{
    rcp_msg_basic_safty_t *p_bsm = &test_bsm_rx_3;
    vam_stastatus_t sta;
    vam_stastatus_t *p_local = &sta;

    #if 1 
    memset(p_local, 0, sizeof(vam_stastatus_t));
    p_local->pos.lat = 40.0; //39.5427f;
    p_local->pos.lon = 120.2;//116.2317f;
    p_local->dir = 90.0;//
    #else
    memcpy(p_local, &p_cms_envar->vam.local, sizeof(vam_stastatus_t));
    #endif
    p_local->pid[0] = 0x01;
    p_local->pid[1] = 0x02;
    p_local->pid[2] = 0x03;
    p_local->pid[3] = 0x04;
    
    /* construct a fake message */
    p_bsm->header.msg_id = RCP_MSG_ID_BSM;
    p_bsm->header.msg_count = 0;
    memcpy(p_bsm->header.temporary_id, p_local->pid, RCP_TEMP_ID_LEN);
    p_bsm->header.dsecond = rcp_get_system_time();

    p_bsm->position.lon = encode_longtitude(p_local->pos.lon);
    p_bsm->position.lat = encode_latitude(p_local->pos.lat);
    p_bsm->position.elev = encode_elevation(p_local->pos.elev);
    p_bsm->position.accu = encode_accuracy(p_local->pos.accu);

    p_bsm->motion.heading = encode_heading(p_local->dir);
    p_bsm->motion.speed = encode_speed(p_local->speed);
    p_bsm->motion.acce.lon = encode_acce_lon(p_local->acce.lon);
    p_bsm->motion.acce.lat = encode_acce_lat(p_local->acce.lat);
    p_bsm->motion.acce.vert = encode_acce_vert(p_local->acce.vert);
    p_bsm->motion.acce.yaw = encode_acce_yaw(p_local->acce.yaw);

    //dump((uint8_t *)p_bsm, sizeof(rcp_msg_basic_safty_t));

    timer_test_bsm_rx_3 = rt_timer_create("tm-tb",timer_test_bsm_rx_callback_3,NULL,\
        MS_TO_TICK(2400),RT_TIMER_FLAG_PERIODIC); 					

    rt_timer_start(timer_test_bsm_rx_3);
}


void stop_test_bsm(void)
{
	rt_timer_stop(timer_test_bsm_rx);
}
void stop_test_bsm_2(void)
{
	rt_timer_stop(timer_test_bsm_rx_2);
}


FINSH_FUNCTION_EXPORT(test_bsm, debug: testing when bsm is received);

FINSH_FUNCTION_EXPORT(stop_test_bsm, debug: testing when bsm stop);

FINSH_FUNCTION_EXPORT(tb2, debug: testing when bsm is received);

FINSH_FUNCTION_EXPORT(stop_test_bsm_2, debug: testing when bsm stop);

FINSH_FUNCTION_EXPORT(tb3, debug: testing when bsm is received);

#if 0
void test_data(void)
{
    float f1 = 5.22222, f2=-3.82222;
    int32_t i1, i2;
    uint32_t u1, u2;
    uint8_t buf[64] = {0};

    sprintf(buf, "f1=%f, f2=%f \n", f1, f2);
    rt_kprintf("%s", buf);  

    i1 = (int32_t)f1;
    i2 = (int32_t)f2;
    rt_kprintf("i1=%d, i2=%d\n", i1,i2);

    u1 = (uint32_t)i1;
    u2 = (uint32_t)i2;
    rt_kprintf("u1=%d, u2=%d\n", u1,u2);
}

FINSH_FUNCTION_EXPORT(test_data, debug: testing datatype);
#endif

