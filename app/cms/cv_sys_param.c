/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_param.c
 @brief  : this file include the system parameter management
 @author : wangyifeng
 @history:
           2014-6-19    wangyifeng    Created file
           2014-7-28    gexueyuan     modified
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


#define PARAM_FLAG_ADDR     ((uint32_t)0x80E0000)

#define PARAM_ADDR    		((uint32_t)0x80E0010)


extern 	int drv_fls_erase(uint32_t	sector);
extern  int drv_fls_read(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);
extern	int drv_fls_write(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

cfg_param_t cms_param, *p_cms_param;
const char* env_var[] = {"id","vam.bh","vam.bbm","vam.bbs","vam.bpm","vam.bpht","vam.bbp","vam.eh","vam.ebt","vam.ebp",\
	                  	 "vsa.ddst","vsa.dap","vsa.cs","vsa.em","vsa.eat","vsa.eaht","print"};

uint8_t	param_init_words[] = "Vanet-param";
/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

int8_t  get_param_pos(const char* param)
{
	
  int8_t pos;

  for(pos = 0;pos<(sizeof(env_var)/sizeof(*env_var));pos++)
  	{
		if(strcmp(param,env_var[pos]) == 0)
			return pos;

  	}
  
  pos = -1;
  
  return pos;	
}
void load_default_param(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

	/******************ID************************/
	param->pid[0] = 0x00;
	param->pid[1] = 0x00;	
	param->pid[2] = 0x00;
	param->pid[3] = 0x00;
    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 1;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
	
    param->vam.evam_hops = 3; 
	param->vam.evam_broadcast_type = 2;
	param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 30;  /* unit: km/h */
    param->vsa.danger_alert_period = 50;  /* 50~1000, unit:ms, min accuracy :10ms */
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

}

void load_param_from_fl(void)
{
	p_cms_param = &cms_param;
	
	drv_fls_read(PARAM_ADDR,(uint8_t *)p_cms_param,sizeof(cfg_param_t));
	

}

void  write_def_param(void)
{
	cfg_param_t  flash_param;

	rt_err_t err;

	load_default_param(&flash_param);
	
	drv_fls_erase(FLASH_Sector_11);
	drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
	err = drv_fls_write(PARAM_ADDR,(uint8_t *)&flash_param,sizeof(cfg_param_t));

	if(-1 == err)
		rt_kprintf("error happened when writing default param to flash");
	else
		rt_kprintf("write default param to flash  success");


}

FINSH_FUNCTION_EXPORT(write_def_param, debug:write default  param to flash);


void param_init(void)
{
		uint8_t magic_word[sizeof(param_init_words)];
	
		drv_fls_read(PARAM_FLAG_ADDR,magic_word,sizeof(param_init_words));
	
		if(strcmp((const char*)param_init_words,(const char*)magic_word) != 0)
			{
				p_cms_param = &cms_param;
				load_default_param(p_cms_param);			
				write_def_param();
			}
		else load_param_from_fl();
	
}



void param_get(void)
{
	//cfg_param_t  *param_temp;

	//drv_fls_read(PARAM_ADDR,(uint8_t *)param_temp,sizeof(cfg_param_t));
		
    rt_kprintf("-------------------parameters in ram------------------\n");	
	rt_kprintf("ID(id)=%d%d%d%d\n",p_cms_param->pid[0],p_cms_param->pid[1],p_cms_param->pid[2],p_cms_param->pid[3]);
    rt_kprintf("vam.bsm_hops(vam.bh)=%d\n", p_cms_param->vam.bsm_hops);
    rt_kprintf("vam.bsm_boardcast_mode(vam.bbm)=%d\n", p_cms_param->vam.bsm_boardcast_mode);
    rt_kprintf("vam.bsm_boardcast_saftyfactor(vam.bbs)=%d\n", p_cms_param->vam.bsm_boardcast_saftyfactor);
    rt_kprintf("vam.bsm_pause_mode(vam.bpm)=%d\n", p_cms_param->vam.bsm_pause_mode);
    rt_kprintf("vam.bsm_pause_hold_time(vam.bpht)=%d (s)\n", p_cms_param->vam.bsm_pause_hold_time);
    rt_kprintf("vam.bsm_boardcast_period(vam.bbp)=%d (ms)\n", p_cms_param->vam.bsm_boardcast_period);

    rt_kprintf("vam.evam_hops(vam.eh)=%d\n", p_cms_param->vam.evam_hops);
    rt_kprintf("vam.evam_broadcast_type(vam.ebt)=%d\n", p_cms_param->vam.evam_broadcast_type);
    rt_kprintf("vam.evam_broadcast_peroid(vam.ebp)=%d\n\n", p_cms_param->vam.evam_broadcast_peroid);

    rt_kprintf("vsa.danger_detect_speed_threshold(vsa.ddst)=%d (m/s)\n", p_cms_param->vsa.danger_detect_speed_threshold);
    rt_kprintf("vsa.danger_alert_period(vsa.dap)=%d (ms)\n", p_cms_param->vsa.danger_alert_period);
	rt_kprintf("vsa.crd_saftyfactor(vsa.cs)=%d\n", p_cms_param->vsa.crd_saftyfactor);
    rt_kprintf("vsa.ebd_mode(vsa.em)=%d\n", p_cms_param->vsa.ebd_mode);
    rt_kprintf("vsa.ebd_acceleration_threshold(vsa.eat)=%d (m/s2)\n", p_cms_param->vsa.ebd_acceleration_threshold);
    rt_kprintf("vsa.ebd_alert_hold_time(vsa.eaht)=%d (s)\n", p_cms_param->vsa.ebd_alert_hold_time);

    rt_kprintf("...\n");

    rt_kprintf("----------------------end---------------------\n");
}
FINSH_FUNCTION_EXPORT(param_get, get system parameters);


void print_bn(void)
{


 rt_kprintf("vam_config_t  is %d bytes\n",sizeof(vam_config_t));

 
 rt_kprintf("vsa_config_t  is %d bytes\n",sizeof(vsa_config_t));

 
 rt_kprintf("cfg_param_t  is %d bytes\n",sizeof(cfg_param_t));

 rt_kprintf("env_var  is %d bytes\n",sizeof(env_var)/sizeof(*env_var));

 rt_kprintf("param_init_words is %d bytes\n",sizeof(param_init_words));

}
FINSH_FUNCTION_EXPORT(print_bn, data struct bytes needed);



void print_init_word(void)//print  flag of initialized
{
	
	uint8_t init_word[sizeof(param_init_words)];

	
	drv_fls_read(PARAM_FLAG_ADDR,init_word,sizeof(param_init_words));

	rt_kprintf("init word in flash is \"%s\"\n",init_word);

}
FINSH_FUNCTION_EXPORT(print_init_word, print init words  in flash);



void print_fd(uint32_t addr)//print  data of specified  address ,e.g:print_fd(0x80E0010),
{

	uint8_t data;

	drv_fls_read(addr,&data,1);

	rt_kprintf("data in address %x  is \"%d\"\n",addr,data);
}

FINSH_FUNCTION_EXPORT(print_fd, print  data of specified  address in flash);



int param_set(const char *param, uint16_t value)
{

	int8_t  pos;

	int err;

	cfg_param_t *cfg_param;

	cfg_param = (cfg_param_t*)rt_malloc(sizeof(cfg_param_t));

	drv_fls_read(PARAM_ADDR,(uint8_t*)cfg_param,sizeof(cfg_param_t));

	pos = get_param_pos(param);
/*
	if(strcmp(param,"vam.bbp")&&strcmp(param,"vsa.dap")&&strcmp(param,"vam.ebp"))
		if(value > 0xff)
			{
				//value = 0xff;
				rt_kprintf("max value is 0xff");
				return -1;
			}
			*/
	switch(pos){

		case 0:
			if(value > 9999) value = 9999;
			cfg_param->pid[0] = value/1000;
			cfg_param->pid[1] = (value%1000)/100;
			cfg_param->pid[2] = ((value%1000)%100)/10;
			cfg_param->pid[3] =	((value%1000)%100)%10;
			break;

		case 1:
			cfg_param->vam.bsm_hops = value;
			break;
		case 2:
			cfg_param->vam.bsm_boardcast_mode = value;
			break;		
		case 3:
			cfg_param->vam.bsm_boardcast_saftyfactor = value;
			break;
		case 4:
			cfg_param->vam.bsm_pause_mode = value;
			break;
		case 5:
			cfg_param->vam.bsm_pause_hold_time = value;
			break;
		case 6:
			cfg_param->vam.bsm_boardcast_period = value;
			break;
			
		case 7:
			cfg_param->vam.evam_hops = value;
			break;
		case 8:
			cfg_param->vam.evam_broadcast_type = value;
			break;
		case 9:
			cfg_param->vam.evam_broadcast_peroid = value;
			break;			


			
		case 10:
			cfg_param->vsa.danger_detect_speed_threshold = value;
			break;
		case 11:
			cfg_param->vsa.danger_alert_period = value;
			
			break;			
		case 12:
			cfg_param->vsa.crd_saftyfactor = value;
			break;			
		case 13:
			cfg_param->vsa.ebd_mode = value;
			break;
		case 14:
			cfg_param->vsa.ebd_acceleration_threshold = value;
			break;			
		case 15:
			cfg_param->vsa.ebd_alert_hold_time = value;
			break;


			
		case 16:
			cfg_param->print_xxx = value;

		default:
			break;

	}

	memcpy((uint8_t*)p_cms_param,(uint8_t*)cfg_param,sizeof(cfg_param_t));
		
	drv_fls_erase(FLASH_Sector_11);
	drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
	
	err = drv_fls_write(PARAM_ADDR,(uint8_t*)cfg_param,sizeof(cfg_param_t));

	if(err == -1)
		rt_kprintf("param set error!!!\n");
	else
		rt_kprintf("param set success!\n");

	rt_free(cfg_param);

	cfg_param = NULL;

	return 0;

}


FINSH_FUNCTION_EXPORT(param_set, set system parameters);




void flash_read(void)
{
	cfg_param_t  *param_temp;

	param_temp = (cfg_param_t*)rt_malloc(sizeof(cfg_param_t));

	drv_fls_read(PARAM_ADDR,(uint8_t *)param_temp,sizeof(cfg_param_t));
		
    rt_kprintf("-------------------parameters in  flash------------------\n");
	rt_kprintf("ID(id)=%d%d%d%d\n",param_temp->pid[0],param_temp->pid[1],param_temp->pid[2],param_temp->pid[3]);
    rt_kprintf("vam.bsm_hops(vam.bh)=%d\n", param_temp->vam.bsm_hops);
    rt_kprintf("vam.bsm_boardcast_mode(vam.bbm)=%d\n", param_temp->vam.bsm_boardcast_mode);
    rt_kprintf("vam.bsm_boardcast_saftyfactor(vam.bbs)=%d\n", param_temp->vam.bsm_boardcast_saftyfactor);
    rt_kprintf("vam.bsm_pause_mode(vam.bpm)=%d\n", param_temp->vam.bsm_pause_mode);
    rt_kprintf("vam.bsm_pause_hold_time(vam.bpht)=%d (s)\n", param_temp->vam.bsm_pause_hold_time);
    rt_kprintf("vam.bsm_boardcast_period(vam.bbp)=%d (ms)\n", param_temp->vam.bsm_boardcast_period);

	rt_kprintf("vam.evam_hops(vam.eh)=%d\n", param_temp->vam.evam_hops);
	rt_kprintf("vam.evam_broadcast_type(vam.ebt)=%d\n", param_temp->vam.evam_broadcast_type);
	rt_kprintf("vam.evam_broadcast_peroid(vam.ebp)=%d\n\n", param_temp->vam.evam_broadcast_peroid);

    rt_kprintf("vsa.danger_detect_speed_threshold(vsa.ddst)=%d (m/s)\n", param_temp->vsa.danger_detect_speed_threshold);
    rt_kprintf("vsa.danger_alert_period(vsa.dap)=%d (ms)\n", param_temp->vsa.danger_alert_period);
	rt_kprintf("vsa.crd_saftyfactor(vsa.cs)=%d\n", param_temp->vsa.crd_saftyfactor);
    rt_kprintf("vsa.ebd_mode(vsa.em)=%d\n", param_temp->vsa.ebd_mode);
    rt_kprintf("vsa.ebd_acceleration_threshold(vsa.eat)=%d (m/s2)\n", param_temp->vsa.ebd_acceleration_threshold);
    rt_kprintf("vsa.ebd_alert_hold_time(vsa.eaht)=%d (s)\n", param_temp->vsa.ebd_alert_hold_time);

    rt_kprintf("...\n");

    rt_kprintf("----------------------end---------------------\n");	

	rt_free(param_temp);

	param_temp = NULL;

}

FINSH_FUNCTION_EXPORT(flash_read, debug:reading param);










