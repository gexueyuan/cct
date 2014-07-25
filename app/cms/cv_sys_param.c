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


#define PARAM_FLAG_ADDR     ((uint32_t)0x80E0000)

#define PARAM_ADDR    		((uint32_t)0x80E0010)

extern 	int drv_fls_erase(uint32_t	sector);
extern  int drv_fls_read(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);
extern	int drv_fls_write(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

cfg_param_t cms_param, *p_cms_param;
const char* env_var[] = {"vam.bh","vam.bbm","vam.bbs","vam.bpm","vam.bpht","vam.eh","vam.bbp",\
	                  	 "vsa.ddst","vsa.cs","vsa.em","vsa.eat","vsa.eaht","vsa.dap","print"};

const char	param_init_words[] = "Vanet-param";
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

rt_err_t  fw_param(void)
{
	cfg_param_t  flash_param;

	rt_err_t err;

	load_default_param(&flash_param);
	
	drv_fls_erase(FLASH_Sector_11);
	drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
	err = drv_fls_write(PARAM_ADDR,(uint8_t *)&flash_param,sizeof(cfg_param_t));

	rt_kprintf("write default param to flash");
	return err;

}

FINSH_FUNCTION_EXPORT(fw_param, debug:write default  param to flash);

void param_init(void)
{
	uint8_t magic_word[sizeof(param_init_words)];

	drv_fls_read(PARAM_FLAG_ADDR,magic_word,sizeof(param_init_words));

	if(strcmp(param_init_words,(const char*)magic_word) != 0)
		{
    		p_cms_param = &cms_param;
    		load_default_param(p_cms_param);
			fw_param();
		}

}


void param_get(void)
{
	//cfg_param_t  *param_temp;

	//drv_fls_read(PARAM_ADDR,(uint8_t *)param_temp,sizeof(cfg_param_t));
		
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


void write_test(void)
{
 uint8_t* test_string = "bbbb thin";
 drv_fls_erase(FLASH_Sector_11);
 drv_fls_write(PARAM_ADDR,test_string,10);

}

void read_test(void)
{
 uint8_t test_string[10];
 
 drv_fls_read(PARAM_ADDR,test_string,10);
 	
 rt_kprintf("test data is %s\n",test_string);

 

 rt_kprintf("vam_config_t  is %d bytes\n",sizeof(vam_config_t));

 
 rt_kprintf("vsa_config_t  is %d bytes\n",sizeof(vsa_config_t));

 
 rt_kprintf("cfg_param_t  is %d bytes\n",sizeof(cfg_param_t));

 rt_kprintf("env_var  is %d bytes\n",sizeof(env_var)/sizeof(*env_var));

 rt_kprintf("param_init_words is %d bytes\n",sizeof(param_init_words));

}

void print_mw(void)//print magic word
{
	
	uint8_t magic_word[sizeof(param_init_words)];

	
	drv_fls_read(PARAM_FLAG_ADDR,magic_word,sizeof(param_init_words));

	rt_kprintf("magic word in flash is \"%s\"\n",magic_word);

}

void print_fd(uint32_t addr)//print_fd(0x80E0010)
{

	uint8_t data;

	drv_fls_read(addr,&data,1);

	rt_kprintf("data in address %x  is \"%d\"\n",addr,data);
}
FINSH_FUNCTION_EXPORT(write_test, debug:testing flash);
FINSH_FUNCTION_EXPORT(read_test, debug:testing flash);

FINSH_FUNCTION_EXPORT(param_get, get system parameters);
FINSH_FUNCTION_EXPORT(print_mw, debug:print magic word in flash);
FINSH_FUNCTION_EXPORT(print_fd, print one data in flash);
void param_set(const char *param, uint16_t value)
{
	//uint8_t *p_string = param;

	int8_t  pos;

	cfg_param_t *cfg_param;

	cfg_param = (cfg_param_t*)rt_malloc(sizeof(cfg_param_t));

	drv_fls_read(PARAM_ADDR,(uint8_t*)cfg_param,sizeof(cfg_param_t));

	//pos = (&(cfg_param_t*)0)->param;
	//pos = offsetof(cfg_param_t,param);
	pos = get_param_pos(param);
	
	switch(pos){

		case 0:
			cfg_param->vam.bsm_hops = value;
			break;
		case 1:
			cfg_param->vam.bsm_boardcast_mode = value;
			break;		
		case 2:
			cfg_param->vam.bsm_boardcast_saftyfactor = value;
			break;
		case 3:
			cfg_param->vam.bsm_pause_mode = value;
			break;
		case 4:
			cfg_param->vam.bsm_pause_hold_time = value;
			break;
		case 5:
			cfg_param->vam.evam_hops = value;
			break;
		case 6:
			cfg_param->vam.bsm_boardcast_period = value;
			break;

			
		case 7:
			cfg_param->vsa.danger_detect_speed_threshold = value;
			break;
		case 8:
			cfg_param->vsa.crd_saftyfactor = value;
			break;			
		case 9:
			cfg_param->vsa.ebd_mode = value;
			break;
		case 10:
			cfg_param->vsa.ebd_acceleration_threshold = value;
			break;			
		case 11:
			cfg_param->vsa.ebd_alert_hold_time = value;
			break;
		case 12:
			cfg_param->vsa.danger_alert_period = value;
			break;

			
		case 13:
			cfg_param->print_xxx = value;

		default:
			break;

	}
	
	drv_fls_erase(FLASH_Sector_11);

	drv_fls_write(PARAM_ADDR,(uint8_t*)cfg_param,sizeof(cfg_param_t));

	rt_free(cfg_param);

}


FINSH_FUNCTION_EXPORT(param_set, set system parameters);




void flash_read(void)
{
	cfg_param_t  *param_temp;

	drv_fls_read(PARAM_ADDR,(uint8_t *)param_temp,sizeof(cfg_param_t));
		
    rt_kprintf("-------------------parameters in  flash------------------\n");
    rt_kprintf("vam.bsm_hops=%d\n", param_temp->vam.bsm_hops);
    rt_kprintf("vam.bsm_boardcast_mode=%d\n", param_temp->vam.bsm_boardcast_mode);
    rt_kprintf("vam.bsm_boardcast_saftyfactor=%d\n", param_temp->vam.bsm_boardcast_saftyfactor);
    rt_kprintf("vam.bsm_boardcast_period=%d (ms)\n", param_temp->vam.bsm_boardcast_period);
    rt_kprintf("vam.bsm_pause_mode=%d\n", param_temp->vam.bsm_pause_mode);
    rt_kprintf("vam.bsm_pause_hold_time=%d (s)\n", param_temp->vam.bsm_pause_hold_time);
    rt_kprintf("vam.evam_hops=%d\n", param_temp->vam.evam_hops);

    rt_kprintf("vsa.danger_detect_speed_threshold=%d (m/s)\n", param_temp->vsa.danger_detect_speed_threshold);
    rt_kprintf("vsa.danger_alert_period=%d (ms)\n", param_temp->vsa.danger_alert_period);
    rt_kprintf("vsa.crd_saftyfactor=%d\n", param_temp->vsa.crd_saftyfactor);
    rt_kprintf("vsa.ebd_mode=%d\n", param_temp->vsa.ebd_mode);
    rt_kprintf("vsa.ebd_acceleration_threshold=%d (m/s2)\n", param_temp->vsa.ebd_acceleration_threshold);
    rt_kprintf("vsa.ebd_alert_hold_time=%d (s)\n", param_temp->vsa.ebd_alert_hold_time);

    rt_kprintf("...\n");

    rt_kprintf("----------------------end---------------------\n");	


}

FINSH_FUNCTION_EXPORT(flash_read, debug:reading param);










