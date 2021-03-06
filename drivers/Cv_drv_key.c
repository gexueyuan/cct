/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_key.c
 @brief  : this file include the key functions
 @author : gexueyuan
 @history:
           2014-7-30    gexueyuan    Created file
           ...
******************************************************************************/
#include <rtthread.h>
#include <board.h>
#include "cv_cms_def.h"
#include "key.h"
#include "components.h"


static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* init gpio configuration */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
                           

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	#if LCD_VERSION!=1	//魔笛f4 使用上拉
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
	#endif
#ifdef HARDWARE_MODULE_V1	
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
#elif defined(HARDWARE_MODULE_V2)  
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
#endif
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}




static void key_thread_entry(void *parameter)
{
    sys_envar_t *p_sys = (sys_envar_t *)parameter;
	uint32_t  key_value = 0;
	
	
	GPIO_Configuration();
	
	key = (struct rtgui_key*)rt_malloc (sizeof(struct rtgui_key));
    if (key == RT_NULL)
		return ; /* no memory yet */
	
	
	key->key_last = 0;
	key->key_current = 0;
	key->key_get = 0;
	key->key_debounce_count = 0;
	key->key_long_count = 0;
	key->key_special_count = 0;
	key->key_relase_count = 0;
	key->key_flag = 0;	
	
	while(1)
	{	
		rt_thread_delay(2);	
		
		key->key_current = key_up_GETVALUE();
		key->key_current |= key_down_GETVALUE()<<1;	
		
	  #if LCD_VERSION==1


	  #else 
		key->key_current=~(key->key_current);
		key->key_current&=0x00000003;
	
	  #endif

		key->key_flag &= ~C_FLAG_SHORT;
		key->key_flag &= ~C_FLAG_COUNT;
		key->key_flag &= ~C_FLAG_LONG;
		key->key_get = 0;	


	/*对于有长按和短按的特殊键做处理*/	
	if ((key->key_flag)&C_FLAG_RELASE)
	{//检查放键
		if (key->key_current == 0)
		{
			if ((++(key->key_relase_count)) >= C_RELASE_COUT)
			{ //按键已经放开
				key->key_relase_count = 0;
				key->key_flag &= ~C_FLAG_RELASE;
			}
		}
		else
		{
			key->key_relase_count = 0;
		}
	}
	else
	{//检查按键
		if (key->key_current == C_SPECIAL_KEY)		
		{
			if ((++(key->key_special_count)) >= C_SPECIAL_LONG_COUT)
			{
				key->key_special_count = 0;
				
				key->key_get = C_HOME_KEY;      
				key->key_flag |= C_FLAG_LONG;	//特殊键 长按键按下
				key->key_flag |= C_FLAG_RELASE;;//按下后要求检测放键
			}
		}
		else
		{//放开键后才检测短按
			if ((key->key_special_count >= C_SHORT_COUT) && (key->key_special_count <C_SHORT_COUT+30)) 
			{
				key->key_get = C_SPECIAL_KEY;
				key->key_flag |= C_FLAG_SHORT;	//特殊键 短按键按下
			}
			key->key_special_count = 0;
		}
	}
	
// 普通按键处理
	if((key->key_current == 0)||(key->key_current != key->key_last)|| (key->key_current == C_SPECIAL_KEY))
	{
		key->key_debounce_count = 0;	//第一次	
		key->key_long_count=0;	        //清除长按键计数器
	}
	else
	{
		if(++(key->key_debounce_count) == DEBOUNCE_SHORT_TIME)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_SHORT;	//短按键按下
		}
		if(key->key_debounce_count == DEBOUNCE_COUT_FIRST + DEBOUNCE_COUT_INTERVAL)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_COUNT;	//连按键 按键按下
			key->key_debounce_count = DEBOUNCE_COUT_FIRST;
			++(key->key_long_count);			
		}
	
		if(key->key_long_count == DEBOUNCE_LONG_TIME)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_LONG;	//短按键按下
			key->key_long_count=DEBOUNCE_LONG_TIME+1;
		}		
	}
	
	key->key_last = key->key_current;				// 保存本次键值
			
	if (key->key_get)
	{	
		if (((key->key_get)==C_UP_KEY) && ((key->key_flag) & C_FLAG_SHORT))
			key_value = C_UP_KEY;
	
		if (((key->key_get)==C_DOWN_KEY) && ((key->key_flag) & C_FLAG_SHORT))
			key_value = C_DOWN_KEY;
		
		if(key_value)	
			sys_add_event_queue(p_sys,SYS_MSG_KEY_PRESSED,0,key_value,NULL);

		key_value = 0;
	}	
	
	}
}

	
int rt_key_init(void)
{
    rt_thread_t key_tid;
    sys_envar_t *p_sys = &p_cms_envar->sys;
	
    key_tid = rt_thread_create("key",
                               key_thread_entry, p_sys,
                               512, 13, 5);
    if (key_tid != RT_NULL) rt_thread_startup(key_tid);

	return 0;
}



