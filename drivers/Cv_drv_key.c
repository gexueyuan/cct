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


//���水������ʹ��
#define DEBOUNCE_SHORT_TIME 	3   // �ᴥ��������ʱ��5����λ��20���룩
#define DEBOUNCE_LONG_TIME  	5  // ������ʱ��DEBOUNCE_COUT_FIRST+DEBOUNCE_COUT_INTERVAL*DEBOUNCE_LONG_TIME����λ��10���룩
#define DEBOUNCE_COUT_FIRST 	500//50 // ���������ʱ��100����λ��20���룩
#define DEBOUNCE_COUT_INTERVAL 	10  // ���������ʱ��50����λ��20���룩

//���ⰴ������ʹ��
#define C_RELASE_COUT			3
#define C_SHORT_COUT			3	//3*20ms
#define C_SPECIAL_LONG_COUT		60  //60*20ms

//������־
#define C_FLAG_SHORT			0x00000001
#define C_FLAG_COUNT			0x00000002
#define C_FLAG_LONG				0x00000004
#define C_FLAG_RELASE			0x00000008

//������ֵ
#define  C_UP_KEY 				0x1
#define  C_DOWN_KEY 			0x2
#define  C_LEFT_KEY 			0x4
#define  C_RIGHT_KEY 			0x8
#define  C_STOP_KEY 			0x10
#define  C_MENU_KEY 			0x20
#define  C_ENTER_KEY 			0x40
#define  C_SPECIAL_KEY 			C_ENTER_KEY



/*enter������ ���Ƕ���Ϊhome��*/
#define  C_HOME_KEY 			C_SPECIAL_KEY+0x44

/*
KEY_UP    PF6
KEY_DOWN  PF7
KEY_LEFT  PF8
KEY_RIGHT PF9
KEY_STOP  PF10
KEY_MENU  PF11
KEY_ENTER PA0  (WKUP)
*/
#define key_up_GETVALUE()     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)
#define key_down_GETVALUE()   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)


/* ʹ���������ķ�ʽ�����������õ�������Ԫ�ؽ��д�� */
struct rtgui_key
{
    rt_timer_t poll_timer;
	
	rt_uint32_t key_last;
	rt_uint32_t key_current;
	//��⵽�İ���ֵ
	rt_uint32_t key_get;
	//���水��ʹ��
	rt_uint32_t key_debounce_count;
	rt_uint32_t key_long_count;
	//���ⰴ��ʹ��
	rt_uint32_t key_special_count;	
	rt_uint32_t key_relase_count;
	//������־
	rt_uint32_t key_flag;
	
};

static struct rtgui_key *key;


static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* init gpio configuration */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOF,
                           ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	#if LCD_VERSION!=1	//ħ��f4 ʹ������
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	
	#endif
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
                                   
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}




static void key_thread_entry(void *parameter)
{
	
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


	/*�����г����Ͷ̰��������������*/	
	if ((key->key_flag)&C_FLAG_RELASE)
	{//���ż�
		if (key->key_current == 0)
		{
			if ((++(key->key_relase_count)) >= C_RELASE_COUT)
			{ //�����Ѿ��ſ�
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
	{//��鰴��
		if (key->key_current == C_SPECIAL_KEY)		
		{
			if ((++(key->key_special_count)) >= C_SPECIAL_LONG_COUT)
			{
				key->key_special_count = 0;
				
				key->key_get = C_HOME_KEY;      
				key->key_flag |= C_FLAG_LONG;	//����� ����������
				key->key_flag |= C_FLAG_RELASE;;//���º�Ҫ����ż�
			}
		}
		else
		{//�ſ�����ż��̰�
			if ((key->key_special_count >= C_SHORT_COUT) && (key->key_special_count <C_SHORT_COUT+30)) 
			{
				key->key_get = C_SPECIAL_KEY;
				key->key_flag |= C_FLAG_SHORT;	//����� �̰�������
			}
			key->key_special_count = 0;
		}
	}
	
// ��ͨ��������
	if((key->key_current == 0)||(key->key_current != key->key_last)|| (key->key_current == C_SPECIAL_KEY))
	{
		key->key_debounce_count = 0;	//��һ��	
		key->key_long_count=0;	        //���������������
	}
	else
	{
		if(++(key->key_debounce_count) == DEBOUNCE_SHORT_TIME)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_SHORT;	//�̰�������
		}
		if(key->key_debounce_count == DEBOUNCE_COUT_FIRST + DEBOUNCE_COUT_INTERVAL)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_COUNT;	//������ ��������
			key->key_debounce_count = DEBOUNCE_COUT_FIRST;
			++(key->key_long_count);			
		}
	
		if(key->key_long_count == DEBOUNCE_LONG_TIME)
		{
			key->key_get = key->key_current;
			key->key_flag |= C_FLAG_LONG;	//�̰�������
			key->key_long_count=DEBOUNCE_LONG_TIME+1;
		}		
	}
	
	key->key_last = key->key_current;				// ���汾�μ�ֵ
			
	if (key->key_get)
	{	
		if (((key->key_get)==C_UP_KEY) && ((key->key_flag) & C_FLAG_SHORT))
		{	
			
			rt_kprintf("key = %x \n",key->key_get);
			//volmn++;
			//if (volmn>=100)volmn=100;
			//rt_kprintf("volmn=%d\r\n",volmn);
		}	
		
		if (((key->key_get)==C_DOWN_KEY) && ((key->key_flag) & C_FLAG_SHORT))
		{	
			
			rt_kprintf("key = %x \n",key->key_get);
			//if (volmn>=1)
			//{
			//	volmn--;
			//}
			
			//rt_kprintf("volmn=%d\r\n",volmn);			
		}	
		
	}	
	
	}
}

	
int rt_key_init(void)
{
    rt_thread_t key_tid;
	
    key_tid = rt_thread_create("key",
                               key_thread_entry, RT_NULL,
                               512, 13, 5);
    if (key_tid != RT_NULL) rt_thread_startup(key_tid);

	return 0;
}



