#ifndef __KEY_H
#define __KEY_H 

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
#ifdef HARDWARE_MODULE_V1
#define key_up_GETVALUE()     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)
#define key_down_GETVALUE()   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#elif defined(HARDWARE_MODULE_V2)
#define key_up_GETVALUE()     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define key_down_GETVALUE()   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
#endif

static struct rtgui_key *key;

extern int32_t vam_active_alert(uint32_t alerttype);
extern int32_t vam_cancel_alert(uint32_t alerttype);

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

#endif
