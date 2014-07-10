#include <string.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "gps.h"
#include "nmea.h"

t_gps_buff __GPSBuff;
gps_data_callback gps_recv_cb = NULL;
uint8_t _GPSBuffer[GPS_BUFF_SIZE];
uint8_t get_gps = 0 ;

struct rt_mutex  mutex_gps ;



void gps_callback_register(gps_data_callback fp)
{
	gps_recv_cb = fp;
}

static void gps_read_data(rt_device_t dev)
{
	uint8_t tmp = 0 ;

	while(1){
		if(rt_device_read(dev, 0, &tmp, 1) == 1){
			if (tmp == '$') {
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Len = 1;
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[0] = '$';
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Flag = 1;	
			}
			else if (tmp == '\n') {
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[__GPSBuff.PpBuf[__GPSBuff.Pipe].Len] = '\n';
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Len++;
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Len %= GPS_BUFF_SIZE;
				__GPSBuff.PpBuf[__GPSBuff.Pipe].Flag = 0;

                {
                    sys_msg_t msg;
                    msg.id = VAM_MSG_GPSDATA;
                    msg.len = __GPSBuff.PpBuf[__GPSBuff.Pipe].Len;
                    msg.argv = &__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf;
                    vam_add_event_queue_2(&p_cms_envar->vam, &msg);
                }

				__GPSBuff.Pipe++;
				__GPSBuff.Pipe %= GPS_PIPE;
				get_gps = 1 ;
			}
			else {
				if (__GPSBuff.PpBuf[__GPSBuff.Pipe].Flag == 1) {
					__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[__GPSBuff.PpBuf[__GPSBuff.Pipe].Len] = tmp;
					__GPSBuff.PpBuf[__GPSBuff.Pipe].Len++;
					__GPSBuff.PpBuf[__GPSBuff.Pipe].Len %= GPS_BUFF_SIZE;
				}
			}
		}
		else{
			break ;
		}
	}
}

void rt_gps_thread_entry (void *parameter) 
{
	rt_device_t dev ;
    
	dev = rt_device_find(RT_GPS_DEVICE_NAME);
	rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) ;

	while(1){
		gps_read_data(dev);
		rt_thread_delay(1);
	}
}

//*******************************************************************************
// 函数名称    : void gps_init(void)
// 功能描述    : 初始化gps模块，例如屏蔽不需要的信息，天线断开延时
// 输入        : None
// 输出        : None
// 返回        : None
//******************************************************************************/
void gps_init(void)
{
    rt_thread_t tid;

	memset(&__GPSBuff, 0, sizeof(__GPSBuff));
	gps_recv_cb = NULL;


	tid = rt_thread_create("t-gps",
                           rt_gps_thread_entry, RT_NULL,
                           RT_GPS_THREAD_STACK_SIZE, RT_GPS_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL)
    rt_thread_startup(tid);
}

void gps_deinit(void)
{
	memset(&__GPSBuff, 0, sizeof(__GPSBuff));
	gps_recv_cb = NULL;	
}

//==============================================================================
//                                   0ooo
//                          ooo0     (   ) 
//                          (   )     ) /
//                           \ (     (_/
//                            \_) 
//==============================================================================


