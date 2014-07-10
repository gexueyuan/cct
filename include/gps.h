#ifndef	__GPS_H__
#define	__GPS_H__
#include <stdint.h>


#define GPSEVENT					0x0001

#define	GPS_RX		GPIO_Pin_10		// pa10
#define GPS_TX		GPIO_Pin_9		// pa9

#define GPS_BUFF_SIZE 256
#define GPS_PIPE 5
#define RT_GPS_DEVICE_NAME	"uart1"

extern struct rt_mutex  mutex_gps ;

typedef struct {
	uint8_t Flag;
	uint8_t Buf[GPS_BUFF_SIZE];
	uint32_t Len;
} t_buff;

typedef struct {
	uint8_t Pipe;
	t_buff PpBuf[GPS_PIPE];
}t_gps_buff;

typedef void (*gps_data_callback)(uint8_t *, uint32_t);

extern t_gps_buff __GPSBuff;
//extern gps_data_callback gps_recv_cb;

void gps_callback_register(gps_data_callback fp);

void gps_init(void);
void gps_deinit(void);

void gps_task (void) ;

//==============================================================================
//                                   0ooo
//                          ooo0     (   ) 
//                          (   )     ) /
//                           \ (     (_/
//                            \_) 
//==============================================================================
#endif	/* __GPS_H__ */

