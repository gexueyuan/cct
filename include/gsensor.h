#ifndef __GSENSOR_H__
#define	__GSENSOR_H__

#include "board.h"

#ifdef HARDWARE_MODULE_V1
#define GSENSOR_LSM303DLHC
#endif

#ifdef HARDWARE_MODULE_V2
#define GSENSOR_BMA250E
#endif

#define GSNR_POLL_TIME_INTERVAL MS_TO_TICK(200)

typedef enum {
    GSNR_CRIT = 0,
    GSNR_ERR,
    GSNR_WARNING,
    GSNR_NOTICE,
    GSNR_INFO,
    GSNR_DEBUG,
}gsnr_log_level_t;

#if 0
#define GSNR_LOG(format, ...)  rt_kprintf(format, ##__VA_ARGS__)
#endif

#define GSNR_LOG(level, format, ...)  \
    do\
    {\
        if(level <= gsnr_log_lvl) \
            rt_kprintf(format, ##__VA_ARGS__); \
    }\
    while(0)

typedef struct _GSENSOR_INFO
{
	float x;
	float y;
	float z ;
	float sum;
}GSENSOR_INFO;

extern GSENSOR_INFO gSensor_Average, gSensor_Threshold, Acce_Sum, Acce_V;
extern uint8_t AdjustGsensor;


/* read, write gsensor register api */
void gsnr_read(uint8_t reg, uint8_t num);
void gsnr_write(uint8_t reg, uint8_t data);
void gsnr_init(void);


//==============================================================================
//                                   0ooo
//                          ooo0     (   ) 
//                          (   )     ) /
//                           \ (     (_/
//                            \_) 
//==============================================================================
#endif	/* __GSENSOR_H__ */

