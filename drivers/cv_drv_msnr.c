#include <stdbool.h>
#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "l3gd20.h"
#include "led.h"

struct rt_mems_device
{
    struct rt_device parent;
    rt_bool_t calibrating;
    float x;
	float y;
	float z;
    struct rt_spi_device * spi_device;
    struct rt_event event;
};
static struct rt_mems_device *MEMS = RT_NULL;

rt_inline void MEMS_int_cmd(FunctionalState NewState);

static void MEMS_NVIC_Configuration(void)
{
//    NVIC_InitTypeDef NVIC_InitStructure;

//    /* Enable the EXTI0 Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
}

rt_inline void MEMS_int_cmd(FunctionalState NewState)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure  EXTI  */
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

    EXTI_InitStructure.EXTI_LineCmd = NewState;

    EXTI_ClearITPendingBit(EXTI_Line3);
    EXTI_Init(&EXTI_InitStructure);
}

static void MEMS_EXTI_Configuration(void)
{
//    {
//        GPIO_InitTypeDef GPIO_InitStructure;

//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);
//    }

//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
    MEMS_int_cmd(ENABLE);
}

/* RT-Thread Device Interface */
static rt_err_t rt_mems_init (rt_device_t dev)
{
//    uint8_t send;
//    struct rtgui_touch_device * touch_device = (struct rtgui_touch_device *)dev;
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
    MEMS_NVIC_Configuration();
    MEMS_EXTI_Configuration();
	
	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
  
    return RT_EOK;
}

static rt_err_t rt_mems_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
		;
    }

    return RT_EOK;
}

static void mems_thread_entry(void *parameter)
{
	uint8_t tmpreg;
	uint8_t send_buffer[2]={0};
	uint8_t recv_buffer[10]= {0};
	uint8_t wCounter = 0 ;
	int16_t RawData[3] = {0};
	float   sensitivity = 0;
	float   pfData[3]={0};
	float   Xval, Yval = 0x00;
	while(1) {
		send_buffer[0] = L3GD20_CTRL_REG4_ADDR|READWRITE_CMD;
		rt_spi_send_then_recv(MEMS->spi_device,send_buffer,1,&tmpreg,1);
		send_buffer[0] = L3GD20_OUT_X_L_ADDR|READWRITE_CMD | MULTIPLEBYTE_CMD;
		rt_spi_send_then_recv(MEMS->spi_device,send_buffer,1,recv_buffer,6);
		if(!(tmpreg & 0x40)){
			for(wCounter=0; wCounter<3; wCounter++){
				RawData[wCounter]=(int16_t)(((uint16_t)recv_buffer[2*wCounter+1] << 8) + recv_buffer[2*wCounter]);
			}
		}
		else{
			for(wCounter=0; wCounter<3; wCounter++){
				RawData[wCounter]=(int16_t)(((uint16_t)recv_buffer[2*wCounter] << 8) + recv_buffer[2*wCounter+1]);
			}
		}
		switch(tmpreg & 0x30)
		{
			case 0x00:
				sensitivity=L3G_Sensitivity_250dps;
			break;
			case 0x10:
				sensitivity=L3G_Sensitivity_500dps;
			break;
			case 0x20:
				sensitivity=L3G_Sensitivity_2000dps;
			break;
		}
		for(wCounter=0; wCounter<3; wCounter++){
			pfData[wCounter]=(float)RawData[wCounter]/sensitivity;
		}
		
		Xval = ABS((float)(pfData[0]));
		Yval = ABS((float)(pfData[1])); 

		if ( Xval>Yval){
			if ((int8_t)pfData[0] > 15.0f){  
               #ifdef HARDWARE_DVB_F401Disco
				STM_EVAL_LEDOn(LED4);
				STM_EVAL_LEDOff(LED3);
				STM_EVAL_LEDOff(LED5);
				STM_EVAL_LEDOff(LED6);
                #endif
			}
			if ((int8_t)pfData[0] < -15.0f){
               #ifdef HARDWARE_DVB_F401Disco
				STM_EVAL_LEDOn(LED5);
				STM_EVAL_LEDOff(LED3);
				STM_EVAL_LEDOff(LED4);
				STM_EVAL_LEDOff(LED6);
                #endif
			}
		}
		else{
			if ((int8_t)pfData[1] < -15.0f){
               #ifdef HARDWARE_DVB_F401Disco
				STM_EVAL_LEDOn(LED3);
				STM_EVAL_LEDOff(LED4);
				STM_EVAL_LEDOff(LED5);
				STM_EVAL_LEDOff(LED6);
                #endif
			}
			if ((int8_t)pfData[1] > 15.0f){
               #ifdef HARDWARE_DVB_F401Disco
				STM_EVAL_LEDOn(LED6);
				STM_EVAL_LEDOff(LED3);
				STM_EVAL_LEDOff(LED4);
				STM_EVAL_LEDOff(LED5);
                #endif
			} 
		}
		rt_thread_delay(1);
    } 
}

void EXTI3_IRQHandler(void)
{
    /* disable interrupt */
//    touch_int_cmd(DISABLE);

//    rt_event_send(&touch->event, 1);

//    EXTI_ClearITPendingBit(EXTI_Line3);
}

rt_err_t rt_mems_hw_init(const char * spi_device_name)
{
    struct rt_spi_device * spi_device;
    struct rt_thread * mems_thread;

    spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(spi_device == RT_NULL) {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = 500 * 1000; /* 500K */
        rt_spi_configure(spi_device, &cfg);
    }

    MEMS = (struct rt_mems_device*)rt_malloc (sizeof(struct rt_mems_device));
    if (MEMS == RT_NULL) {
		return RT_ENOMEM; /* no memory yet */
	}
    /* clear device structure */
    rt_memset(&(MEMS->parent), 0, sizeof(struct rt_device));

//    rt_event_init(&MEMS->event, "MEMS", RT_IPC_FLAG_FIFO);

    MEMS->spi_device = spi_device;

    /* init device structure */
    MEMS->parent.type      = RT_Device_Class_Miscellaneous;
    MEMS->parent.init      = rt_mems_init;
    MEMS->parent.control   = rt_mems_control;
    MEMS->parent.user_data = RT_NULL;

	/* register touch device to RT-Thread */
	rt_device_register(&(MEMS->parent), "MEMS", RT_DEVICE_FLAG_RDWR);

    mems_thread = rt_thread_create("MEMS",
                                    mems_thread_entry, RT_NULL,
                                    1024, RT_MEMS_THREAD_PRIORITY, 10);
    if (mems_thread != RT_NULL) rt_thread_startup(mems_thread);

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void touch_t( rt_uint16_t x , rt_uint16_t y )
{
	;
}

FINSH_FUNCTION_EXPORT(touch_t, x & y ) ;
#endif
