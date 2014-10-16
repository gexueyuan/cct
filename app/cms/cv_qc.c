/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_qc.c
 @brief  : this file include the QC functions
 @author : gexueyuan
 @history:
           2014-10-14    gexueyuan    Created file
           ...
******************************************************************************/
typedef struct
{
	rt_thread_t gps_qc;
	rt_thread_t gsensor_qc;
	rt_thread_t key_qc;

}qc_task_t;

qc_task_t* qc_task; 

int start_qc_task(void)
{
    rt_thread_startup(qc_task->gps_qc);
	
    rt_thread_startup(qc_task->gsensor_qc);

	rt_thread_startup(qc_task->key_qc);
}

void rt_GPS_QC_entry(void *parameter)
{


}
void rt_Gsensor_QC_entry(void *parameter)
{
	


}

void rt_key_QC_entry(void *parameter)
{


}

int rt_QC_application_init(void)
{
	rt_thread_t pid;

	rt_kprintf("begin QC process!!......");

	qc_task->gps_qc = rt_thread_create("qc-gps",rt_GPS_QC_entry,RT_NULL,/
			RT_INIT_THREAD_STACK_SIZE,RT_QC_THREAD_PRIORTY,20);
	RT_ASSERT(qc_task->gps_qc != RT_NULL);

	qc_task->gsensor_qc = rt_thread_create("qc-gsensor",rt_Gsensor_QC_entry,RT_NULL,/
			RT_INIT_THREAD_STACK_SIZE,RT_QC_THREAD_PRIORTY,20);
	RT_ASSERT(qc_task->gsensor_qc != RT_NULL);

	
	qc_task->key_qc = rt_thread_create("qc-key",rt_key_QC_entry,RT_NULL,/
			RT_INIT_THREAD_STACK_SIZE,RT_QC_THREAD_PRIORTY,20);	
	RT_ASSERT(qc_task->key_qc != RT_NULL);
	
}
