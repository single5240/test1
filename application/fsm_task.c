#include "fsm_task.h"

#include "cmsis_os.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "drv_modbus.h"
#include "modbus1_task.h"
#include "modbus2_task.h"
#include "drv_e28.h"
#include "radio_task.h"
#include "event_mgr.h"

#define LOG_TAG "fsm_task"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"



void Fsm_Task(void const * argument)
{	
	
	modbus_rtu_p mbr2 = (modbus_rtu_p)get_mbr_no2();
	modbus_rtu_p mbr3 = (modbus_rtu_p)get_mbr_no3();
	e28_manage_obj_p e28 = (e28_manage_obj_p)get_e28_obj();
	
	
	log_i("Fsm_Task_launch!");

    uint32_t period = osKernelSysTick();
	
	while(1)
	{
		
		if(mbr2->fsm_eventHandle_f != NULL)
				mbr2->fsm_eventHandle_f(mbr2);
		else
			log_e("null pointer!");
		
		
		if(mbr3->fsm_eventHandle_f != NULL)
				mbr3->fsm_eventHandle_f(mbr3);
		else
			log_e("null pointer!");
		
		if(e28->fsm_eventHandle_f != NULL)
			e28->fsm_eventHandle_f(e28);
		else
			log_e("null pointer!");
		
		osDelayUntil(&period,10);
	}
}
