#include "io_task.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"

#include "drv_uart.h"
#include "init.h"
#include "radio_task.h"
#include "event_mgr.h"
#include "drv_key.h"
#include "drv_led.h"

#define LOG_TAG "io_task"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

io_manage_obj_t io_manage_obj = {0};

static void io_manage_init(void);
static void an1_key_led_handle(an_key_obj_p obj);
static void an2_key_led_handle(an_key_obj_p obj);



void IO_Task(void const * argument)
{		
		
	io_manage_init();

	log_i("io_task_launch!");	
		
    uint32_t period = osKernelSysTick();
	while(1)
	{
		io_manage_obj.io_sampling_f(&io_manage_obj.io_data);
	
		an1_key_led_handle(&io_manage_obj.obj_an1_key);
		an2_key_led_handle(&io_manage_obj.obj_an2_key);
			
		if(io_manage_obj.obj_an2_key.key.output == 1)
			rc_auto_match_start();

		osDelayUntil(&period, 20);
	}
}

static void io_data_sampling(io_data_struct_p io_struct)
{
	
	io_struct->an1_key = 	io_manage_obj.obj_an1_key.key.read_key_f(&io_manage_obj.obj_an1_key.key);
	io_struct->an2_key = 	io_manage_obj.obj_an2_key.key.read_key_f(&io_manage_obj.obj_an2_key.key);
}



static void io_manage_init(void)
{
		io_manage_obj.io_sampling_f = io_data_sampling;
	
		key_s_obj_create(&io_manage_obj.obj_an1_key.key,AN1_KEY_GPIO_Port,AN1_KEY_Pin,1);
		key_s_obj_create(&io_manage_obj.obj_an2_key.key,AN2_KEY_GPIO_Port,AN2_KEY_Pin,1);

		led_obj_create(&io_manage_obj.obj_an1_key.led,AN1_LED_GPIO_Port,AN1_LED_Pin,0);
		led_obj_create(&io_manage_obj.obj_an2_key.led,AN2_LED_GPIO_Port,AN2_LED_Pin,0);
}


static void an1_key_led_handle(an_key_obj_p obj)
{
		obj->led.set_led_f(&obj->led,obj->key.output);
}


static void an2_key_led_handle(an_key_obj_p obj)
{
		obj->led.set_led_f(&obj->led,obj->key.output);
}


