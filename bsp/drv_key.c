#include "drv_key.h"

#include "drv_uart.h"
#include "init.h"
#include "fifo.h"


#define LOG_TAG "drv_key"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"


static void key_obj_init(key_s_obj_p obj, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t (*read_key_f)(void *), bool inv);
static uint8_t read_key(void *parm);



void key_s_obj_create(key_s_obj_p obj, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool inv)
{
		key_obj_init(obj,GPIOx,GPIO_Pin,read_key,inv);
	
}


static uint8_t read_key(void *parm)
{
		key_s_obj_p obj_p;
		if(parm != NULL)
				obj_p = parm;
		else
				log_e("null pointer!");
		
		obj_p->io_val = HAL_GPIO_ReadPin(obj_p->GPIOx,obj_p->GPIO_Pin);
		
		if(obj_p->inverse == 1)
				obj_p->output = !obj_p->io_val;
		else
				obj_p->output = obj_p->io_val;
		
		return obj_p->output;
}

static void key_obj_init(key_s_obj_p obj, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t (*read_key_f)(void *), bool inv)
{
		obj->GPIOx = GPIOx;
		obj->GPIO_Pin = GPIO_Pin;
		obj->read_key_f = read_key_f;
		obj->inverse = inv;
}


