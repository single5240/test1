#ifndef __DRV_KEY_H__
#define __DRV_KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "stm32f4xx_hal.h"



typedef struct
{
		GPIO_TypeDef* GPIOx;
		uint16_t GPIO_Pin;
	
    uint8_t (*read_key_f)(void *);  
	
		uint8_t io_val;
		uint8_t output;
	
		bool inverse;
		
}key_s_obj_t,*key_s_obj_p;


void key_s_obj_create(key_s_obj_p obj, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool inv);



#ifdef __cplusplus
}
#endif


#endif // __DRV_KEY_H__


