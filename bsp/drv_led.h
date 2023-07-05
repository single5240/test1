#ifndef __DRV_LED_H__
#define __DRV_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "stm32f4xx_hal.h"



typedef struct
{
		GPIO_TypeDef* GPIOx;
		uint16_t GPIO_Pin;
	
    uint8_t (*set_led_f)(void *, uint8_t);  
    uint8_t (*toggle_f)(void *);  
		uint8_t state;

		bool inverse;
		
		int16_t led_cnt;
	
}led_s_obj_t,*led_s_obj_p;


void led_obj_create(led_s_obj_p obj, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool inv);

#ifdef __cplusplus
}
#endif


#endif // __DRV_LED_H__


