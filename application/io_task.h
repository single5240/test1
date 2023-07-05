#ifndef __IO_TASK_H__
#define __IO_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "proj_config.h"
#include "stm32f4xx_hal.h"


#include "drv_key.h"
#include "drv_led.h"




#pragma pack(push,1)		//���������Լ��Ķ��뷽ʽ��ѹ��ջ���������������߱���������ľͰ���1�ֽڶ���
typedef struct io_data_struct						//�������Ϊ1�ֽڣ�1�ֽڶ��롣
{
	
		uint8_t  an1_key	: 1;				//��λ
		uint8_t  an2_key	: 1;				
			
}io_data_struct_t,*io_data_struct_p;
#pragma pack(pop)				//)��ԭ���Ķ��뷽ʽ���ͷų�����




typedef struct
{
		key_s_obj_t key;
		
		led_s_obj_t led;
	
}an_key_obj_t,*an_key_obj_p;


typedef struct
{
    void (*io_sampling_f)(io_data_struct_p);  
	
		an_key_obj_t obj_an1_key;
		an_key_obj_t obj_an2_key;

		io_data_struct_t io_data;
		
} io_manage_obj_t,*io_manage_obj_p;








void IO_Task(void const * argument);
	
#ifdef __cplusplus
}
#endif

#endif // __INIT_H__
