#ifndef _MODBUS1_TASK_H
#define _MODBUS1_TASK_H


#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"





void Modbus1_Task(void const * argument);
void modbus_usart2_rx_callback(uint8_t *buff, uint16_t len);
void* get_mbr_no2(void);




#ifdef __cplusplus
}
#endif


#endif




