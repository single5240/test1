#ifndef _MODBUS2_TASK_H
#define _MODBUS2_TASK_H


#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"

void Modbus2_Task(void const * argument);
void modbus_usart3_rx_callback(uint8_t *buff, uint16_t len);
void* get_mbr_no3(void);




#ifdef __cplusplus
}
#endif


#endif




