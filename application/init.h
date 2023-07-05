#ifndef __INIT_H__
#define __INIT_H__

#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "proj_config.h"
#include "radio_task.h"

#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"


#define RC_DATA_MSG  0
#define RC_DATA_MSG_LEN sizeof(rc_data_pack_t)

#define RADIO_RX_MSG  1
#define RADIO_RX_MSG_LEN 32


#define  ENV_BOOT_TIMES 		"boot_times"
#define  ENV_RADIO_ADDR 		"radio_addr"
#define  ENV_RADIO_CHAN 		"radio_chan"
#define  ENV_RADIO_MATCH_KEY 	"radio_match_key"


#pragma pack(push,1)		//���������Լ��Ķ��뷽ʽ��ѹ��ջ���������������߱���������ľͰ���1�ֽڶ���
typedef struct env_pack						
{
	struct
	{
		uint16_t addr;
		uint8_t chan;
		uint16_t match_key;
	}radio;

	struct
	{
		int32_t boot_times;
	}sys;
	
	uint8_t  crcH;			
	uint8_t  crcL; 			
	
}env_pack_t,*env_pack_p;
#pragma pack(pop)				//)��ԭ���Ķ��뷽ʽ���ͷų�����



void board_config(void);




uint32_t get_time_us(void);
uint32_t get_time_ms(void);
float get_time_ms_us(void);
void* get_rc_env(void);
void printf_all_env(void);

uint32_t usart6_rx_callback(uint8_t *buff, uint16_t len);


#ifdef __cplusplus
}
#endif


#endif // __INIT_H__
