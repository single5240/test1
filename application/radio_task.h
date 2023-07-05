#ifndef __RADIO_TASK_H__
#define __RADIO_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "proj_config.h"

//*************************radio version v1.1*************************
	
typedef enum
{
    RADIO_NORMAL_DATA_PACK_HEAD = 0x60,
    RADIO_MATCH_PACK_HEAD,
} radio_head_type;			

typedef enum
{
    RADIO_CONTROLLER = 0,
    RADIO_RECEIVER,
} radio_dev_type;			

typedef struct rc_data_pack						//�������Ϊ1�ֽڣ�1�ֽڶ��롣
{
	struct
	{		
		uint32_t  head 			: 8;				//��λ
	
		uint32_t  xn2_key 	: 2;				//
		uint32_t  stop_key 	: 1;				//
		uint32_t  an1_key 	: 1;				//
		uint32_t  an2_key 	: 1;				//
		uint32_t  an3_key 	: 1;				//
		uint32_t  V_key1 		: 1;				//				
		uint32_t  V_key2 		: 1;				//				total: 2byte
	
		uint32_t  reseved 	: 16;				//				total: 4byte
	} seg1_32b;

	struct
	{
		uint32_t  ch1_joy_x : 11;				//��λ   	
		uint32_t  ch2_joy_y : 11;				//
		uint32_t  xn1_an 		: 10;				//��λ			total: 4byte
	} seg2_32b;

	struct
	{
		uint32_t  xn2_an 		: 10;				//��λ
		uint32_t  xn3_an 		: 10;				//
		uint32_t  xn4_an 		: 10;				//
		uint32_t  xn1_key 	: 2;				//					total: 4byte
	} seg3_32b;

	struct
	{
		uint32_t  match_key		: 16;				//��λ
		uint32_t  crcH			: 8;				//
		uint32_t  crcL 			: 8;				//					total: 4byte
	} seg4_32b;

}rc_data_pack_t,*rc_data_pack_p;


#pragma pack(push,1)		//���������Լ��Ķ��뷽ʽ��ѹ��ջ���������������߱���������ľͰ���1�ֽڶ���
typedef struct rc_match_pack						
{
	uint8_t head;
	
    union
    {
        uint16_t ad16;
        struct
        {
            uint8_t  ad8l 	: 8;				//��λ   	���߿������ʣ�bps��
            uint8_t  ad8h 	: 8;				//TTL 		��������(bps)
        }bit;
    }addr;
	
	uint8_t chan;
	uint8_t dey_type;
	
	int16_t match_key;
	
	uint8_t  crcH;			
	uint8_t  crcL; 			
	
}rc_match_pack_t,*rc_match_pack_p;
#pragma pack(pop)				//)��ԭ���Ķ��뷽ʽ���ͷų�����




//**********************************************************************
	
void Radio_Task(void const * argument);
uint32_t e28_rx_callback(uint8_t *buff, uint16_t len);
void* get_e28_obj                  (void);



/****************************** �¼��������� ******************************/
void rc_auto_match_start(void);



#ifdef __cplusplus
}
#endif

#endif // __RADIO_TASK_H__
