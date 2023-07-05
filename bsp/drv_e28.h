#ifndef __DRV_E28_H__
#define __DRV_E28_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include "fifo.h"
#include "fsm.h"
		
typedef enum
{
    E28_HEAD_SAVE			= 0xc0,						//�����õĲ�������籣�档
    E28_HEAD_WITHOUTSAVE 	= 0xc2						//�����õĲ���������籣�档
} e28_head_type;						//֡ͷ����

typedef enum
{
    E28_PARITY_8N1 = 0,
    E28_PARITY_8O1 = 1,
    E28_PARITY_8E1 = 2
} e28_parity_type;					//����У������

typedef enum
{
    E28_TTL_BPS_1200  	= 0,
    E28_TTL_BPS_4800 	= 1,
    E28_TTL_BPS_9600 	= 2,
    E28_TTL_BPS_19200 	= 3,
    E28_TTL_BPS_57600 	= 4,
    E28_TTL_BPS_115200 	= 5,
    E28_TTL_BPS_460800 	= 6,
    E28_TTL_BPS_921600 	= 7
} e28_ttl_speed;					//TTL�������ʣ�bps��

typedef enum
{
    E28_AIR_BPS_AUTO  	= 0,									//������������Ӧ���������䣩
    E28_AIR_BPS_1K 		= 1,
    E28_AIR_BPS_5K 		= 2,
    E28_AIR_BPS_10K 	= 3,
    E28_AIR_BPS_50K 	= 4,
    E28_AIR_BPS_100K 	= 5,
    E28_AIR_BPS_1M	 	= 6,									//��������Ϊ 1M��FLRC��
    E28_AIR_BPS_2M	 	= 7										//��������Ϊ 2M��FSK��
} e28_air_speed;					//���߿������ʣ�bps��

typedef enum
{
    E28_FPT_DISABLE = 0,
    E28_FPT_ENABLE 	= 1
} e28_fpt_type;						//���㷢��ʹ��

typedef enum
{
    E28_LBT_DISABLE = 0,
    E28_LBT_ENABLE 	= 1
} e28_lbt_type;						//LBT����,�� LBT �󣬷���ÿ������֮ǰ���鵱ǰ�ŵ�����������Ϻã�ֱ�ӷ��ͣ��ŵ����ڸ�����ȴ�������ʧ���ٷ��͡�115200 ������������֧�֣����������Ӱ���������ܡ�

typedef enum
{
    E28_IO_OUTPUT_OD 	= 0,				//TXD��AUX ��·�����RXD ��·����
    E28_IO_OUTPUT_PP 	= 1					//TXD��AUX ���������RXD �������루Ĭ�ϣ�
} e28_io_type;						//IO ������ʽ

typedef enum
{
    E28_POWER_27DBM = 0,
    E28_POWER_23DBM = 1,
    E28_POWER_20DBM = 2,
    E28_POWER_17DBM = 3
} e28_power_type;						//���书��(��Լֵ)


typedef enum
{
    E28_WORKINGMODE_TRANS 	= 0,
    E28_WORKINGMODE_RSSI 	= 1,
    E28_WORKINGMODE_RANGE 	= 2,
    E28_WORKINGMODE_CONFIG 	= 3,
    E28_WORKINGMODE_SLEEP 	= 4,
} e28_working_mode;						//����ģʽ

typedef struct				
{
	e28_head_type 		head;
	uint16_t			addr;
	e28_parity_type 	par_type;
	e28_ttl_speed 		ttl_bps;
	e28_air_speed		air_bps;
	e28_fpt_type 		fpt_type;
	e28_lbt_type 		lbt_type;
	e28_io_type			io_type;
	e28_power_type 		power_type;
	uint8_t				chan;
}e28_parm_TypeDef;


typedef struct e28_parm_pack						//�������Ϊ1�ֽڣ�1�ֽڶ��롣
{
	uint8_t head;
	uint8_t addr8h;
	uint8_t addr8l;
    union
    {
        uint8_t byte;
        struct
        {
            uint8_t  air_bps 	: 3;				//��λ   	���߿������ʣ�bps��
            uint8_t  ttl_bps 	: 3;				//TTL 		��������(bps)
            uint8_t  parity 	: 2;				//��λ		����У��λ
        }bit;
    }sped;
		
	uint8_t chan;
		
    union
    {
        uint8_t byte;
        struct
        {
            uint8_t  	power 		: 2;			//���书��(��Լֵ)
            uint8_t  	io_mode 	: 1;			//IO������ʽ
            uint8_t  	lbt_en 		: 1;			//LBT����
            uint8_t  	reserved4_6 : 3;			//����
            uint8_t  	fpt_en 		: 1;			//���㷢��ʹ��λ
        }bit;
    }option;
}e28_parm_pack_t,*e28_parm_pack_p;



typedef struct
{
    void (*mode_set_f)(e28_working_mode);  
    void (*uart_deinit_f)(void);  
    void (*uart_reinit_f)(uint32_t);  
    void (*uart_send_f)(uint8_t *, uint16_t);  
	
	uint8_t dev_type;
	e28_parm_pack_t e28_parm_pack;

	uint8_t rx_updata;
	uint16_t rx_len;
	uint8_t *rx_buf;
	
	e28_working_mode mode;

	/* ״̬����� */
	FSM_T fsm;	
	fifo_s_t 	event_fifo;                					/* �¼��������� */
	uint8_t 	event_fifo_buffer[8];						/* �¼��������� */
	void (*fsm_init_f)(void* this_p, FsmTable_T*, uint16_t, uint8_t);
	void (*fsm_eventHandle_f)(void* this_p);
	void (*fsm_eventUpdate_f)(void* this_p, uint8_t);
	
} e28_manage_obj_t, *e28_manage_obj_p;



void e28_debug(e28_manage_obj_t* e28);


void e28_manage_init(e28_manage_obj_p e28, uint8_t dev_type);
void e28_init(e28_manage_obj_p e28, uint16_t addr, uint8_t chan);
void e28_set_rx_state(e28_manage_obj_t* e28_obj);
uint8_t e28_parm_config(e28_manage_obj_t* e28_obj, e28_parm_TypeDef* e28_parm_InitStruct);


#ifdef __cplusplus
}
#endif


#endif // __DRV_E28_H__
