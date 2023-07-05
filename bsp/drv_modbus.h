#ifndef __DRV_MODBUS_H__
#define __DRV_MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "drv_led.h"
#include "fsm.h"
#include "fifo.h"


#define MODBUS_INFO_PRINTF_SET 0
#define MODBUS_ERROR_PRINTF_SET 1


#define modbus_log_d(...)  log_d(__VA_ARGS__);
#define modbus_log_i(...)  log_i(__VA_ARGS__);
#define modbus_log_e(...)  log_e(__VA_ARGS__);


#if (MODBUS_INFO_PRINTF_SET)
    #ifndef MODBUS_INFO_PRINTF
        #define MODBUS_INFO_PRINTF(...) modbus_log_i(__VA_ARGS__);
    #endif
#else
    #ifndef MODBUS_INFO_PRINTF
        #define MODBUS_INFO_PRINTF(...)
    #endif
#endif

#if (MODBUS_ERROR_PRINTF_SET)
    #ifndef MODBUS_ERROR_PRINTF
        #define MODBUS_ERROR_PRINTF(...) modbus_log_e(__VA_ARGS__);
    #endif
#else
    #ifndef MODBUS_ERROR_PRINTF
        #define MODBUS_ERROR_PRINTF(...)
    #endif
#endif

	
typedef void (*unpack_success_callback)(void *,u8* , uint16_t);
	
	
	
	
#define G100_ADDRESS		0x03



#define G100_REG_RAM				0x8000
#define G100_REG_P00_01			0x0001			//命令源选择	0：键盘		1：端子		2：通讯
#define G100_REG_P00_02			0x0002			//主频率源X选择	7：通讯
#define G100_REG_P00_07			0x0007			//主频率源叠加选择	0：主频率源X
#define G100_REG_P00_08			0x0008			//预设频率

#define G100_REG_PD1_00			0xd100			//通信设定值		10000对应100%

#define G100_REG_PD2_00			0xd200			//通讯命令控制		0001：正转运行		0002：反转运行		0003：正转点动		0004：反转点动		0005：自由停机		0006：减速停机		0007：故障复位		0008：调谐启动



typedef enum
{
		MBR_DEV_SD710 = 1,
		MBR_DEV_HS100,
		MBR_DEV_G100,
} mbr_dev_type_e;

typedef enum
{
		MBR_DEV_SPEED_MODE = 1,
		MBR_DEV_POS_MODE,
		MBR_DEV_TORQUE_MODE,
} mbr_dev_mode_e;



typedef void (*usart_dma_send_start_fn)(int16_t length);
typedef struct _modbus_rtu_t modbus_rtu_t,*modbus_rtu_p;

typedef struct _modbus_rtu_t {
	
		/* modbus 驱动层相关 */
    void (*usart_send_f)(uint8_t *, uint16_t);
    unpack_success_callback unpack_success_callback_f;
    void (*unpack_03h_callback_f)(void* this_p, uint8_t*);
		uint8_t*				obj_name;
		uint8_t 				obj_name_len;
		u8   						addr;
		u16	 						reg;
		volatile u8 	 	ack;
		u8   						cmd;
		u8 	 						length;
	
		/* RJ45 LED */
		led_s_obj_t led_L;
		led_s_obj_t led_R;
		uint16_t led_process_cnt;
	
		/* 状态机相关 */
		FSM_T fsm;	
    fifo_s_t 	event_fifo;                			/* 事件触发队列 */
		uint8_t 	event_fifo_buffer[8];						/* 事件触发缓存 */
    void (*fsm_init_f)(void* this_p, FsmTable_T*, uint16_t, uint8_t);
    void (*fsm_eventHandle_f)(void* this_p);
    void (*fsm_eventUpdate_f)(void* this_p, uint8_t);

		/* 设备相关 */
    uint8_t (*dev_03h_unpack_f)(modbus_rtu_p, uint8_t* );				/* 特定设备 modbus rtu 03h 协议解析函数 */
    uint8_t (*dev_pos_cfg_f)(modbus_rtu_p );											/* 伺服驱动器位置模式配置函数 */
    uint8_t (*dev_speed_cfg_f)(modbus_rtu_p );										/* 伺服驱动器速度模式配置函数 */
    uint8_t (*dev_torque_cfg_f)(modbus_rtu_p );									/* 伺服驱动器力矩模式配置函数 */
    uint8_t (*dev_output_enable_f)(modbus_rtu_p );								/* 驱动器使能函数 */
    uint8_t (*dev_speed_set_f)(modbus_rtu_p ,int16_t );					/* 驱动器速度设定函数 */
    uint8_t (*dev_pos_set_f)(modbus_rtu_p ,int32_t );						/* 驱动器位置设定函数 */
    uint8_t (*dev_torque_set_f)(modbus_rtu_p ,int16_t );					/* 驱动器力矩设定函数 */
    uint8_t (*dev_read_pos_f)(modbus_rtu_p );						/* 驱动器位置读取函数 */
    uint8_t (*dev_read_test_f)(modbus_rtu_p );						/* 驱动器位置读取函数 */

		uint8_t dev_type;
		uint8_t  dev_mode;
		uint8_t  dev_addr;
		int64_t dev_pos_s;
		uint8_t dev_mbr_delay;
		
			
} modbus_rtu_t,*modbus_rtu_p;






typedef struct _hs100servo_t {
	
		int32_t Absolute_position_feedback;
		uint16_t posmode_speed;	
	
		u8	mode;
	
} hs100servo_t;


typedef struct _g100_converter_t {
	
		uint16_t set_frequency;	
		u8	cmd_cfg;
	
} g100_converter_t;





uint8_t MBR_ReadParam_03H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _num);
uint8_t MBR_WriteParam_06H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _value);
uint8_t MBR_WriteParam_10H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf);

void MBR_AnalyzeApp(modbus_rtu_t *mbr,u8* buff, uint16_t rec_length);

void MBR_Read_03H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _num);
void MBR_Write_06H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _value);
void MBR_Write_10H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf);

void mbr_fsm_config(modbus_rtu_p mbr);
void mbr_up_ok_callback_register(modbus_rtu_t *m_obj, unpack_success_callback fun);

	
//G100变频器配置，内含阻塞操作，只能在初始化阶段调用
void G100_Converter_Config(modbus_rtu_t *mbr,uint8_t addr, int16_t Frequency);


void modbus_usart3_rx_callback(uint8_t *buff, uint16_t len);

void mbr_led_process(modbus_rtu_p mbr);
void mbr_dev_mode_set(modbus_rtu_p mbr,uint8_t mode);

	

#ifdef __cplusplus
}
#endif


#endif // __DRV_MODBUS_H__
