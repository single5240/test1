#include "modbus2_task.h"

#include "cmsis_os.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "drv_e28.h"
#include "drv_modbus.h"
#include "event_mgr.h"
#include "sd710.h"
#include "hs100.h"

#define LOG_TAG "modbus2_task"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

modbus_rtu_t mbr_no3;
static rc_data_pack_t modbus_rc_data;
	

/** @enum radio_fsm_state_e
 *  @brief 状态机运行状态
 *  
 */
static enum
{
    STATE_DISCONNECT = 0x00, 		// 
	
    STATE_CONNECTING,           // 
    STATE_CONNECTED_SLEEP,      // 
	
    STATE_DURING_CONFIG,      			// 
    STATE_CONFIGED_SLEEP,      	// 
	
    STATE_DURING_ENABLE,      	// 
    STATE_RUNNING,      	// 
	
}fsm_state_e;



/** @enum radio_trig_event_e
 *  @brief 状态机触发事件
 *  
 */
static enum
{       
    EVENT_CONNECT_START,   	// 
    EVENT_CONNECT_FINISH,    // 
	
		EVENT_CONFIG_START,				// 
		EVENT_CONFIG_FINISH,				// 
	
		EVENT_ENABLE_START,				// 
		EVENT_ENABLE_FINISH,				// 
	
		EVENT_RECONFIG_SPEED_MODE,
		EVENT_RECONFIG_POS_MODE,
		EVENT_RECONFIG_TORQUE_MODE,
	
		EVENT_CONNECT_LOST,				// 
	
}trig_event_e;



/* 动作函数 */
static void fsm_connect_start_callback(void *parm);
static void fsm_connect_finish_callback(void *parm);
static void fsm_connect_lost_callback(void *parm);
static void fsm_config_start_callback(void *parm);
static void fsm_config_finish_callback(void *parm);
static void fsm_enable_start_callback(void *parm);
static void fsm_enable_finish_callback(void *parm);
static void fsm_reconfig_speed_mode_callback(void *parm);


/* 状态函数 */
static void state_disconnect_proccess(modbus_rtu_p mbr);
static void state_connecting_proccess(modbus_rtu_p mbr);
static void state_connected_sleep_proccess(modbus_rtu_p mbr);
static void state_duringconfig_proccess(modbus_rtu_p mbr);
static void state_configed_sleep_proccess(modbus_rtu_p mbr);
static void state_during_enable_proccess(modbus_rtu_p mbr);
static void state_running_proccess(modbus_rtu_p mbr);


/* 状态迁移表 */
static FsmTable_T fsm_table[] = 
{
    /* 触发事件         							初态            								动作函数             							次态  	*/	
	{EVENT_CONNECT_START,     		STATE_DISCONNECT,     		fsm_connect_start_callback,       	STATE_CONNECTING},
	{EVENT_CONNECT_FINISH,    		STATE_CONNECTING,      		fsm_connect_finish_callback,     		STATE_CONNECTED_SLEEP},
	
	{EVENT_CONFIG_START,    			STATE_CONNECTED_SLEEP,      fsm_config_start_callback,     			STATE_DURING_CONFIG},
	{EVENT_CONFIG_FINISH,    			STATE_DURING_CONFIG,      	fsm_config_finish_callback,     		STATE_CONFIGED_SLEEP},
	
	{EVENT_ENABLE_START,    			STATE_CONFIGED_SLEEP,    	fsm_enable_start_callback,     			STATE_DURING_ENABLE},
	{EVENT_ENABLE_FINISH,    			STATE_DURING_ENABLE,    	fsm_enable_finish_callback,     			STATE_RUNNING},
	
	{EVENT_RECONFIG_SPEED_MODE,  STATE_CONFIGED_SLEEP,     	fsm_reconfig_speed_mode_callback,    STATE_DISCONNECT},
	{EVENT_RECONFIG_SPEED_MODE,  STATE_RUNNING,      				fsm_reconfig_speed_mode_callback,    STATE_DISCONNECT},
	
	{EVENT_CONNECT_LOST,    			STATE_CONNECTED_SLEEP,    fsm_connect_lost_callback,     			STATE_DISCONNECT},
	{EVENT_CONNECT_LOST,    			STATE_CONFIGED_SLEEP,    	fsm_connect_lost_callback,     			STATE_DISCONNECT},
	{EVENT_CONNECT_LOST,    			STATE_RUNNING,    				fsm_connect_lost_callback,     			STATE_DISCONNECT},
	
};

/* 计算状态迁移表长度 */
static uint16_t fsm_table_lenth = sizeof(fsm_table)/sizeof(FsmTable_T);


/* 状态函数处理 */
static void fsm_stateHandle(modbus_rtu_p mbr) 
{
		switch(mbr->fsm.curState)
		{
				case STATE_DISCONNECT:		
						state_disconnect_proccess(mbr);
						break;
				
				case STATE_CONNECTING:		
						state_connecting_proccess(mbr);
						break;
					
				case STATE_CONNECTED_SLEEP:		
						state_connected_sleep_proccess(mbr);
						break;
				
				case STATE_DURING_CONFIG:		
						state_duringconfig_proccess(mbr);
						break;
				
				case STATE_CONFIGED_SLEEP:		
						state_configed_sleep_proccess(mbr);
						break;
				
				case STATE_DURING_ENABLE:		
						state_during_enable_proccess(mbr);
						break;
				
				case STATE_RUNNING:		
						state_running_proccess(mbr);
						break;
				
				
				default:
				break;
		}
}

/****************************** 事件触发函数 ******************************/


/******************************** 动作函数 ********************************/
static void fsm_connect_start_callback(void *parm)
{
		log_i("catch event: EVENT_CONNECT_START");
}

static void fsm_connect_finish_callback(void *parm)
{
		log_i("catch event: EVENT_CONNECT_FINISH");
}

static void fsm_connect_lost_callback(void *parm)
{
		log_i("catch event: EVENT_CONNECT_LOST");
}

static void fsm_config_start_callback(void *parm)
{
		log_i("catch event: EVENT_CONFIG_START");
}

static void fsm_config_finish_callback(void *parm)
{
		log_i("catch event: EVENT_CONFIG_FINISH");
}

static void fsm_enable_start_callback(void *parm)
{
		log_i("catch event: EVENT_ENABLE_START");
}

static void fsm_enable_finish_callback(void *parm)
{
		log_i("catch event: EVENT_ENABLE_FINISH");
}

static void fsm_reconfig_speed_mode_callback(void *parm)
{
		mbr_dev_mode_set(&mbr_no3,MBR_DEV_SPEED_MODE);
		log_i("catch event: EVENT_RECONFIG_SPEED_MODE");
}

/******************************** 状态函数 ********************************/
static void state_disconnect_proccess(modbus_rtu_p mbr)
{
		mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_START);
}

static void state_connecting_proccess(modbus_rtu_p mbr)
{
		uint8_t pass = mbr->dev_read_test_f(mbr);

		if(pass == 1)
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_FINISH);
}

static void state_connected_sleep_proccess(modbus_rtu_p mbr)
{
		if(mbr->dev_read_pos_f(mbr) == 0)
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_LOST);
		else
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONFIG_START);
}

static void state_duringconfig_proccess(modbus_rtu_p mbr)
{		
		if(mbr->dev_mode == MBR_DEV_SPEED_MODE)
		{
				mbr->dev_speed_cfg_f(mbr);
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONFIG_FINISH);
		}
		else if(mbr->dev_mode == MBR_DEV_POS_MODE)
		{
				mbr->dev_pos_cfg_f(mbr);
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONFIG_FINISH);
		}
}

static void state_configed_sleep_proccess(modbus_rtu_p mbr)
{	
		if(mbr->dev_read_pos_f(mbr) == 0)
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_LOST);
		else
				mbr->fsm_eventUpdate_f(mbr,EVENT_ENABLE_START);
}

static void state_during_enable_proccess(modbus_rtu_p mbr)
{	
		if(mbr->dev_output_enable_f(mbr) == 1)
				mbr->fsm_eventUpdate_f(mbr,EVENT_ENABLE_FINISH);
	
}

static void state_running_proccess(modbus_rtu_p mbr)
{	
		if(mbr->dev_mode == MBR_DEV_SPEED_MODE)
		{
				int16_t speed_set = (modbus_rc_data.seg2_32b.ch2_joy_y - 1024);
				speed_set = LIMIT(speed_set,-3000,3000);

				if(mbr->dev_speed_set_f(mbr,speed_set) == 0)
						mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_LOST);
		}
		else if(mbr->dev_mode == MBR_DEV_POS_MODE)
		{
				int32_t pos_set = (modbus_rc_data.seg2_32b.ch2_joy_y - 1024) * 10;

				if(mbr->dev_pos_set_f(mbr,pos_set) == 0)
						mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_LOST);
		}
		
		osDelay(mbr_no3.dev_mbr_delay);
		
		if(mbr->dev_read_pos_f(mbr) == 0)
				mbr->fsm_eventUpdate_f(mbr,EVENT_CONNECT_LOST);
}



static void mbr_no3_unpack_ok_callback(void* this_p, uint8_t* buf, uint16_t len)
{
		modbus_rtu_p obj_p = this_p;
	
		if(++obj_p->led_L.led_cnt > 5)
		{
				obj_p->led_L.toggle_f(&obj_p->led_L);
				obj_p->led_L.led_cnt = 0;
		}
}


static void mbr_no3_init(uint8_t dev_type,uint8_t dev_addr ,uint8_t mbr_delay)
{
		uint8_t name_buf[] = "mbr_no3";
		mbr_no3.obj_name = name_buf;
		mbr_no3.obj_name_len = sizeof(name_buf);
	
		mbr_no3.usart_send_f = usart3_transmit;
		mbr_up_ok_callback_register(&mbr_no3,mbr_no3_unpack_ok_callback);
	
		mbr_no3.dev_type = dev_type;																			//modbus设备，华远、伟创具体设备
		mbr_no3.dev_addr = dev_addr;
		mbr_no3.dev_mbr_delay = mbr_delay;
	
		if(mbr_no3.dev_type == MBR_DEV_SD710)															//设备支持函数注册
		{
				mbr_no3.unpack_03h_callback_f = sd710_03h_unpack;									//modbus 03h 协议解包函数
				mbr_no3.dev_pos_cfg_f = sd710_pos_mode_config;
				mbr_no3.dev_pos_set_f = sd710_pos_set;
				mbr_no3.dev_speed_cfg_f = sd710_speed_mode_config;
				mbr_no3.dev_speed_set_f = sd710_speed_set;
				mbr_no3.dev_read_test_f = sd710_read_test;
				mbr_no3.dev_read_pos_f = sd710_read_pos;
				mbr_no3.dev_output_enable_f = sd710_output_enable;
				mbr_no3.dev_torque_cfg_f = NULL;
				mbr_no3.dev_torque_set_f = NULL;
			
				mbr_no3.dev_mode = MBR_DEV_POS_MODE;
		}
		else if(mbr_no3.dev_type == MBR_DEV_HS100)
		{
				mbr_no3.unpack_03h_callback_f = hs100_03h_unpack;									//modbus 03h 协议解包函数
				mbr_no3.dev_pos_cfg_f = hs100_pos_mode_config;
				mbr_no3.dev_pos_set_f = hs100_pos_set;
				mbr_no3.dev_speed_cfg_f = hs100_speed_mode_config;
				mbr_no3.dev_speed_set_f = hs100_speed_set;
				mbr_no3.dev_read_test_f = hs100_read_test;
				mbr_no3.dev_read_pos_f = hs100_read_pos;
				mbr_no3.dev_output_enable_f = hs100_output_enable;
				mbr_no3.dev_torque_cfg_f = hs100_torque_mode_config;
				mbr_no3.dev_torque_set_f = hs100_torque_set;
			
				mbr_no3.dev_mode = MBR_DEV_SPEED_MODE;
		}
			
	
		led_obj_create(&mbr_no3.led_L,RJ45_LED_L3_GPIO_Port,RJ45_LED_L3_Pin,1);
		led_obj_create(&mbr_no3.led_R,RJ45_LED_R3_GPIO_Port,RJ45_LED_R3_Pin,1);
		mbr_no3.led_L.set_led_f(&mbr_no3.led_L,0);
		mbr_no3.led_R.set_led_f(&mbr_no3.led_R,0);
	
		mbr_fsm_config(&mbr_no3);
}


void modbus_usart3_rx_callback(uint8_t *buff, uint16_t len)
{
		MBR_AnalyzeApp(&mbr_no3,buff,len);
	
		mbr_no3.led_R.set_led_f(&mbr_no3.led_R,1);
}


void* get_mbr_no3(void)
{
		return &mbr_no3;
}


void Modbus2_Task(void const * argument)
{	
		UNUSED(fsm_state_e);
		UNUSED(trig_event_e);
	
		mbr_no3_init(MBR_DEV_HS100,HS100_ADDRESS_1,5);
		mbr_no3.fsm_init_f(&mbr_no3,fsm_table,fsm_table_lenth,STATE_DISCONNECT);
	
    subscriber_t nolistSubs;
    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, RC_DATA_MSG, RC_DATA_MSG_LEN, 0, NULL);
		
		log_i("Modbus2_Task_launch!");

    uint32_t period = osKernelSysTick();
	
		while(1)
		{
        EventMsgGetLast(&nolistSubs, RC_DATA_MSG, &modbus_rc_data, NULL);
			
				mbr_led_process(&mbr_no3);
			
				fsm_stateHandle(&mbr_no3);
			
				osDelay(mbr_no3.dev_mbr_delay);
		}
}
