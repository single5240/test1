#include "radio_task.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"

#include "drv_uart.h"
#include "init.h"
#include "fsm.h"
#include "fifo.h"
#include "drv_e28.h"
#include "mf_crc.h"
#include "event_mgr.h"
#include "easyflash.h"

#define LOG_TAG "radio_task"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

#define RC_DEV_TYPE RADIO_RECEIVER

extern RNG_HandleTypeDef hrng;
extern publisher_t radio_rx_Pub;
extern publisher_t rc_data_Pub;

subscriber_t radio_rx_Sub;
subscriber_t io_data_Subs;

e28_manage_obj_t e28_manage_obj = {0};

rc_data_pack_t rc_pack;
rc_match_pack_t rc_match;

static env_pack_p rc_env_s;


static void fsm_rc_data_send_callback(void *parm);
static void rc_match_init(rc_match_pack_p match_pack);
static uint8_t check_match_pack(uint8_t* buf, uint16_t len, rc_match_pack_p my_pack);
static uint8_t match_pack_verify(uint8_t* buf, uint16_t len);
static void rc_match_env_updata(env_pack_p env);
static uint8_t rc_match_pack_fill(uint8_t* buf, uint16_t len, rc_match_pack_p my_pack);



/** @enum radio_fsm_state_e
 *  @brief 状态机运行状态
 *  
 */
static enum
{
    STATE_UNINITIALIZED = 0x00, 		// 
		
	STATE_INITIALIZING,
	STATE_INITIALIZED_SLEEP,

	STATE_RC_PACK_SENDING,

	STATE_RC_MATCH_PREPARE,
	STATE_RC_MATCHING,
	
	STATE_DEBUG,
		
		
}fsm_state_e;



/** @enum radio_trig_event_e
 *  @brief 状态机触发事件
 *  
 */
static enum
{       
    EVENT_INITIALIZE_START,   	// 
    EVENT_INITIALIZE_FINISH,   	// 
				
	EVENT_RC_MATCH_START,
	EVENT_RC_MATCH_READY,
	EVENT_RC_MATCH_FINISH,
	
	EVENT_RC_DATA_SEND,
	
	EVENT_DEBUG,

}trig_event_e;



/* 动作函数 */
static void fsm_initialize_start_callback(void *parm);
static void fsm_initialize_finish_callback(void *parm);
static void fsm_rc_match_start_callback(void *parm);
static void fsm_rc_match_ready_callback(void *parm);
static void fsm_rc_match_finish_callback(void *parm);
static void fsm_rc_data_send_callback(void *parm);
static void fsm_debug_callback(void *parm);

/* 状态函数 */
static void state_uninitialized_proccess(e28_manage_obj_p e28);
static void state_initializing_proccess(e28_manage_obj_p e28);
static void state_initialized_sleep_proccess(e28_manage_obj_p e28);
static void state_rc_data_sending_proccess(e28_manage_obj_p e28);
static void state_rc_match_prepare_proccess(e28_manage_obj_p e28);
static void state_rc_matching_proccess(e28_manage_obj_p e28);

/* 状态迁移表 */
static FsmTable_T fsm_table[] = 
{
    /* 触发事件         							初态            				动作函数             					次态  	*/	
	{EVENT_INITIALIZE_START,     		STATE_UNINITIALIZED,     		fsm_initialize_start_callback,       	STATE_INITIALIZING},
	{EVENT_INITIALIZE_FINISH,     		STATE_INITIALIZING,     		fsm_initialize_finish_callback,       	STATE_INITIALIZED_SLEEP},

	
	{EVENT_RC_DATA_SEND,     			STATE_INITIALIZED_SLEEP,    	fsm_rc_data_send_callback,       		STATE_RC_PACK_SENDING},
	{EVENT_RC_DATA_SEND,     			STATE_RC_PACK_SENDING,    		fsm_rc_data_send_callback,       		STATE_RC_PACK_SENDING},
	

	{EVENT_RC_MATCH_START,     			STATE_RC_PACK_SENDING,    		fsm_rc_match_start_callback,       		STATE_RC_MATCH_PREPARE},
	{EVENT_RC_MATCH_START,     			STATE_INITIALIZED_SLEEP,    	fsm_rc_match_start_callback,       		STATE_RC_MATCH_PREPARE},
		
	{EVENT_RC_MATCH_READY,     			STATE_RC_MATCH_PREPARE,    		fsm_rc_match_ready_callback,       		STATE_RC_MATCHING},
	
	{EVENT_RC_MATCH_FINISH,     		STATE_RC_MATCHING,    			fsm_rc_match_finish_callback,       	STATE_INITIALIZED_SLEEP},

	{EVENT_DEBUG,     					STATE_RC_MATCHING,    			fsm_debug_callback,       				STATE_DEBUG},
	
};

/* 计算状态迁移表长度 */
static uint16_t fsm_table_lenth = sizeof(fsm_table)/sizeof(FsmTable_T);


/* 状态函数处理 */
static void fsm_stateHandle(e28_manage_obj_p e28) 
{
	switch(e28->fsm.curState)
	{
		case STATE_UNINITIALIZED:		
			state_uninitialized_proccess(e28);
			break;		

		case STATE_INITIALIZING:		
			state_initializing_proccess(e28);
			break;		

		case STATE_INITIALIZED_SLEEP:		
			state_initialized_sleep_proccess(e28);
			break;		
			
		case STATE_RC_PACK_SENDING:		
			state_rc_data_sending_proccess(e28);
			break;		
		
		case STATE_RC_MATCH_PREPARE:		
			state_rc_match_prepare_proccess(e28);
			break;		
		
		case STATE_RC_MATCHING:		
			state_rc_matching_proccess(e28);
			break;		
	
		default:
		break;
	}
}

/****************************** 事件触发函数 ******************************/

void rc_auto_match_start(void)
{
	if(e28_manage_obj.fsm_eventUpdate_f != NULL)
		e28_manage_obj.fsm_eventUpdate_f(&e28_manage_obj,EVENT_RC_MATCH_START);
}


/******************************** 动作函数 ********************************/
static void fsm_initialize_start_callback(void *parm)
{
	log_i("catch event: EVENT_INITIALIZE_START");
}

static void fsm_initialize_finish_callback(void *parm)
{
	log_i("catch event: EVENT_INITIALIZE_FINISH");
}

static void fsm_rc_match_start_callback(void *parm)
{
	log_i("catch event: EVENT_RC_MATCH_START");
}

static void fsm_rc_match_ready_callback(void *parm)
{
	log_i("catch event: EVENT_RC_MATCH_READY");
}

static void fsm_rc_match_finish_callback(void *parm)
{
	log_i("catch event: EVENT_RC_MATCH_FINISH");
}

static void fsm_rc_data_send_callback(void *parm)
{
	log_i("catch event: EVENT_RC_DATA_SEND");
}

static void fsm_debug_callback(void *parm)
{
	log_i("catch event: EVENT_DEBUG");
}


/******************************** 状态函数 ********************************/
static void state_uninitialized_proccess(e28_manage_obj_p e28)
{
	e28->fsm_eventUpdate_f(e28,EVENT_INITIALIZE_START);
}

static void state_initializing_proccess(e28_manage_obj_p e28)
{
	
	e28_init(&e28_manage_obj,rc_env_s->radio.addr,rc_env_s->radio.chan);
	if(e28->fsm_eventUpdate_f != NULL)
		e28->fsm_eventUpdate_f(e28,EVENT_INITIALIZE_FINISH);
}

static void state_initialized_sleep_proccess(e28_manage_obj_p e28)
{
	if(e28->fsm_eventUpdate_f != NULL)
		e28->fsm_eventUpdate_f(e28,EVENT_RC_DATA_SEND);
}



static void state_rc_data_sending_proccess(e28_manage_obj_p e28)
{
	
//	EventMsgGetLast(&io_data_Subs, IO_DATA_MSG, &io_data, NULL);
//	io_data_to_rc_pack(&io_data,&rc_pack,rc_env_s->radio.match_key);

//	if(e28_manage_obj.uart_send_f != NULL)
//		e28_manage_obj.uart_send_f((uint8_t*)&rc_pack,sizeof(rc_data_pack_t));
	
	
}

static void state_rc_match_prepare_proccess(e28_manage_obj_p e28)
{	
	if(e28->dev_type == RADIO_CONTROLLER)
	{	
		e28_init(&e28_manage_obj,0xffff,rc_env_s->radio.chan);
		
		rc_match_init(&rc_match);
		
		if(e28->fsm_eventUpdate_f != NULL)
			e28->fsm_eventUpdate_f(e28,EVENT_RC_MATCH_READY);
	}
	else if(e28->dev_type == RADIO_RECEIVER)
	{
		e28_init(&e28_manage_obj,0xffff,rc_env_s->radio.chan);
		
		if(e28->fsm_eventUpdate_f != NULL)
			e28->fsm_eventUpdate_f(e28,EVENT_RC_MATCH_READY);
	}
}

	
static void state_rc_matching_proccess(e28_manage_obj_p e28)
{
	uint8_t buf[RADIO_RX_MSG_LEN];
	int i = 0;
	if(e28->dev_type == RADIO_CONTROLLER)
	{
		if(e28_manage_obj.uart_send_f != NULL)
			e28_manage_obj.uart_send_f((uint8_t*)&rc_match,sizeof(rc_match_pack_t));
		osDelay(50);
		EventMsgGetLast(&radio_rx_Sub, RADIO_RX_MSG, buf, NULL);
		
		if(check_match_pack(buf,sizeof(rc_match_pack_t),&rc_match) == 1)
		{
			log_i("check_match_pack success!");
			rc_env_s->radio.addr = rc_match.addr.ad16;						//更新环境变量，保存对频结果
			rc_env_s->radio.match_key = rc_match.match_key;
			rc_env_s->radio.chan = rc_match.chan;
			rc_match_env_updata(rc_env_s);
			
			e28_init(&e28_manage_obj,rc_env_s->radio.addr,rc_env_s->radio.chan);
			e28->fsm_eventUpdate_f(e28,EVENT_RC_MATCH_FINISH);
		}
//		else
//			log_i("check_match_pack fail!");
	}
	else if(e28->dev_type == RADIO_RECEIVER)
	{
		EventMsgGetLast(&radio_rx_Sub, RADIO_RX_MSG, buf, NULL);
		
		if(match_pack_verify(buf,sizeof(rc_match_pack_t)) == 1)
		{
			log_i("match_pack verify success!");
			rc_match_pack_fill(buf, sizeof(rc_match_pack_t), &rc_match);
			rc_env_s->radio.addr = rc_match.addr.ad16;						//更新环境变量，保存对频结果
			rc_env_s->radio.match_key = rc_match.match_key;
			rc_env_s->radio.chan = rc_match.chan;
			rc_match_env_updata(rc_env_s);
			
			printf_all_env();
			for(i = 0; i < 40; i++)
			{
				if(e28_manage_obj.uart_send_f != NULL)
					e28_manage_obj.uart_send_f((uint8_t*)&rc_match,sizeof(rc_match_pack_t));
				osDelay(50);
			}
			e28_init(&e28_manage_obj,rc_env_s->radio.addr,rc_env_s->radio.chan);
			e28->fsm_eventUpdate_f(e28,EVENT_RC_MATCH_FINISH);
		}
//		else
//			log_i("check_match_pack fail!");
		
	}

//	uint8_t buf[RADIO_RX_MSG_LEN];
//	int8_t e28_rssi;
//	
//	e28_debug(e28);
//	
//	if(e28->mode == E28_WORKINGMODE_RSSI)
//	{
//		EventMsgGetLast(&radio_rx_Sub, RADIO_RX_MSG, buf, NULL);
//		e28_rssi = buf[0];
//		log_i("e28_rssi = %d , hex = %x",e28_rssi,buf[0]);
//		
//	}
	
}




uint32_t e28_rx_callback(uint8_t *buff, uint16_t len)
{
//    shell_interupt(buff, len);
	
//		__log_output(buff, len);
//		for(int i = 0; i < len; i++)
//			log_i("e28_rx_buf[%d]=%x\r\n",i,*(buff + i));
	
//		log_i("e28_rx_buf= %x %x %x %x %x %x\r\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);
	
	e28_manage_obj.rx_buf = buff;
	e28_manage_obj.rx_len = len;
	e28_set_rx_state(&e28_manage_obj);
	
	EventMsgPost(&radio_rx_Pub, e28_manage_obj.rx_buf,0);
	
    return 0;
}




void  debug_1(void)
{
	rc_data_pack_t rc_data1,rc_data2;
	log_i("the size of rc_data_pack_t = %d bytes",sizeof(rc_data_pack_t));
	log_i("the size of rc_data = %d bytes",sizeof(rc_data1));

	rc_data1.seg2_32b.ch1_joy_x = 2011;

	memcpy(&rc_data2, &rc_data1,sizeof(rc_data_pack_t));

	log_i("rc_data2.seg2_32b.ch1_joy_x = %d ",rc_data2.seg2_32b.ch1_joy_x);
}

void* get_e28_obj(void)
{
	return &e28_manage_obj;
}


void Radio_Task(void const * argument)
{
	
	UNUSED(fsm_state_e);
	UNUSED(trig_event_e);
	
	rc_env_s = (env_pack_p)get_rc_env();

	e28_manage_init(&e28_manage_obj,RC_DEV_TYPE);
	e28_manage_obj.fsm_init_f(&e28_manage_obj,fsm_table,fsm_table_lenth,STATE_UNINITIALIZED);


	log_i("Radio_Task_launch!");
	
//		debug_1();

//	EventSubscribeInit(&io_data_Subs, SUBS_MODE_NOLIST);
//	EventSubscribe(&io_data_Subs, IO_DATA_MSG, IO_DATA_MSG_LEN, 0, NULL);

	EventSubscribeInit(&radio_rx_Sub, SUBS_MODE_NOLIST);
	EventSubscribe(&radio_rx_Sub, RADIO_RX_MSG, RADIO_RX_MSG_LEN, 0, NULL);

	while(1)
	{			
	
		fsm_stateHandle(&e28_manage_obj);
		
		osDelay(10);	
	}
}

static uint8_t match_pack_verify(uint8_t* buf, uint16_t len)
{
	rc_match_pack_p rx_pack = (rc_match_pack_p)buf;
	
	if(buf[0] == RADIO_MATCH_PACK_HEAD && verify_crc16(buf,len))
	{
		if(rx_pack->dey_type != RADIO_CONTROLLER)
			return 0;
	}
	else
	{
//		log_i("head = %x",buf[0]);
		return 0;
	}
	
	return 1;
}




static uint8_t check_match_pack(uint8_t* buf, uint16_t len, rc_match_pack_p my_pack)
{
	rc_match_pack_p rx_pack = (rc_match_pack_p)buf;
	
	if(buf[0] == RADIO_MATCH_PACK_HEAD && verify_crc16(buf,len))
	{
		if(rx_pack->addr.ad16 != my_pack->addr.ad16)
			return 0;
		if(rx_pack->chan != my_pack->chan)
			return 0;
		if(rx_pack->match_key != my_pack->match_key)
			return 0;
		if(rx_pack->dey_type == my_pack->dey_type)
			return 0;
	}
	else
		return 0;
	
	return 1;
}

static void rc_match_init(rc_match_pack_p match_pack)
{
	match_pack->head = RADIO_MATCH_PACK_HEAD;
	match_pack->addr.ad16 = (uint16_t)HAL_RNG_GetRandomNumber(&hrng);		//获取随机数
	match_pack->dey_type = RC_DEV_TYPE;
	match_pack->chan = rc_env_s->radio.chan;
	match_pack->match_key = (uint16_t)HAL_RNG_GetRandomNumber(&hrng);		//获取随机数
	append_crc16((uint8_t*)match_pack,sizeof(rc_match_pack_t));
	
}

static void rc_match_env_updata(env_pack_p env)
{
	ef_set_env_blob(ENV_RADIO_ADDR,&env->radio.addr,sizeof(env->radio.addr));
	ef_set_env_blob(ENV_RADIO_MATCH_KEY,&env->radio.match_key,sizeof(env->radio.match_key));
	ef_set_env_blob(ENV_RADIO_CHAN,&env->radio.chan,sizeof(env->radio.chan));
}

static uint8_t rc_match_pack_fill(uint8_t* buf, uint16_t len, rc_match_pack_p my_pack)
{	
	memcpy(my_pack,buf,sizeof(rc_match_pack_t));
	
	my_pack->dey_type = RC_DEV_TYPE;
	append_crc16((uint8_t*)my_pack,sizeof(rc_match_pack_t));
	
//	log_i("head = %x",my_pack->head);
	return 1;
}


void radio_data_unpack(uint8_t *buff, uint16_t len)
{
	rc_data_pack_p pack_p = (rc_data_pack_p)buff;

	if(len == sizeof(rc_data_pack_t) && verify_crc16(buff,len))
	{
		memcpy(&rc_pack,pack_p,sizeof(rc_data_pack_t));
//				log_i("unpack_success!");
	
		EventMsgPost(&rc_data_Pub,buff,RC_DATA_MSG_LEN);
	
		if(rc_pack.seg1_32b.an1_key == 1)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);
		else	if(rc_pack.seg1_32b.an2_key == 1)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);				
		else if(rc_pack.seg1_32b.an3_key == 1)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);
		else if(rc_pack.seg1_32b.stop_key == 1)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);
		else if(rc_pack.seg1_32b.xn2_key != 3)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);				
		else if(abs(rc_pack.seg2_32b.ch1_joy_x - 1024) > 100 )
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);				
		else if(abs(rc_pack.seg2_32b.ch2_joy_y - 1024) > 100 )
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);			
		else if(rc_pack.seg3_32b.xn1_key != 3)
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_RESET);
		else
				HAL_GPIO_WritePin(BLINK_LED_GPIO_Port,BLINK_LED_Pin, GPIO_PIN_SET); 
			
	}
	else
		log_i("error_pack!");

}
