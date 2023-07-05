#include "drv_e28.h"

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "radio_task.h"

#define LOG_TAG "drv_e28"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

extern UART_HandleTypeDef huart6;


static void parm_pack(e28_parm_TypeDef* e28_parm_InitStruct, e28_parm_pack_t* e28_parm_pack);
uint8_t e28_parm_config(e28_manage_obj_t* e28_obj, e28_parm_TypeDef* e28_parm_InitStruct);

static void fsmEventHandle(void* this_p);
static void fsmUpdateEvent(void* this_p, uint8_t event);
static void fsm_init(void* this_p, FsmTable_T *pTable, uint16_t stuMaxNum, uint8_t curState);

static void e28_mode_set(e28_working_mode mode)
{
	switch(mode)
	{																	//    M2		M1		M0
		case E28_WORKINGMODE_TRANS:		//		1			0			0
			HAL_GPIO_WritePin(E28_M0_GPIO_Port, E28_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M1_GPIO_Port, E28_M1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M2_GPIO_Port, E28_M2_Pin, GPIO_PIN_SET);
			break;
		
		case E28_WORKINGMODE_RSSI:		//		1			0			1
			HAL_GPIO_WritePin(E28_M0_GPIO_Port, E28_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E28_M1_GPIO_Port, E28_M1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M2_GPIO_Port, E28_M2_Pin, GPIO_PIN_SET);
			break;
		
		case E28_WORKINGMODE_RANGE:		//		1			1			0
			HAL_GPIO_WritePin(E28_M0_GPIO_Port, E28_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M1_GPIO_Port, E28_M1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E28_M2_GPIO_Port, E28_M2_Pin, GPIO_PIN_SET);
			break;
		
		case E28_WORKINGMODE_CONFIG:	//		1			1			1
			HAL_GPIO_WritePin(E28_M0_GPIO_Port, E28_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E28_M1_GPIO_Port, E28_M1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(E28_M2_GPIO_Port, E28_M2_Pin, GPIO_PIN_SET);
			break;
		
		case E28_WORKINGMODE_SLEEP:		//		0			X			X
			HAL_GPIO_WritePin(E28_M0_GPIO_Port, E28_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M1_GPIO_Port, E28_M1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(E28_M2_GPIO_Port, E28_M2_Pin, GPIO_PIN_RESET);
			break;
	
		default:
				break;
	}
}



void e28_uart_deinit(void)
{
	usart6_manage_deinit();
	HAL_UART_DeInit(&huart6);
}

void e28_uart_reinit(uint32_t bps)
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = bps;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
			Error_Handler();
	}
		
    usart6_manage_init();
    usart6_rx_callback_register(usart6_rx_callback);
}

void e28_uart_send(uint8_t *buff, uint16_t len)
{
	usart6_transmit(buff,len);
}

void e28_delayms(uint32_t ms)
{
		osDelay(ms);
}

static void e28_fsm_config(e28_manage_obj_p e28)
{
	e28->fsm_init_f = fsm_init;
	e28->fsm_eventHandle_f = fsmEventHandle;
	e28->fsm_eventUpdate_f = fsmUpdateEvent;
}


void e28_manage_init(e28_manage_obj_p e28, uint8_t dev_type)
{
	/* e28相关函数 */
	e28->mode_set_f 		= e28_mode_set;
	e28->uart_deinit_f 	= e28_uart_deinit;
	e28->uart_reinit_f 	= e28_uart_reinit;
	e28->uart_send_f 		= e28_uart_send;
		 
	e28->mode = E28_WORKINGMODE_TRANS;
	e28->dev_type = dev_type;

	/* fsm相关函数 */
	e28_fsm_config(e28);
}


void e28_init(e28_manage_obj_p e28, uint16_t addr, uint8_t chan)
{
	e28_parm_TypeDef e28_InitStruct;

	e28_InitStruct.head = E28_HEAD_SAVE;
	e28_InitStruct.addr = addr;
	e28_InitStruct.par_type = E28_PARITY_8N1;
	e28_InitStruct.ttl_bps = E28_TTL_BPS_115200;
	e28_InitStruct.air_bps = E28_AIR_BPS_100K;
	e28_InitStruct.fpt_type = E28_FPT_DISABLE;
	e28_InitStruct.lbt_type = E28_LBT_DISABLE;
	e28_InitStruct.io_type = E28_IO_OUTPUT_PP;
	e28_InitStruct.power_type = E28_POWER_27DBM;
	e28_InitStruct.chan = chan;

	e28_parm_config(e28, &e28_InitStruct);
	
}


/********************** Usual API **********************************************/

/*==================================================================
* Function  : FsmEventHandle
* Description : 在定时器中定时轮询，避免动作函数中含有长任务函数
* Input Para  : 
* Output Para : 
* Return Value: 
==================================================================*/
static void fsmEventHandle(void* this_p)
{       
	e28_manage_obj_p e28;
	if(this_p != NULL)
		e28 = this_p;
	else
		log_e("null pointer!");
		
    uint8_t event;
    
    /* 取出触发事件队列中的事件 */
    if (fifo_s_isempty(&e28->event_fifo) != 1)
    {  
		event = fifo_s_get(&e28->event_fifo);
//		log_i("取出触发事件队列中的事件!");
			
        /* 在其它模块中改变触发事件，即可完成相应动作的执行 */
        FSM_EventHandle(&e28->fsm, event, NULL);
    } 
	else
	{
//		log_e("事件队列为空！");
	}
}

/*==================================================================
* Function  : fsmUpdateEvent
* Description : 更新触发事件
* Input Para  : 
* Output Para : 
* Return Value: 
==================================================================*/
static void fsmUpdateEvent(void* this_p, uint8_t event)
{    
	e28_manage_obj_p e28;
	if(this_p != NULL)
		e28 = this_p;
	else
		log_e("null pointer!");

	/* 触发事件入队 */
	if(fifo_s_isfull(&e28->event_fifo) == 1)
		log_e("触发事件入队失败,事件队列已满! event = %d\r\n",event);
	else
		fifo_s_put(&e28->event_fifo,event);
		
}

static void fsm_init(void* this_p, FsmTable_T *pTable, uint16_t stuMaxNum, uint8_t curState)
{   
	e28_manage_obj_p e28;
	if(this_p != NULL)
		e28 = this_p;
	else
		log_e("null pointer!");
	
    /* 创建事件触发队列 */
	fifo_s_init(&e28->event_fifo,\
				e28->event_fifo_buffer,\
				sizeof(e28->event_fifo_buffer));
 		
    /* 初始化状态机 */
    FSM_Init(&e28->fsm, pTable, stuMaxNum, curState);
}


static uint8_t e28_get_rx_state(e28_manage_obj_t* e28_obj)
{	
	uint8_t state = e28_obj->rx_updata;
	e28_obj->rx_updata = 0;
	return state;
}

void e28_set_rx_state(e28_manage_obj_t* e28_obj)
{
	e28_obj->rx_updata = 1;
}

uint8_t u8buf_compare(uint8_t *buf1,uint8_t *buf2, uint16_t len)
{
	for(int i = 0; i < len; i++)
	{
		if( *(buf1 + i) != *(buf2 + i) )
			return 0;
	}
	return 1;
}


static uint8_t E28_CMD_READ_PARM[3] = {0xc1,0xc1,0xc1};					//16 进制格式发送三个 C1，模块返回已保存的参数，必须连续发送。


static uint8_t e28_parm_read_compare(e28_manage_obj_t* e28_obj)
{
	e28_obj->uart_send_f(E28_CMD_READ_PARM,sizeof(E28_CMD_READ_PARM));
	e28_delayms(100);
	while(e28_get_rx_state(e28_obj) != 1 && e28_obj->rx_len == sizeof(e28_obj->e28_parm_pack))
	{
		e28_obj->uart_send_f(E28_CMD_READ_PARM,sizeof(E28_CMD_READ_PARM));
		e28_delayms(100);
	}
	if(e28_obj->rx_len != sizeof(e28_obj->e28_parm_pack))
	{
		log_e("e28_read_parm_error!\r\n");
		return 0;
	}
			
	if(u8buf_compare(e28_obj->rx_buf,(uint8_t*)&e28_obj->e28_parm_pack,sizeof(e28_obj->e28_parm_pack)) == 1)
	{
		return 1;
	}
	return 0;
}


static uint8_t e28_parm_write(e28_manage_obj_t* e28_obj)
{
	e28_obj->uart_send_f((uint8_t*)&e28_obj->e28_parm_pack,sizeof(e28_obj->e28_parm_pack));
	e28_delayms(100);
	while(e28_get_rx_state(e28_obj) != 1 && e28_obj->rx_len == sizeof(e28_obj->e28_parm_pack))
	{
		e28_obj->uart_send_f((uint8_t*)&e28_obj->e28_parm_pack,sizeof(e28_obj->e28_parm_pack));
		e28_delayms(100);
	}
	if(e28_obj->rx_len != sizeof(e28_obj->e28_parm_pack))
	{
		log_e("e28_read_parm_error!\r\n");
		return 0;
	}
	
	if(u8buf_compare(e28_obj->rx_buf,(uint8_t*)&e28_obj->e28_parm_pack,sizeof(e28_obj->e28_parm_pack)) == 1)
	{
		return 1;
	}
	return 0;
}

static uint32_t e28_get_uart_bps(e28_parm_pack_t *parm_pack)
{
	uint32_t bps = 9600;
	switch(parm_pack->sped.bit.ttl_bps)
	{
		case E28_TTL_BPS_1200:
			bps = 1200;
			break;
	
		case E28_TTL_BPS_4800:
			bps = 4800;
			break;

		case E28_TTL_BPS_9600:
			bps = 9600;
			break;
		
		case E28_TTL_BPS_19200:
			bps = 19200;
			break;
		
		case E28_TTL_BPS_57600:
			bps = 57600;
			break;
		
		case E28_TTL_BPS_115200:
			bps = 115200;
			break;
		
		case E28_TTL_BPS_460800:
			bps = 460800;
			break;	

		case E28_TTL_BPS_921600:
			bps = 921600;
			break;	
		
		default:
			log_e("get bps error!\r\n");
			break;
	}
	return bps;
}

static void e28_switch_mode(e28_manage_obj_t* e28_obj, e28_working_mode mode)
{
	uint32_t bps;

	e28_obj->mode_set_f(mode);		
	if(e28_obj->mode != mode && (e28_obj->mode == E28_WORKINGMODE_CONFIG | mode == E28_WORKINGMODE_CONFIG))
	{
		e28_obj->uart_deinit_f();
		if(mode == E28_WORKINGMODE_CONFIG)
			bps = 9600;
		else
			bps = e28_get_uart_bps(&e28_obj->e28_parm_pack);
		log_i("get bps %d\r\n",bps);
		e28_obj->uart_reinit_f(bps);
		e28_delayms(1000);
	}
	e28_obj->mode = mode;
}

uint8_t e28_parm_config(e28_manage_obj_t* e28_obj, e28_parm_TypeDef* e28_parm_InitStruct)
{
	
	parm_pack(e28_parm_InitStruct,&e28_obj->e28_parm_pack);

//	uint8_t* buf = (uint8_t*)&e28_obj->e28_parm_pack;
//	for(u8 i = 0; i < sizeof(e28_obj->e28_parm_pack); i++)
//	{
//		log_i("buf[%d] = %x",i,*(buf + i));
//	}		
	
	e28_switch_mode(e28_obj,E28_WORKINGMODE_CONFIG);
	
	if(e28_parm_read_compare(e28_obj) == 1)
	{
		log_i("parm is same! e28_parm_config_complete!");
		e28_switch_mode(e28_obj,E28_WORKINGMODE_TRANS);
		return 1;
	}
	e28_delayms(100);

	if(e28_parm_write(e28_obj) == 1)
	{
		log_i("e28_parm_config_complete!\r\n");
		e28_switch_mode(e28_obj,E28_WORKINGMODE_TRANS);
		return 1;
	}
	
	log_e("e28_parm_config_fail!\r\n");
	return 0;
}


static void parm_pack(e28_parm_TypeDef* e28_parm_InitStruct, e28_parm_pack_t* e28_parm_pack)
{
	e28_parm_pack->head = e28_parm_InitStruct->head;
	e28_parm_pack->addr8h = (uint8_t)(e28_parm_InitStruct->addr >> 8);
	e28_parm_pack->addr8l = (uint8_t)(e28_parm_InitStruct->addr);
	e28_parm_pack->sped.bit.parity = e28_parm_InitStruct->par_type;
	e28_parm_pack->sped.bit.ttl_bps = e28_parm_InitStruct->ttl_bps;
	e28_parm_pack->sped.bit.air_bps = e28_parm_InitStruct->air_bps;
	
	e28_parm_pack->chan = e28_parm_InitStruct->chan;
							 
	e28_parm_pack->option.bit.fpt_en = e28_parm_InitStruct->fpt_type;
	e28_parm_pack->option.bit.lbt_en = e28_parm_InitStruct->lbt_type;
	e28_parm_pack->option.bit.io_mode = e28_parm_InitStruct->io_type;
	e28_parm_pack->option.bit.power = e28_parm_InitStruct->power_type;
}


void e28_debug(e28_manage_obj_t* e28)
{
	e28_switch_mode(e28, E28_WORKINGMODE_RSSI);
}





