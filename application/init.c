#include "init.h"

#include "main.h"
#include "drv_uart.h"
#include "drv_modbus.h"
#include "drv_e28.h"
#include "modbus1_task.h"

#include "event_mgr.h"
#include "easyflash.h"

#define LOG_TAG "init"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

publisher_t rc_data_Pub;
publisher_t radio_rx_Pub;

env_pack_t rc_env;

extern RNG_HandleTypeDef hrng;

static void boot_times_cal(void);
static void env_auto_create(void);


uint32_t get_time_us(void)
{
    return TIM7->CNT;
}

uint32_t get_time_ms(void)
{
    return HAL_GetTick();
}

float get_time_ms_us(void)
{
    return get_time_ms() + get_time_us() / 1000.0f;
}

void* get_rc_env(void)
{
	return &rc_env;
}


/**
  * @brief  usart1 interupt, debug shell
  * @param
  * @retval void
  */
uint32_t uart5_rx_callback(uint8_t *buff, uint16_t len)
{
//    shell_interupt(buff, len);
	
//		__log_output(buff, len);
//		usart6_transmit(buff,len);
    return 0;
}


/**
  * @brief  usart3 interupt
  * @param
  * @retval void
  */
uint32_t usart6_rx_callback(uint8_t *buff, uint16_t len)
{
	e28_rx_callback(buff,len);
    return 0;
}


uint32_t usart2_rx_callback(uint8_t *buff, uint16_t len)
{
	modbus_usart2_rx_callback(buff,len);
    return 0;
}


uint32_t usart3_rx_callback(uint8_t *buff, uint16_t len)
{
	modbus_usart3_rx_callback(buff,len);
    return 0;
}

/**
  * @brief  board init
  * @param
  * @retval void
  */
void board_config(void)
{
    /* system log */
    uart5_manage_init();
    log_printf("\r\n\r\n"
               "************SANGLIANG Receiver**************\r\n");
    log_printf("* Copy right: All right reserved.\r\n");
    log_printf("* Release Time: %s.\r\n", __TIME__);
    log_printf("********************************************\r\n");
	
    uart5_rx_callback_register(uart5_rx_callback);
	
    usart6_manage_init();
    usart6_rx_callback_register(usart6_rx_callback);
	
    usart2_manage_init();
    usart2_rx_callback_register(usart2_rx_callback);
	
    usart3_manage_init();
    usart3_rx_callback_register(usart3_rx_callback);
	
	EventPostInit(&rc_data_Pub, RC_DATA_MSG, RC_DATA_MSG_LEN);
	EventPostInit(&radio_rx_Pub, RADIO_RX_MSG, RADIO_RX_MSG_LEN);
	
	easyflash_init();

	env_auto_create();
	
	boot_times_cal();
	
	printf_all_env();
	
}

static void env_auto_create(void)
{
	size_t read_len;
	
	read_len = ef_get_env_blob("boot_times",&rc_env.sys.boot_times,sizeof(rc_env.sys.boot_times),&read_len);	
	if(read_len == 0)			//如果env不存在，直接用出厂值覆写ENV。
		ef_set_env_blob("boot_times",0,sizeof(rc_env.sys.boot_times));
	
	read_len = ef_get_env_blob("radio_addr",&rc_env.radio.addr,sizeof(rc_env.radio.addr),&read_len);
	uint16_t addr = (uint16_t)HAL_RNG_GetRandomNumber(&hrng);
	if(read_len == 0)			//如果env不存在，直接用出厂值覆写ENV。
		ef_set_env_blob("radio_addr",&addr,sizeof(rc_env.radio.addr));
	
	read_len = ef_get_env_blob("radio_chan",&rc_env.radio.chan,sizeof(rc_env.radio.chan),&read_len);
	uint8_t chan = 0x18;
	if(read_len == 0)			//如果env不存在，直接用出厂值覆写ENV。
		ef_set_env_blob("radio_chan",&chan,sizeof(rc_env.radio.chan));
	
	read_len = ef_get_env_blob("radio_match_key",&rc_env.radio.match_key,sizeof(rc_env.radio.match_key),&read_len);	
	uint16_t match_key = (uint16_t)HAL_RNG_GetRandomNumber(&hrng);
	if(read_len == 0)			//如果env不存在，直接用出厂值覆写ENV。
		ef_set_env_blob("radio_match_key",&match_key,sizeof(rc_env.radio.match_key));
		
}

void printf_all_env(void)
{
	size_t read_len;
	
	int32_t boot_times;
	ef_get_env_blob("boot_times",&boot_times,sizeof(boot_times),&read_len);
	log_i("boot_times = %d",boot_times);
	
	uint16_t radio_addr;
	ef_get_env_blob("radio_addr",&radio_addr,sizeof(radio_addr),&read_len);
	log_i("radio_addr = %d",radio_addr);
	
	uint8_t radio_chan;
	ef_get_env_blob("radio_chan",&radio_chan,sizeof(radio_chan),&read_len);
	log_i("radio_chan = %d",radio_chan);
	
	uint16_t match_key;
	ef_get_env_blob("radio_match_key",&match_key,sizeof(match_key),&read_len);
	log_i("match_key = %d",match_key);
	
}



static void boot_times_cal(void)
{
	int boot_times = 0;
	size_t read_len = 0;
	ef_get_env_blob("boot_times",&boot_times,sizeof(boot_times),&read_len);
	boot_times ++;
	ef_set_env_blob("boot_times",&boot_times,sizeof(boot_times));
//	log_i("boot_times = %d",boot_times);	
}


