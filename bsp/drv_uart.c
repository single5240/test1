/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "usart.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "drv_uart.h"
#include "init.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define LOG_TAG "drv_uart"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
static uint8_t uart5_rx_buff[UART5_RX_BUFFER_SIZE];
static uint8_t uart5_tx_buff[UART5_TX_BUFFER_SIZE];
static uint8_t uart5_tx_fifo_buff[UART5_TX_FIFO_SIZE];
usart_manage_obj_t uart5_manage_obj = {0};


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
static uint8_t usart6_rx_buff[USART6_RX_BUFFER_SIZE];
static uint8_t usart6_tx_buff[USART6_TX_BUFFER_SIZE];
static uint8_t usart6_tx_fifo_buff[USART6_TX_FIFO_SIZE];
usart_manage_obj_t usart6_manage_obj = {0};


extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
static uint8_t usart2_rx_buff[USART2_RX_BUFFER_SIZE];
static uint8_t usart2_tx_buff[USART2_TX_BUFFER_SIZE];
static uint8_t usart2_tx_fifo_buff[USART2_TX_FIFO_SIZE];
usart_manage_obj_t usart2_manage_obj = {0};


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
static uint8_t usart3_rx_buff[USART3_RX_BUFFER_SIZE];
static uint8_t usart3_tx_buff[USART3_TX_BUFFER_SIZE];
static uint8_t usart3_tx_fifo_buff[USART3_TX_FIFO_SIZE];
usart_manage_obj_t usart3_manage_obj = {0};


static void usart_rec_to_buff(usart_manage_obj_t *m_obj, interrput_type int_type);
static void usart_transmit_hook(usart_manage_obj_t *m_obj);
static void usart_manage_delete(usart_manage_obj_t *m_obj);
int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);

/********************** Custom API **********************************************/
void debug_raw_printf(char *fmt, ...)
{
    va_list arg;
    uint8_t buff[UART5_PRINTF_BUFF_SIZE];
    uint8_t printf_len;

    va_start(arg, fmt);
    printf_len = vsnprintf((char *)buff, UART5_PRINTF_BUFF_SIZE, fmt, arg);
    va_end(arg);

    if (printf_len > UART5_PRINTF_BUFF_SIZE)
    {
        printf_len = UART5_PRINTF_BUFF_SIZE;
    }

    HAL_UART_Transmit(&huart5, (uint8_t *)buff, printf_len, 0xFFFF);
}

/********************** Custom API **********************************************/

void uart5_idle_callback(void)
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        usart_rec_to_buff(&uart5_manage_obj, INTERRUPT_TYPE_UART);
    }
}

void usart6_idle_callback(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart6.hdmarx);

        /* handle dbus data dbus_buf from DMA */

				 usart6_manage_obj.call_back_f(usart6_manage_obj.rx_buffer,USART6_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
			
        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART6_RX_BUFFER_SIZE);
        __HAL_DMA_ENABLE(huart6.hdmarx);
    }
}

void usart2_idle_callback(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart2.hdmarx);

        /* handle dbus data dbus_buf from DMA */

				 usart2_manage_obj.call_back_f(usart2_manage_obj.rx_buffer,USART2_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
			
        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart2.hdmarx, USART2_RX_BUFFER_SIZE);
        __HAL_DMA_ENABLE(huart2.hdmarx);
    }

}

void usart3_idle_callback(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart3.hdmarx);

        /* handle dbus data dbus_buf from DMA */

				 usart3_manage_obj.call_back_f(usart3_manage_obj.rx_buffer,USART3_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
			
        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart3.hdmarx, USART3_RX_BUFFER_SIZE);
        __HAL_DMA_ENABLE(huart3.hdmarx);
    }

}


void uart5_manage_init(void)
{
    uart5_manage_obj.rx_buffer = uart5_rx_buff;
    uart5_manage_obj.rx_buffer_size = UART5_RX_BUFFER_SIZE;
    uart5_manage_obj.dma_h = &hdma_uart5_rx;
    uart5_manage_obj.uart_h = &huart5;
    uart5_manage_obj.tx_fifo_buffer = uart5_tx_fifo_buff;
    uart5_manage_obj.tx_fifo_size = UART5_TX_FIFO_SIZE;
    uart5_manage_obj.tx_buffer_size = UART5_TX_BUFFER_SIZE;
    uart5_manage_obj.tx_buffer = uart5_tx_buff;
    uart5_manage_obj.is_sending = 0;

    fifo_s_init(&(uart5_manage_obj.tx_fifo), uart5_tx_fifo_buff, UART5_TX_FIFO_SIZE);

    HAL_UART_Receive_DMA(&huart5, uart5_rx_buff, UART5_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}



void usart6_manage_init(void)					//e28
{
    usart6_manage_obj.rx_buffer = usart6_rx_buff;
    usart6_manage_obj.rx_buffer_size = USART6_RX_BUFFER_SIZE;
    usart6_manage_obj.dma_h = &hdma_usart6_rx;
    usart6_manage_obj.uart_h = &huart6;
    usart6_manage_obj.tx_fifo_buffer = usart6_tx_fifo_buff;
    usart6_manage_obj.tx_fifo_size = USART6_TX_FIFO_SIZE;
    usart6_manage_obj.tx_buffer_size = USART6_TX_BUFFER_SIZE;
    usart6_manage_obj.tx_buffer = usart6_tx_buff;
    usart6_manage_obj.is_sending = 0;

    fifo_s_init(&(usart6_manage_obj.tx_fifo), usart6_tx_fifo_buff, USART6_TX_FIFO_SIZE);

    UART_Receive_DMA_No_IT(&huart6, usart6_rx_buff, USART6_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}


void usart2_manage_init(void)				//mbr_no2
{
    usart2_manage_obj.rx_buffer = usart2_rx_buff;
    usart2_manage_obj.rx_buffer_size = USART2_RX_BUFFER_SIZE;
    usart2_manage_obj.dma_h = &hdma_usart2_rx;
    usart2_manage_obj.uart_h = &huart2;
    usart2_manage_obj.tx_fifo_buffer = usart2_tx_fifo_buff;
    usart2_manage_obj.tx_fifo_size = USART2_TX_FIFO_SIZE;
    usart2_manage_obj.tx_buffer_size = USART2_TX_BUFFER_SIZE;
    usart2_manage_obj.tx_buffer = usart2_tx_buff;
    usart2_manage_obj.is_sending = 0;

    fifo_s_init(&(usart2_manage_obj.tx_fifo), usart2_tx_fifo_buff, USART2_TX_FIFO_SIZE);

    UART_Receive_DMA_No_IT(&huart2, usart2_rx_buff, USART2_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}


void usart3_manage_init(void)
{
    usart3_manage_obj.rx_buffer = usart3_rx_buff;
    usart3_manage_obj.rx_buffer_size = USART3_RX_BUFFER_SIZE;
    usart3_manage_obj.dma_h = &hdma_usart3_rx;
    usart3_manage_obj.uart_h = &huart3;
    usart3_manage_obj.tx_fifo_buffer = usart3_tx_fifo_buff;
    usart3_manage_obj.tx_fifo_size = USART3_TX_FIFO_SIZE;
    usart3_manage_obj.tx_buffer_size = USART3_TX_BUFFER_SIZE;
    usart3_manage_obj.tx_buffer = usart3_tx_buff;
    usart3_manage_obj.is_sending = 0;

    fifo_s_init(&(usart3_manage_obj.tx_fifo), usart3_tx_fifo_buff, USART3_TX_FIFO_SIZE);

    UART_Receive_DMA_No_IT(&huart3, usart3_rx_buff, USART3_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}



void usart6_manage_deinit(void)
{
		usart_manage_delete(&usart6_manage_obj);
}


int uart5_printf(char *fmt, ...)
{

    va_list arg;
    uint8_t buff[UART5_PRINTF_BUFF_SIZE];
    uint8_t printf_len;

    va_start(arg, fmt);
    printf_len = vsnprintf((char *)buff, UART5_PRINTF_BUFF_SIZE, fmt, arg);
    va_end(arg);

    if (printf_len > UART5_PRINTF_BUFF_SIZE)
    {
        printf_len = UART5_PRINTF_BUFF_SIZE;
    }

    usart_transmit(&uart5_manage_obj, buff, printf_len);

    return 0;
}

void uart5_transmit(uint8_t *buff, uint16_t len)
{
    usart_transmit(&uart5_manage_obj, buff, len);
}

void usart6_transmit(uint8_t *buff, uint16_t len)
{
    usart_transmit(&usart6_manage_obj, buff, len);
}

void usart2_send_before(void)
{
		HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port,USART2_TXEN_Pin,GPIO_PIN_SET);
}

void usart2_send_over(void)
{
		HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port,USART2_TXEN_Pin,GPIO_PIN_RESET);
}

void usart3_send_before(void)
{
		HAL_GPIO_WritePin(USART3_TXEN_GPIO_Port,USART3_TXEN_Pin,GPIO_PIN_SET);
}

void usart3_send_over(void)
{
		HAL_GPIO_WritePin(USART3_TXEN_GPIO_Port,USART3_TXEN_Pin,GPIO_PIN_RESET);
}

void usart2_transmit(uint8_t *buff, uint16_t len)
{
		usart2_send_before();
    usart_transmit(&usart2_manage_obj, buff, len);
}

void usart3_transmit(uint8_t *buff, uint16_t len)
{
		usart3_send_before();
    usart_transmit(&usart3_manage_obj, buff, len);
}



void uart5_rx_callback_register(usart_call_back fun)
{
    usart_rx_callback_register(&uart5_manage_obj, fun);
    return;
}

void usart6_rx_callback_register(usart_call_back fun)
{
    usart_rx_callback_register(&usart6_manage_obj, fun);
    return;
}

void usart2_rx_callback_register(usart_call_back fun)
{
    usart_rx_callback_register(&usart2_manage_obj, fun);
    return;
}

void usart3_rx_callback_register(usart_call_back fun)
{
    usart_rx_callback_register(&usart3_manage_obj, fun);
    return;
}



/********************** Usual API **********************************************/
/**
  * @brief  register uart callback function.
  * @param
  * @retval void
  */
void usart_rx_callback_register(usart_manage_obj_t *m_obj, usart_call_back fun)
{
    m_obj->call_back_f = fun;
    return;
}

/**
  * @brief  rx dma half complete interupt
  * @param
  * @retval void
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        usart_rec_to_buff(&uart5_manage_obj, INTERRUPT_TYPE_DMA_HALF);
    }
//		else if(huart == &huart6)
//		{
//        usart_rec_to_buff(&usart6_manage_obj, INTERRUPT_TYPE_DMA_HALF);
//		}
//		else if(huart == &huart2)
//		{
//        usart_rec_to_buff(&usart2_manage_obj, INTERRUPT_TYPE_DMA_HALF);
//		}
//		else if(huart == &huart3)
//		{
//        usart_rec_to_buff(&usart3_manage_obj, INTERRUPT_TYPE_DMA_HALF);
//		}

    return;
}

/**
  * @brief  rx dma complete interupt
  * @param
  * @retval void
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        usart_rec_to_buff(&uart5_manage_obj, INTERRUPT_TYPE_DMA_ALL);
    }
//    else if (huart == &huart6)
//    {
//        usart_rec_to_buff(&usart6_manage_obj, INTERRUPT_TYPE_DMA_ALL);
//    }
//    else if (huart == &huart2)
//    {
//        usart_rec_to_buff(&usart2_manage_obj, INTERRUPT_TYPE_DMA_ALL);
//    }
//    else if (huart == &huart3)
//    {
//        usart_rec_to_buff(&usart3_manage_obj, INTERRUPT_TYPE_DMA_ALL);
//    }
		
    return;
}

/**
  * @brief  tx complete interupt
  * @param
  * @retval void
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        usart_transmit_hook(&uart5_manage_obj);
    }
    else if (huart == &huart6)
    {
        usart_transmit_hook(&usart6_manage_obj);
    }
    else if (huart == &huart2)
    {
				usart2_send_over();
        usart_transmit_hook(&usart2_manage_obj);
    }
    else if (huart == &huart3)
    {
				usart3_send_over();
        usart_transmit_hook(&usart3_manage_obj);
    }
    return;
}



/******************************************抽象层，各版本应保持同步更新，version：1.2 **********************************************/

/**
  * @brief  uart fifo transmit
  * @param
  * @retval error code
  */
int32_t usart_transmit(usart_manage_obj_t *m_obj, uint8_t *buf, uint16_t len)
{
    uint16_t to_send_len;
    uint16_t to_tx_fifo_len;

    if (m_obj->is_sending == 0)
    {
        if (len < m_obj->tx_buffer_size)
        {
            to_send_len = len;
            to_tx_fifo_len = 0;
        }
        else if (len < m_obj->tx_buffer_size + m_obj->tx_fifo_size)
        {
            to_send_len = m_obj->tx_buffer_size;
            to_tx_fifo_len = len - m_obj->tx_buffer_size;
        }
        else
        {
            to_send_len = m_obj->tx_buffer_size;
            to_tx_fifo_len = m_obj->tx_fifo_size;
        }
    }
    else
    {
        if (len < m_obj->tx_fifo_size)
        {
            to_send_len = 0;
            to_tx_fifo_len = len;
        }
        else
        {
            to_send_len = 0;
            to_tx_fifo_len = m_obj->tx_fifo_size;
        }
    }

    if (to_send_len > 0)
    {
        memcpy(m_obj->tx_buffer, buf, to_send_len);
        m_obj->is_sending = 1;
        HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->tx_buffer, to_send_len);
    }
    if (to_tx_fifo_len > 0)
    {
        uint8_t len;
        len = fifo_s_puts(&(m_obj->tx_fifo), (char *)(buf) + to_send_len, to_tx_fifo_len);

        if (len != to_tx_fifo_len)
        {
            return -1;
        }
    }

    return 0;
}

static void usart_transmit_hook(usart_manage_obj_t *m_obj)
{
    uint16_t fifo_data_num = 0;
    uint16_t send_num = 0;

    fifo_data_num = m_obj->tx_fifo.used_num;

    if (fifo_data_num != 0)
    {
        if (fifo_data_num < m_obj->tx_buffer_size)
        {
            send_num = fifo_data_num;
        }
        else
        {
            send_num = m_obj->tx_buffer_size;
        }
        fifo_s_gets(&(m_obj->tx_fifo), (char *)(m_obj->tx_buffer), send_num);
        m_obj->is_sending = 1;
        HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->tx_buffer, send_num);
    }
    else
    {
        m_obj->is_sending = 0;
    }
    return;
}

/**
  * @brief  rx to fifo
  * @param
  * @retval void
  */
static void usart_rec_to_buff(usart_manage_obj_t *m_obj, interrput_type int_type)
{
    uint16_t read_end_ptr = 0;
    uint16_t read_length = 0;
    uint16_t read_success_length = 0;
    uint16_t read_start_ptr = m_obj->read_start_index;
    uint8_t *pdata = m_obj->rx_buffer;
		uint8_t buffer_forfixbug[512];
	
    UNUSED(read_success_length);

    uint16_t buff_left = m_obj->dma_h->Instance->NDTR;

    if (int_type == INTERRUPT_TYPE_UART)
    {
        read_end_ptr = m_obj->rx_buffer_size - buff_left;
    }

    if (int_type == INTERRUPT_TYPE_DMA_HALF)
    {
        read_end_ptr = m_obj->rx_buffer_size / 2;
    }

    if (int_type == INTERRUPT_TYPE_DMA_ALL)
    {
        read_end_ptr = m_obj->rx_buffer_size;
    }

		if(read_end_ptr > m_obj->read_start_index)
		{
				read_length = read_end_ptr - m_obj->read_start_index;
				if (m_obj->call_back_f != NULL)
				{
						uint8_t *read_ptr = pdata + read_start_ptr;
						read_success_length = m_obj->call_back_f(read_ptr, read_length);
				}
		}
		else
		{
				read_length = read_end_ptr + m_obj->rx_buffer_size - m_obj->read_start_index;
				memcpy(buffer_forfixbug, (uint8_t *)(pdata + read_start_ptr),(m_obj->rx_buffer_size - m_obj->read_start_index));
				memcpy((uint8_t *)(buffer_forfixbug + (m_obj->rx_buffer_size - m_obj->read_start_index)), pdata, read_end_ptr);
			
				if (m_obj->call_back_f != NULL)
				{
						read_success_length = m_obj->call_back_f(buffer_forfixbug, read_length);
				}
		}
		

    m_obj->read_start_index = (m_obj->read_start_index + read_length) % (m_obj->rx_buffer_size);

    return;
}


static void usart_manage_delete(usart_manage_obj_t *m_obj)
{
		memset(m_obj,0,sizeof(*m_obj));
}



/**
  * @brief  dr16 uart dma configration
  * @param
  * @retval error code
  */
int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
    uint32_t tmp = 0;

    tmp = huart->RxState;
    if (tmp == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;

        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                      (uint32_t)pData, Size);

        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
        in the UART CR3 register */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

