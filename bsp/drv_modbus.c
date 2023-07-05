#include "drv_modbus.h"

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "radio_task.h"
#include "u8tool.h"
#include "drv_led.h"
#include "sd710.h"
#include "hs100.h"

#define LOG_TAG "drv_modbus"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"


//#define debug_printf(x) log_i(x)


// CRC ��λ�ֽ�ֵ��
static const uint8_t s_CRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC ��λ�ֽ�ֵ��
const uint8_t s_CRCLo[] = {
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
		0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
		0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
		0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
		0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
		0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
		0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
		0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
		0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
		0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
		0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
		0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
		0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*
*********************************************************************************************************
*	�� �� ��: CRC16_Modbus
*	����˵��: ����CRC�� ����ModbusЭ�顣
*	��    ��: _pBuf : ����У�������
*			  _usLen : ���ݳ���
*	�� �� ֵ: 16λ����ֵ�� ����Modbus ���˽�����ֽ��ȴ��ͣ����ֽں��͡�
*
*   ���п��ܵ�CRCֵ����Ԥװ���������鵱�У������㱨������ʱ���Լ򵥵��������ɣ�
*   һ�����������16λCRC�������256�����ܵĸ�λ�ֽڣ���һ�����麬�е�λ�ֽڵ�ֵ��
*   ������������CRC�ķ�ʽ�ṩ�˱ȶԱ��Ļ�������ÿһ�����ַ��������µ�CRC����ķ�����
*
*  ע�⣺�˳����ڲ�ִ�и�/��CRC�ֽڵĽ������˺������ص����Ѿ�����������CRCֵ��Ҳ����˵���ú����ķ���ֵ����ֱ�ӷ���
*        �ڱ������ڷ��ͣ�
*********************************************************************************************************
*/
uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
		uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
		uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
		uint16_t usIndex;  /* CRCѭ���е����� */

		while (_usLen--)
		{
				usIndex = ucCRCHi ^ *_pBuf++; /* ����CRC */
				ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
				ucCRCLo = s_CRCLo[usIndex];
		}
		return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
u8 Verify_CRC16_Modbus(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = CRC16_Modbus( pchMessage, dwLength - 2);
    return  (((wExpected >> 8) & 0xff) == pchMessage[dwLength  - 2] &&  (wExpected & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Modbus(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = CRC16_Modbus(pchMessage,dwLength-2);
    pchMessage[dwLength-2] = (u8)((wCRC >> 8)& 0x00ff);
    pchMessage[dwLength-1] = (u8)(wCRC & 0x00ff);
}


void Modbus_Delay_MS(uint32_t ms)
{
		osDelay(ms);
	
}




uint8_t MBR_ReadParam_03H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _num)
{
		u8 retval = 0;
		u8 txcount = 0;
		uint8_t buf[50];
	
		if(mbr->ack == 0)
		{
				MODBUS_INFO_PRINTF("��ʧ�ظ�, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��03 \r\n",_addr,_reg);
				retval = 0;
		}
		else
				retval = 1;
		
		buf[txcount++] = _addr;
		buf[txcount++] = 0x03;		/* ������ */	
		buf[txcount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _reg;		/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
		buf[txcount++] = _num;		/* �Ĵ������� ���ֽ� */
	
		Append_CRC16_Modbus(buf,txcount+2);
	
		mbr->addr = _addr;
		mbr->cmd  = 0x03;
		mbr->reg  = _reg;
		mbr->length = txcount+2;
	
		mbr->usart_send_f(buf,mbr->length);
		mbr->ack = 0;
		
		return retval;
		
}

uint8_t MBR_WriteParam_06H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _value)
{
		u8 retval = 0;
		u8 txcount = 0;
		uint8_t buf[50];
	
		if(mbr->ack == 0)
		{
				MODBUS_INFO_PRINTF("��ʧ�ظ�, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��06 \r\n",_addr,_reg);
				retval = 0;
		}
		else
				retval = 1;
		
		buf[txcount++] = _addr;			/* ��վ��ַ */
		buf[txcount++] = 0x06;			/* ������ */	
		buf[txcount++] = _reg >> 8;		/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _reg;			/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _value >> 8;		/* �Ĵ���ֵ ���ֽ� */
		buf[txcount++] = _value;			/* �Ĵ���ֵ ���ֽ� */
	
		Append_CRC16_Modbus(buf,txcount+2);
	
		mbr->addr = _addr;
		mbr->cmd  = 0x06;
		mbr->reg  = _reg;
		mbr->length = txcount+2;
	
		mbr->usart_send_f(buf,mbr->length);
		mbr->ack = 0;
		
		return retval;
}

uint8_t MBR_WriteParam_10H(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
		u8 retval = 0;
		uint16_t i;
		u8 txcount = 0;
		uint8_t buf[50];
	
		if(mbr->ack == 0)
		{
				MODBUS_ERROR_PRINTF("��ʧ�ظ�, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��10 \r\n",_addr,_reg);
				retval = 0;
		}
		else
		{
//				MODBUS_INFO_PRINTF("success! �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��10 \r\n",_addr,_reg);
				retval = 1;
		}
	
		buf[txcount++] = _addr;		/* ��վ��ַ */
		buf[txcount++] = 0x10;		/* ��վ��ַ */	
		buf[txcount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _reg;		/* �Ĵ������ ���ֽ� */
		buf[txcount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
		buf[txcount++] = _num;		/* �Ĵ������� ���ֽ� */
		buf[txcount++] = 2 * _num;	/* �����ֽ��� */
		
		for (i = 0; i < 2 * _num; i++)
		{
				if (txcount > 50 - 3)
				{
						MODBUS_ERROR_PRINTF("���ݳ������������ȣ�ֱ�Ӷ���������");
						return 0;		/* ���ݳ������������ȣ�ֱ�Ӷ��������� */
				}
				buf[txcount++] = _buf[i];		/* ��������ݳ��� */
		}
	
		Append_CRC16_Modbus(buf,txcount+2);
	
		mbr->addr = _addr;
		mbr->cmd  = 0x10;
		mbr->reg  = _reg;
		mbr->length = txcount+2;
	
		mbr->usart_send_f(buf,mbr->length);
		mbr->ack = 0;
		return retval;

}

void MBR_Read_03H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _num)
{
		mbr->ack = 0;
		while(mbr->ack == 0)
		{
				Modbus_Delay_MS(10);
				MODBUS_INFO_PRINTF("MBR_Read_03H_Block, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��03 \r\n",_addr,_reg);
				Modbus_Delay_MS(10);
				mbr->ack = 1;
				MBR_ReadParam_03H(mbr,_addr,_reg,_num);
				Modbus_Delay_MS(20);
		}
}


void MBR_Write_06H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint16_t _value)
{
		mbr->ack = 0;
		while(mbr->ack == 0)
		{
				Modbus_Delay_MS(10);
				MODBUS_INFO_PRINTF("MBR_Read_06H_Block, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��06, ֵ��%d \r\n",_addr,_reg,_value);
				Modbus_Delay_MS(15);
				mbr->ack = 1;
				MBR_WriteParam_06H(mbr,_addr,_reg,_value);
				Modbus_Delay_MS(20);
		}
}


void MBR_Write_10H_Block(modbus_rtu_t *mbr,uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
		mbr->ack = 0;
		while(mbr->ack == 0)
		{
				Modbus_Delay_MS(10);
				MODBUS_INFO_PRINTF("MBR_Read_10H_Block, �ӻ���0x%x, �Ĵ�����ַ��0x%x, cmd��10 \r\n",_addr,_reg);
				Modbus_Delay_MS(15);
				mbr->ack = 1;
				MBR_WriteParam_10H(mbr,_addr,_reg,_num,_buf);
				Modbus_Delay_MS(20);
		}
}


static void fsm_init(void* this_p, FsmTable_T *pTable, uint16_t stuMaxNum, uint8_t curState)
{   
		modbus_rtu_p mbr;
		if(this_p != NULL)
				mbr = this_p;
		else
				log_e("null pointer!");
	
    /* �����¼��������� */
		fifo_s_init(&mbr->event_fifo,\
								mbr->event_fifo_buffer,\
								sizeof(mbr->event_fifo_buffer));
 		
    /* ��ʼ��״̬�� */
    FSM_Init(&mbr->fsm, pTable, stuMaxNum, curState);
}








/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_03H
*	����˵��: ����03Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//void MODH_Read_03H(modbus_rtu_t *mbr,u8* buff, uint8_t addr)
//{
//	
//		hs100servo_t* hs100;
//	
//		switch(addr)
//		{
//				case HS100_ADDRESS_1:
//						hs100 = &hs100_servo[0];
//						break;
//				case HS100_ADDRESS_2:
//						hs100 = &hs100_servo[1];
//						break;
//				
//				default:
//						break;
//			
//		}
//	
//		switch (mbr->reg)
//		{
//				case HS100_REG_F31_41:

//						mbr->ack = 1;
//						break;
//					
//				case HS100_REG_P03_00:		//��һ��滮λ�ƣ�32λ�Ĵ���
//			
//						mbr->ack = 1;
//						break;
//				
//				case HS100_REG_D40_12:		//����λ�÷��������̵�λ
//						u8TOs32(3,buff,&hs100->Absolute_position_feedback);
//						mbr->ack = 1;
//						break;
//				
//				case HS100_REG_D40_16:		//����λ�÷������û���λ
//						u8TOs32(3,buff,&hs100->Absolute_position_feedback);
//						mbr->ack = 1;
//						break;
//				
//				case HS100_REG_P03_02:		//��һ��滮�ٶ�
//						u8TOu16(3,buff,&hs100->posmode_speed);
//						mbr->ack = 1;
//						break;
//				
//				case HS100_REG_P00_02:		//��ȡ�ŷ�����ģʽ
//						u8TOu16(3,buff,(uint16_t*)&hs100->mode);
//				
//						mbr->ack = 1;
//						break;
//				
//				case HS100_REG_P01_20:		//�ٶ�ָ��ѡ��
//				
//						mbr->ack = 1;
//						break;
//				
//				case G100_REG_P00_01:		//����Դѡ��
//						u8TOu16(3,buff,(uint16_t*)&g100_converter.cmd_cfg);
//						mbr->ack = 1;
//						break;
//				
//				case G100_REG_P00_08:		//Ԥ��Ƶ��
//						u8TOu16(3,buff,&g100_converter.set_frequency);
//						mbr->ack = 1;
//						break;
//				
//				case G100_REG_PD1_00:		//ͨ���趨ֵ
//						u8TOu16(3,buff,&g100_converter.set_frequency);
//						mbr->ack = 1;
//						break;
//				
////				case SD710_REG_P00_00:		
////						mbr->ack = 1;
////						break;
//				
////				case SD710_REG_P00_01:		
////						mbr->ack = 1;
////						break;
//				
//				default:
//						break;

//		}
//}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_06H
*	����˵��: ����06Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_06H(modbus_rtu_t *mbr,u8* buff)
{

		if (buff[0] == mbr->addr)		
		{
				mbr->ack = 1;		/* ���յ�Ӧ�� */
//				log_i("06h get ack!");
		}

}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_10H
*	����˵��: ����10Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Read_10H(modbus_rtu_t *mbr,u8* buff)
{
	/*
		10Hָ���Ӧ��:
			�ӻ���ַ                11
			������                  10
			�Ĵ�����ʼ��ַ���ֽ�	00
			�Ĵ�����ʼ��ַ���ֽ�    01
			�Ĵ����������ֽ�        00
			�Ĵ����������ֽ�        02
			CRCУ����ֽ�           12
			CRCУ����ֽ�           98
	*/
		if (buff[0] == mbr->addr)		
		{
				mbr->ack = 1;		/* ���յ�Ӧ�� */
		}
}



void MBR_AnalyzeApp(modbus_rtu_t *mbr,u8* buff, uint16_t rec_length)
{
	
//		for(u8 i = 0; i < rec_length; i++)
//		{
//				log_i("buf[%d] = 0x%x",i,*(buff + i));
//		}		
		
//		log_i("%x %x %x %x",*(buff + 3),*(buff + 4),*(buff + 5),*(buff + 6));
	
		if(buff[0] == mbr->addr && Verify_CRC16_Modbus(buff,rec_length))
		{
				switch (buff[1])			/* ��2���ֽ� ������ */
				{
//					case 0x01:	/* ��ȡ��Ȧ״̬ */
//							MODH_Read_01H();
//							break;
//					case 0x02:	/* ��ȡ����״̬ */
//							MODH_Read_02H();
//							break;
					case 0x03:	/* ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ */
							if(mbr->unpack_03h_callback_f != NULL)
									mbr->unpack_03h_callback_f(mbr,buff);
							else
									MODBUS_ERROR_PRINTF("unpack_03h_callback_f not register!");
							break;
//					case 0x04:	/* ��ȡ����Ĵ��� */
//							MODH_Read_04H();
//							break;
//					case 0x05:	/* ǿ�Ƶ���Ȧ */
//							MODH_Read_05H();
//							break;
					case 0x06:	/* д�����Ĵ��� */
							MODH_Read_06H(mbr,buff);
							break;	
					case 0x10:	/* д����Ĵ��� */
							MODH_Read_10H(mbr,buff);
							break;		
					default:
							break;
				}
				
				if(mbr->unpack_success_callback_f != NULL)
						mbr->unpack_success_callback_f(mbr,buff,rec_length);
				else
						MODBUS_ERROR_PRINTF("mbr->unpack_success_callback not register!");
				
		}
		else
		{
				MODBUS_INFO_PRINTF("crc error!!\r\n");
				MODBUS_INFO_PRINTF("length = %d", rec_length);
		}
}


void mbr_up_ok_callback_register(modbus_rtu_t *m_obj, unpack_success_callback fun)
{
    m_obj->unpack_success_callback_f = fun;
    return;
}



////G100��Ƶ�����ã��ں�����������ֻ���ڳ�ʼ���׶ε���
//void G100_Converter_Config(modbus_rtu_t *mbr,uint8_t addr, int16_t Frequency)
//{
//		Modbus_Delay_MS(50);
//	
//		MODBUS_INFO_PRINTF("׼����ʼ����Ƶ��\r\n");
//		
//		MBR_Read_03H_Block(mbr,addr,G100_REG_P00_01,1);		//��ȡ����Դѡ��ͨѶ
//		
//		if(g100_converter.cmd_cfg == 2)
//		{
//				MODBUS_INFO_PRINTF("�ñ�Ƶ������Դѡ���Ѿ�����ΪͨѶ������������д��RAM		\r\n");
//			
//				Modbus_Delay_MS(10);
//			
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_01 + G100_REG_RAM,0x0002);	//����Դѡ��ͨѶ
//				
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_02 + G100_REG_RAM,0x0007);	//��Ƶ��ԴXѡ��	7��ͨѶ

//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_07 + G100_REG_RAM,0x0000);	//��Ƶ��Դ����ѡ��	0����Ƶ��ԴX
//				
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_08 + G100_REG_RAM,0x0000);	//Ԥ��Ƶ��
//							
//		}
//		else
//		{
//				MODBUS_INFO_PRINTF("�ñ�Ƶ������Դѡ��δ����ΪͨѶ������������д��EEPROM		\r\n");
//			
//				Modbus_Delay_MS(10);
//			
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_01,0x0002);	//����Դѡ��ͨѶ
//				
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_02,0x0007);	//��Ƶ��ԴXѡ��	7��ͨѶ

//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_07,0x0000);	//��Ƶ��Դ����ѡ��	0����Ƶ��ԴX
//				
//				MBR_Write_06H_Block(mbr,addr,G100_REG_P00_08,0x0000);	//Ԥ��Ƶ��
//		}
//		
//		MBR_Write_06H_Block(mbr,addr,G100_REG_PD1_00,Frequency);	//�޸�ͨ���趨ֵ
//		
//		MBR_Read_03H_Block(mbr,addr,G100_REG_PD1_00,1);		//�ض�ͨ���趨ֵ
//		
//		Modbus_Delay_MS(50);
//		
//}





/*==================================================================
* Function  : FsmEventHandle
* Description : �ڶ�ʱ���ж�ʱ��ѯ�����⶯�������к��г�������
* Input Para  : 
* Output Para : 
* Return Value: 
==================================================================*/
static void fsmEventHandle(void* this_p)
{       
		modbus_rtu_p mbr;
		if(this_p != NULL)
				mbr = this_p;
		else
				log_e("null pointer!");
		
    uint8_t event;
    
    /* ȡ�������¼������е��¼� */
    if (fifo_s_isempty(&mbr->event_fifo) != 1)
    {  
				event = fifo_s_get(&mbr->event_fifo);
//				log_i("ȡ�������¼������е��¼�!");
			
        /* ������ģ���иı䴥���¼������������Ӧ������ִ�� */
        FSM_EventHandle(&mbr->fsm, event, NULL);
    } 
		else
		{
//				log_e("�¼�����Ϊ�գ�");
		}
}

/*==================================================================
* Function  : fsmUpdateEvent
* Description : ���´����¼�
* Input Para  : 
* Output Para : 
* Return Value: 
==================================================================*/
static void fsmUpdateEvent(void* this_p, uint8_t event)
{    
		modbus_rtu_p mbr;
		if(this_p != NULL)
				mbr = this_p;
		else
				log_e("null pointer!");
	
    /* �����¼���� */
		if(fifo_s_isfull(&mbr->event_fifo) == 1)
        log_e("�����¼����ʧ��,�¼���������!\r\n");
		else
				fifo_s_put(&mbr->event_fifo,event);
		
}


void mbr_fsm_config(modbus_rtu_p mbr)
{
		mbr->fsm_init_f = fsm_init;
		mbr->fsm_eventHandle_f = fsmEventHandle;
		mbr->fsm_eventUpdate_f = fsmUpdateEvent;
}

void mbr_led_process(modbus_rtu_p mbr)
{
		if(++mbr->led_process_cnt > 10)
		{
				mbr->led_R.set_led_f(&mbr->led_R,0);
				mbr->led_L.set_led_f(&mbr->led_L,0);
			
				mbr->led_process_cnt = 0;
		}
	
}

void mbr_dev_mode_set(modbus_rtu_p mbr,uint8_t mode)
{
		mbr->dev_mode = mode;
}
