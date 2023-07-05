#include "sd710.h"
#include "drv_modbus.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "u8tool.h"

#define LOG_TAG "sd710_dev"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"



static void sd710_u8TOu32(uint16_t offest,uint8_t* savebuffer,uint32_t* data);
static void sd710_s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data);


static void sd710_delayms(uint16_t ms)
{
		osDelay(ms);
}


void sd710_03h_unpack(void* this_p, uint8_t* buff)
{
		modbus_rtu_p mbr;
		if(this_p != NULL)
				mbr = this_p;
		else
				log_e("null pointer!");
	
		uint32_t pos_temp;

		
		switch(mbr->addr)
		{
				case SD710_ADDRESS_1:
					
						break;
				case SD710_ADDRESS_2:
					
						break;
				
				default:
						break;
		}
		
		switch (mbr->reg)
		{		
				case SD710_REG_P00_00:		
						mbr->ack = 1;
						break;
				
				case SD710_REG_P00_01:		
						mbr->ack = 1;
						break;
				
				case SD710_REG_P03_00:		
						mbr->ack = 1;
						break;
				
				case SD710_REG_P03_04:		
						mbr->ack = 1;
						break;
				
				case SD710_REG_PE0_10:		
						sd710_u8TOu32(3,buff,&pos_temp);
						mbr->dev_pos_s = pos_temp;
						mbr->ack = 1;
						break;
				
				default:
						break;

		}
}

uint8_t sd710_read_test(modbus_rtu_p mbr)
{
		MBR_Read_03H_Block(mbr,mbr->dev_addr,SD710_REG_P00_00,1);
		return 1;
}

uint8_t sd710_read_pos(modbus_rtu_p mbr)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
	
		pass = MBR_ReadParam_03H(mbr,mbr->dev_addr,SD710_REG_PE0_10,2);
	
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}


uint8_t sd710_speed_mode_config(modbus_rtu_p mbr)
{
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_01,0x0000);			//�ر��ŷ�ʹ��
		sd710_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_85,0x0000);			//��д��eeprom
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_00,0x0001);			//����Ϊ�ٶ�ģʽ
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P03_00,0x0000);			//�ٶ�ָ������Ϊ�ڲ�����

		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P03_04,0);					//�ٶ�����Ϊ0
	
		return 1;
}

uint8_t sd710_pos_mode_config(modbus_rtu_p mbr)													//ע�⣺SD710_REG_P00_00��SD710_REG_P02_00Ϊ�����ϵ���Ч���ʲ����ɳ��������л�����ģʽ
{
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_01,0x0000);			//�ر��ŷ�ʹ��
		sd710_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_00,0x0000);			//����Ϊλ��ģʽ
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_85,0x0000);			//��д��eeprom
	
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P02_00,0x0003);			//λ��ָ������Ϊ�ڲ�����
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P08_00,0x0000);			//�ڲ�λ��ָ���趨
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P08_02,0x0100);			//�ڲ�λ��ָ���趨
	
		uint8_t data_buf[4];
		sd710_s32TOu8(0,data_buf,1);
		MBR_Write_10H_Block(mbr,mbr->dev_addr,SD710_REG_P02_04,2,data_buf);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P08_10,0x0010);			//PR ·�� 1 ������ L
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P08_A0,3000);			//�ڲ�Ŀ���ٶ��趨(���#0)
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P08_06,1);			//Pr ָ��ͨѶ�������������У�
	
		return 1;
	
}

uint8_t sd710_output_enable(modbus_rtu_p mbr)
{
		MBR_Write_06H_Block(mbr,mbr->dev_addr,SD710_REG_P00_01,0x0001);			//�����ŷ�ʹ��
		return 1;
}


uint8_t sd710_speed_set(modbus_rtu_p mbr,int16_t speed_cmd)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
	
		pass = MBR_WriteParam_06H(mbr,mbr->dev_addr,SD710_REG_P03_04,speed_cmd);
	
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}

uint8_t sd710_pos_set(modbus_rtu_p mbr,int32_t pos_cmd)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
		u8 data_buf[4];

		sd710_s32TOu8(0,data_buf,pos_cmd);

		pass = MBR_WriteParam_10H(mbr,mbr->dev_addr,SD710_REG_P08_12,2,data_buf);
	
		osDelay(5);
	
		pass = MBR_WriteParam_06H(mbr,mbr->dev_addr,SD710_REG_P08_06,0x0001);			//Pr ָ��ͨѶ�������������У�

		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}


static void sd710_s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data)
{
		savebuffer += offest;
		*savebuffer = (uint8_t)(data >> 8);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data >> 24);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data >> 16);
}



static void sd710_u8TOu32(uint16_t offest,uint8_t* savebuffer,uint32_t* data)
{
		*data = (uint32_t)(*(savebuffer + offest + 1) | *(savebuffer + offest)<<8 | *(savebuffer + offest + 3)<<16 | *(savebuffer + offest + 2)<<24);
}
