#include "hs100.h"
#include "drv_modbus.h"
#include "drv_uart.h"
#include "init.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "u8tool.h"

#define LOG_TAG "hs100_dev"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"



static void hs100_u8TOs32(uint16_t offest,uint8_t* savebuffer,int32_t* data);
static void hs100_s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data);


static void hs100_delayms(uint16_t ms)
{
		osDelay(ms);
}


void hs100_03h_unpack(void* this_p, uint8_t* buff)
{
		modbus_rtu_p mbr;
		if(this_p != NULL)
				mbr = this_p;
		else
				log_e("null pointer!");
		
		int32_t pos_temp;
		switch(mbr->addr)
		{
				case HS100_ADDRESS_1:
					
						break;
				case HS100_ADDRESS_2:
					
						break;
				
				default:
						break;
			
		}
	
		switch (mbr->reg)
		{
				case HS100_REG_F31_41:

						mbr->ack = 1;
						break;
					
				case HS100_REG_P03_00:		//��һ��滮λ�ƣ�32λ�Ĵ���
			
						mbr->ack = 1;
						break;
				
				case HS100_REG_D40_12:		//����λ�÷��������̵�λ
						hs100_u8TOs32(3,buff,&pos_temp);
						mbr->dev_pos_s = pos_temp;
						mbr->ack = 1;
						break;
				
				case HS100_REG_D40_16:		//����λ�÷������û���λ
//						u8TOs32(3,buff,&hs100->Absolute_position_feedback);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P03_02:		//��һ��滮�ٶ�
//						u8TOu16(3,buff,&hs100->posmode_speed);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P00_02:		//��ȡ�ŷ�����ģʽ
//						u8TOu16(3,buff,(uint16_t*)&hs100->mode);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P01_20:		//�ٶ�ָ��ѡ��
				
						mbr->ack = 1;
						break;
												
				default:
						break;

		}
		
	
}


uint8_t hs100_read_test(modbus_rtu_p mbr)
{
//		return MBR_ReadParam_03H(mbr,mbr->dev_addr,HS100_REG_F31_41,1);
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,1);
		return 1;
}

//HS100�ŷ�λ��ģʽ���ã��ں�����������ֻ���ڳ�ʼ���׶ε���
uint8_t hs100_pos_mode_config(modbus_rtu_p mbr)
{
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//�ر��ŷ�ʹ��
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM�洢���ã�������eeprom
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0000);	//�����ŷ�����ģʽ��λ��ģʽ

		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//��ȡ�ŷ�����ģʽ
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_00,0x0001);	//λ��ָ��ѡ��  1��λ�ù滮   0����������
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_1A,0x0001);	//λ��ָ����·�ʽ���ã�1����������
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_18,0x0001);	//λ�ù滮ģʽѡ��		3�������켣ģʽ���У�DI����13��������,��ʵʱ�޸ĵ�һ��λ��ָ�����PO1-1Aѡ��ĸ��·�ʽ�������У�
				
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P03_02,3000);	//���õ�һ��滮�ٶ�
		
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P03_02,1);	//��ȡ��һ��滮�ٶ�
	
		hs100_delayms(50);
		
		return 1;
}

//HS100�ŷ��ٶ�ģʽ���ã��ں�����������ֻ���ڳ�ʼ���׶ε���
uint8_t hs100_speed_mode_config(modbus_rtu_p mbr)
{
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//�ر��ŷ�ʹ��
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM�洢���ã�������eeprom

		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0001);	//�����ŷ�����ģʽ���ٶ�ģʽ

		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//��ȡ�ŷ�����ģʽ
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_20,0x0000);	//�ٶ�ָ��ѡ��0�����ָ���
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_21,0);	//�ٶȸ���ֵ
		
		hs100_delayms(50);
	
		return 1;
}

//HS100�ŷ�ת��ģʽ���ã��ں�����������ֻ���ڳ�ʼ���׶ε���
uint8_t hs100_torque_mode_config(modbus_rtu_p mbr)
{
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,1);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//�ر��ŷ�ʹ��
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM�洢���ã�������eeprom
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0002);	//�����ŷ�����ģʽ��ת��ģʽ
		
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//��ȡ�ŷ�����ģʽ
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_40,0x0000);	//ת��ָ��ѡ��0�����ָ���
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_41,0);	//ת�ظ���ֵ
			
		hs100_delayms(50);
	
		return 1;
}


uint8_t hs100_output_enable(modbus_rtu_p mbr)
{
		if(mbr->dev_mode == MBR_DEV_POS_MODE)
		{
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0001);			//�����ŷ�ʹ��
				hs100_delayms(500);
			
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x1001);	//DI����13����λ�ù滮ָ�����
				hs100_delayms(50);
		}
		else
		{
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0001);			//�����ŷ�ʹ��
				hs100_delayms(500);
		}
		return 1;
}

//HS�ŷ����ٶ�ģʽ�������ٶȸ���		-6000~6000
uint8_t hs100_speed_set(modbus_rtu_p mbr,int16_t speed_cmd)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
	
		pass = MBR_WriteParam_06H(mbr,mbr->dev_addr,HS100_REG_P01_21,speed_cmd);
		
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}

//HS�ŷ���λ��ģʽ������Ŀ��λ�ã�������λ��ģʽ�£�
uint8_t hs100_pos_set(modbus_rtu_p mbr,int32_t pos_cmd)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
		u8 data_buf[4];

		hs100_s32TOu8(0,data_buf,pos_cmd);

		pass = MBR_WriteParam_10H(mbr,mbr->dev_addr,HS100_REG_P03_00,2,data_buf);
	
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}

//HS�ŷ���ת��ģʽ������ת�ظ���		-4000~4000
uint8_t hs100_torque_set(modbus_rtu_p mbr,int16_t torque_cmd)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;

		pass = MBR_WriteParam_06H(mbr,mbr->dev_addr,HS100_REG_P01_41,torque_cmd);
	
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}


uint8_t hs100_read_pos(modbus_rtu_p mbr)
{
		uint8_t pass;
		static uint8_t fail_cnt = 0;
	
		pass = MBR_ReadParam_03H(mbr,mbr->dev_addr,HS100_REG_D40_12,2);
	
		if(pass == 0)
				fail_cnt++;
		else
				fail_cnt = 0;
		
		if(fail_cnt > 20)
				return 0;
		else
				return 1;
}

static void hs100_u8TOs32(uint16_t offest,uint8_t* savebuffer,int32_t* data)
{
		*data = (int32_t)(*(savebuffer + offest + 1) | *(savebuffer + offest)<<8 | *(savebuffer + offest + 3)<<16 | *(savebuffer + offest + 2)<<24);
}



static void hs100_s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data)
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
