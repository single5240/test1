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
					
				case HS100_REG_P03_00:		//第一组规划位移，32位寄存器
			
						mbr->ack = 1;
						break;
				
				case HS100_REG_D40_12:		//绝对位置反馈，码盘单位
						hs100_u8TOs32(3,buff,&pos_temp);
						mbr->dev_pos_s = pos_temp;
						mbr->ack = 1;
						break;
				
				case HS100_REG_D40_16:		//绝对位置反馈，用户单位
//						u8TOs32(3,buff,&hs100->Absolute_position_feedback);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P03_02:		//第一组规划速度
//						u8TOu16(3,buff,&hs100->posmode_speed);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P00_02:		//读取伺服运行模式
//						u8TOu16(3,buff,(uint16_t*)&hs100->mode);
						mbr->ack = 1;
						break;
				
				case HS100_REG_P01_20:		//速度指令选择
				
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

//HS100伺服位置模式配置，内含阻塞操作，只能在初始化阶段调用
uint8_t hs100_pos_mode_config(modbus_rtu_p mbr)
{
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//关闭伺服使能
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM存储设置，不存入eeprom
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0000);	//设置伺服运行模式：位置模式

		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//读取伺服运行模式
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_00,0x0001);	//位置指令选择  1：位置规划   0：脉冲输入
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_1A,0x0001);	//位置指令更新方式设置，1：立即更新
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_18,0x0001);	//位置规划模式选择		3：轮廓轨迹模式运行（DI功能13触发启动,可实时修改第一段位置指令并根据PO1-1A选择的更新方式连续运行）
				
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P03_02,3000);	//设置第一组规划速度
		
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P03_02,1);	//读取第一组规划速度
	
		hs100_delayms(50);
		
		return 1;
}

//HS100伺服速度模式配置，内含阻塞操作，只能在初始化阶段调用
uint8_t hs100_speed_mode_config(modbus_rtu_p mbr)
{
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//关闭伺服使能
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM存储设置，不存入eeprom

		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0001);	//设置伺服运行模式：速度模式

		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//读取伺服运行模式
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_20,0x0000);	//速度指令选择，0：数字给定
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_21,0);	//速度给定值
		
		hs100_delayms(50);
	
		return 1;
}

//HS100伺服转矩模式配置，内含阻塞操作，只能在初始化阶段调用
uint8_t hs100_torque_mode_config(modbus_rtu_p mbr)
{
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,1);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0000);	//关闭伺服使能
	
		hs100_delayms(1000);
	
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P0A_05,0x0000);	//EEPROM存储设置，不存入eeprom
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,0x0002);	//设置伺服运行模式：转矩模式
		
		MBR_Read_03H_Block(mbr,mbr->dev_addr,HS100_REG_P00_02,1);		//读取伺服运行模式
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_40,0x0000);	//转矩指令选择，0：数字给定
		
		MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_P01_41,0);	//转矩给定值
			
		hs100_delayms(50);
	
		return 1;
}


uint8_t hs100_output_enable(modbus_rtu_p mbr)
{
		if(mbr->dev_mode == MBR_DEV_POS_MODE)
		{
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0001);			//开启伺服使能
				hs100_delayms(500);
			
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x1001);	//DI功能13触发位置规划指令更新
				hs100_delayms(50);
		}
		else
		{
				MBR_Write_06H_Block(mbr,mbr->dev_addr,HS100_REG_F31_41,0x0001);			//开启伺服使能
				hs100_delayms(500);
		}
		return 1;
}

//HS伺服在速度模式下设置速度给定		-6000~6000
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

//HS伺服在位置模式下设置目标位置（必须在位置模式下）
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

//HS伺服在转矩模式下设置转矩给定		-4000~4000
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
