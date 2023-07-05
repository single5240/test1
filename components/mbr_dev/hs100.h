#ifndef __HS100_H__
#define __HS100_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "drv_modbus.h"

#define HS100_ADDRESS_1	0x01
#define HS100_ADDRESS_2	0x02


#define HS100_REG_P00_02		0x0002			//伺服运行模式选择， 0：位置模式		1：速度模式		2：转矩模式		3：位置模式->速度模式		4：位置模式->转矩模式	 5：速度模式->转矩模式
#define HS100_REG_P01_20		0x0120			//速度指令选择，0：数字给定		1：AI1给定		2：AI2给定		3：速度规划
#define HS100_REG_P01_21		0x0121			//速度给定 -6000~6000

#define HS100_REG_P01_40		0x0140			//转矩指令选择，0：数字给定		1：AI1给定		2：AI2给定		
#define HS100_REG_P01_41		0x0141			//转矩给定		-4000~4000

#define HS100_REG_P40_00		0x4000	
#define HS100_REG_F31_41		0x3141	
#define HS100_REG_P01_00		0x0100	
#define HS100_REG_P03_00		0x0300			//第一组规划位移，32位寄存器
#define HS100_REG_P03_02		0x0302			//第一组规划速度，16位寄存器

#define HS100_REG_P01_1A		0x011A			//位置规划指令更新方式		0：缓存更新  1:立即更新
#define HS100_REG_P01_18		0x0118			//位置规划模式选择		0：单次运行		1：连续运行		2：DI选择运行		3：轮廓轨迹模式运行（DI功能13触发启动,可实时修改第一段位置指令并根据PO1-1A选择的更新方式连续运行）
#define HS100_REG_P0A_05		0x0A05			//Modbus通讯存储选择		0：不存入eeprom		1：存入eeprom

#define HS100_REG_D40_12		0x4012			//绝对位置反馈，码盘单位
#define HS100_REG_D40_16		0x4016			//绝对位置反馈，用户单位



void hs100_03h_unpack(void* this_p, uint8_t* buff);

uint8_t hs100_pos_mode_config(modbus_rtu_p mbr);
uint8_t hs100_speed_mode_config(modbus_rtu_p mbr);
uint8_t hs100_torque_mode_config(modbus_rtu_p mbr);

uint8_t hs100_speed_set(modbus_rtu_p mbr,int16_t speed_cmd);
uint8_t hs100_pos_set(modbus_rtu_p mbr,int32_t pos_cmd);
uint8_t hs100_torque_set(modbus_rtu_p mbr,int16_t torque_cmd);

uint8_t hs100_read_pos(modbus_rtu_p mbr);
uint8_t hs100_output_enable(modbus_rtu_p mbr);

uint8_t hs100_read_test(modbus_rtu_p mbr);

#ifdef __cplusplus
}
#endif


#endif // __HS100_H__


