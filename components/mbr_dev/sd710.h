#ifndef __SD710_H__
#define __SD710_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "drv_modbus.h"

#define SD710_ADDRESS_1	0x01
#define SD710_ADDRESS_2	0x02


#define SD710_REG_P00_00		0x0000			//功能选择基本开关 0
#define SD710_REG_P00_01		0x0001			//功能选择基本开关 1
#define SD710_REG_P00_85		0x0085			//通讯写入功能码是否存入 Eeprom

#define SD710_REG_PE0_10		0xe010			//绝对值编码器单圈值,编码器单位 

#define SD710_REG_P02_00		0x0200			//位置指令源选择
#define SD710_REG_P02_04		0x0204			//电子齿轮分子(N) 
#define SD710_REG_P08_00		0x0800			//内部位置指令设定
#define SD710_REG_P08_02		0x0802			//内部多段位置（速度）运行模式
#define SD710_REG_P08_03		0x0803			//多段位置（速度）终点路径
#define SD710_REG_P08_04		0x0804			//顺序运行起始路径
#define SD710_REG_P08_06		0x0806			//Pr 指令通讯参数（单段运行）
#define SD710_REG_P08_10		0x0810			//PR 路径 1 控制字 L
#define SD710_REG_P08_12		0x0812			//Pr1 路径资料

#define SD710_REG_P08_A0		0x08A0			//内部目标速度设定(编号#0)

#define SD710_REG_P03_00		0x0300			//速度指令源选择
#define SD710_REG_P03_04		0x0304			//速度指令源选择



void sd710_03h_unpack(void* this_p, uint8_t* buff);

uint8_t sd710_read_test(modbus_rtu_p mbr);
uint8_t sd710_read_pos(modbus_rtu_p mbr);
uint8_t sd710_output_enable(modbus_rtu_p mbr);

uint8_t sd710_speed_mode_config(modbus_rtu_p mbr);
uint8_t sd710_pos_mode_config(modbus_rtu_p mbr);

uint8_t sd710_speed_set(modbus_rtu_p mbr,int16_t speed_cmd);
uint8_t sd710_pos_set(modbus_rtu_p mbr,int32_t pos_cmd);


#ifdef __cplusplus
}
#endif


#endif // __SD710_H__


