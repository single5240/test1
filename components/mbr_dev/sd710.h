#ifndef __SD710_H__
#define __SD710_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "drv_modbus.h"

#define SD710_ADDRESS_1	0x01
#define SD710_ADDRESS_2	0x02


#define SD710_REG_P00_00		0x0000			//����ѡ��������� 0
#define SD710_REG_P00_01		0x0001			//����ѡ��������� 1
#define SD710_REG_P00_85		0x0085			//ͨѶд�빦�����Ƿ���� Eeprom

#define SD710_REG_PE0_10		0xe010			//����ֵ��������Ȧֵ,��������λ 

#define SD710_REG_P02_00		0x0200			//λ��ָ��Դѡ��
#define SD710_REG_P02_04		0x0204			//���ӳ��ַ���(N) 
#define SD710_REG_P08_00		0x0800			//�ڲ�λ��ָ���趨
#define SD710_REG_P08_02		0x0802			//�ڲ����λ�ã��ٶȣ�����ģʽ
#define SD710_REG_P08_03		0x0803			//���λ�ã��ٶȣ��յ�·��
#define SD710_REG_P08_04		0x0804			//˳��������ʼ·��
#define SD710_REG_P08_06		0x0806			//Pr ָ��ͨѶ�������������У�
#define SD710_REG_P08_10		0x0810			//PR ·�� 1 ������ L
#define SD710_REG_P08_12		0x0812			//Pr1 ·������

#define SD710_REG_P08_A0		0x08A0			//�ڲ�Ŀ���ٶ��趨(���#0)

#define SD710_REG_P03_00		0x0300			//�ٶ�ָ��Դѡ��
#define SD710_REG_P03_04		0x0304			//�ٶ�ָ��Դѡ��



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


