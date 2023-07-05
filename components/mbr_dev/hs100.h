#ifndef __HS100_H__
#define __HS100_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj_config.h"
#include "drv_modbus.h"

#define HS100_ADDRESS_1	0x01
#define HS100_ADDRESS_2	0x02


#define HS100_REG_P00_02		0x0002			//�ŷ�����ģʽѡ�� 0��λ��ģʽ		1���ٶ�ģʽ		2��ת��ģʽ		3��λ��ģʽ->�ٶ�ģʽ		4��λ��ģʽ->ת��ģʽ	 5���ٶ�ģʽ->ת��ģʽ
#define HS100_REG_P01_20		0x0120			//�ٶ�ָ��ѡ��0�����ָ���		1��AI1����		2��AI2����		3���ٶȹ滮
#define HS100_REG_P01_21		0x0121			//�ٶȸ��� -6000~6000

#define HS100_REG_P01_40		0x0140			//ת��ָ��ѡ��0�����ָ���		1��AI1����		2��AI2����		
#define HS100_REG_P01_41		0x0141			//ת�ظ���		-4000~4000

#define HS100_REG_P40_00		0x4000	
#define HS100_REG_F31_41		0x3141	
#define HS100_REG_P01_00		0x0100	
#define HS100_REG_P03_00		0x0300			//��һ��滮λ�ƣ�32λ�Ĵ���
#define HS100_REG_P03_02		0x0302			//��һ��滮�ٶȣ�16λ�Ĵ���

#define HS100_REG_P01_1A		0x011A			//λ�ù滮ָ����·�ʽ		0���������  1:��������
#define HS100_REG_P01_18		0x0118			//λ�ù滮ģʽѡ��		0����������		1����������		2��DIѡ������		3�������켣ģʽ���У�DI����13��������,��ʵʱ�޸ĵ�һ��λ��ָ�����PO1-1Aѡ��ĸ��·�ʽ�������У�
#define HS100_REG_P0A_05		0x0A05			//ModbusͨѶ�洢ѡ��		0��������eeprom		1������eeprom

#define HS100_REG_D40_12		0x4012			//����λ�÷��������̵�λ
#define HS100_REG_D40_16		0x4016			//����λ�÷������û���λ



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


