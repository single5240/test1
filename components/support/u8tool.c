#include "u8tool.h"


//��ʽת��������
typedef union
{
    unsigned char U[4];
    float F;
    unsigned short I;
} FormatTrans;

//��ʽת��������
typedef union
{
    unsigned char U[8];
    double F;
    unsigned short I;
} FormatTrans_8Bytes;



//�޷���16λ���//
void u16TOu8(uint16_t offest,uint8_t* savebuffer,uint16_t data)
{
		savebuffer += offest;
		*savebuffer = (uint8_t)(data >> 8);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data);
}
void u8TOu16(uint16_t offest,uint8_t* savebuffer,uint16_t* data)
{
		uint8_t temp[2];
		savebuffer += offest+1;
		temp[1] = *savebuffer;
		savebuffer -= 1;
		temp[0] = *savebuffer;
		
		*data = (uint16_t)(temp[1] | temp[0]<<8);
}


//�з���16λ���//
void s16TOu8(uint16_t offest,uint8_t* savebuffer,int16_t data)
{
		savebuffer += offest;
		*savebuffer = (uint8_t)(data >> 8);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data);
}
void u8TOs16(uint16_t offest,uint8_t* savebuffer,int16_t* data)
{
		uint8_t temp[2];
		savebuffer += offest+1;
		temp[1] = *savebuffer;
		savebuffer -= 1;
		temp[0] = *savebuffer;
		
		*data = (int16_t)(temp[1] | temp[0]<<8);
}

//�޷�32λ���//
void u32TOu8(uint16_t offest,uint8_t* savebuffer,uint32_t data)
{
		savebuffer += offest;
		*savebuffer = (uint8_t)(data >> 24);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data >> 16);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data >> 8);
		savebuffer +=1;
		*savebuffer = (uint8_t)(data);
}
void u8TOu32(uint16_t offest,uint8_t* savebuffer,uint32_t* data)
{
		uint8_t temp[4];
		savebuffer += offest+3;
		temp[3] = *savebuffer;
		savebuffer -= 1;
		temp[2] = *savebuffer;
		savebuffer -= 1;
		temp[1] = *savebuffer;
		savebuffer -= 1;
		temp[0] = *savebuffer;
	
		*data = (uint32_t)(temp[3] | temp[2]<<8 | temp[1]<<16 | temp[0]<<24);
}


//�����ȸ�����//
void floatTou8(uint16_t offest,uint8_t* savebuffer,float data)
{
		FormatTrans temp;
	
		temp.F = data;
	
		savebuffer += offest;
		*savebuffer = temp.U[0];
		savebuffer +=1;
		*savebuffer = temp.U[1];
		savebuffer +=1;
		*savebuffer = temp.U[2];
		savebuffer +=1;
		*savebuffer = temp.U[3];
}
void u8Tofloat(uint16_t offest,uint8_t* savebuffer,float* data)
{
		FormatTrans temp;
		savebuffer += offest;
	
		temp.U[0] = *savebuffer;
		savebuffer +=1;
		temp.U[1] = *savebuffer;
		savebuffer +=1;
		temp.U[2] = *savebuffer;
		savebuffer +=1;
		temp.U[3] = *savebuffer;
	
		*data = temp.F;
}

//˫���ȸ�����//
void doubleTou8(uint16_t offest,uint8_t* savebuffer,double data)
{
		FormatTrans_8Bytes temp;
	
		temp.F = data;
	
		savebuffer += offest;
		*savebuffer = temp.U[0];
		savebuffer +=1;
		*savebuffer = temp.U[1];
		savebuffer +=1;
		*savebuffer = temp.U[2];
		savebuffer +=1;
		*savebuffer = temp.U[3];
		savebuffer +=1;
		*savebuffer = temp.U[4];
		savebuffer +=1;
		*savebuffer = temp.U[5];
		savebuffer +=1;
		*savebuffer = temp.U[6];
		savebuffer +=1;
		*savebuffer = temp.U[7];
}
void u8Todouble(uint16_t offest,uint8_t* savebuffer,double* data)
{
		FormatTrans_8Bytes temp;
		savebuffer += offest;
	
		temp.U[0] = *savebuffer;
		savebuffer +=1;
		temp.U[1] = *savebuffer;
		savebuffer +=1;
		temp.U[2] = *savebuffer;
		savebuffer +=1;
		temp.U[3] = *savebuffer;
		savebuffer +=1;
		temp.U[4] = *savebuffer;
		savebuffer +=1;
		temp.U[5] = *savebuffer;
		savebuffer +=1;
		temp.U[6] = *savebuffer;
		savebuffer +=1;
		temp.U[7] = *savebuffer;
	
		*data = temp.F;
}



void u8TOs32(uint16_t offest,uint8_t* savebuffer,int32_t* data)
{
		uint8_t temp[4];
	
		savebuffer += offest;
	
		temp[0] = *savebuffer;
		savebuffer +=1;
	
		temp[1] = *savebuffer;
		savebuffer +=1;
	
		temp[2] = *savebuffer;
		savebuffer +=1;
	
		temp[3] = *savebuffer;
	
		
		*data = (int32_t)(temp[1] | temp[0]<<8 | temp[3]<<16 | temp[2]<<24);
}


//�з���32λ���//
void s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data)
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
