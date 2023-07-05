#ifndef __U8_TOOL_H__
#define __U8_TOOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

	
	
	
	
void u16TOu8(uint16_t offest,uint8_t* savebuffer,uint16_t data);
void u8TOu16(uint16_t offest,uint8_t* savebuffer,uint16_t* data);

void s16TOu8(uint16_t offest,uint8_t* savebuffer,int16_t data);
void u8TOs16(uint16_t offest,uint8_t* savebuffer,int16_t* data);
	
void floatTou8(uint16_t offest,uint8_t* savebuffer,float data);
void u8Tofloat(uint16_t offest,uint8_t* savebuffer,float* data);
	
void doubleTou8(uint16_t offest,uint8_t* savebuffer,double data);
void u8Todouble(uint16_t offest,uint8_t* savebuffer,double* data);
	
void u8TOs32(uint16_t offest,uint8_t* savebuffer,int32_t* data);
void s32TOu8(uint16_t offest,uint8_t* savebuffer,int32_t data);

void u8TOu32(uint16_t offest,uint8_t* savebuffer,uint32_t* data);
void u32TOu8(uint16_t offest,uint8_t* savebuffer,uint32_t data);



#ifdef __cplusplus
}
#endif


#endif // __U8_TOOL_H__


