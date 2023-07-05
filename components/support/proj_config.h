#ifndef __PROJ_CONFIG_H__
#define __PROJ_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
	
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
	
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


#ifdef __cplusplus
}
#endif

#endif // __PROJ_CONFIG_H__
