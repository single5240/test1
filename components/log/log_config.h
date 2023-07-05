#ifndef __LOG_CONFIG_H__
#define __LOG_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"

/* config */
#define LOG_OUTPUT_MAX_LEN  128

#define LOG_TIMESTAMP_EN     1
#define LOG_FUNCTION_EN      1
#define LOG_FILE_LINE_EN     1

#define LOG_ASSERT_EN        1
#define LOG_ERROR_EN         1
#define LOG_WARINING_EN      1
#define LOG_INFO_EN          1
#define LOG_DEBUG_EN         1

#define __log_output(log_str, len) do{uart5_transmit((uint8_t*)log_str, len);}while(0)

//#define __log_output(log_str, len) do{SEGGER_SYSVIEW_PrintfTargetEx((const char*)log_str,SEGGER_SYSVIEW_LOG);}while(0)

//#define __log_output(log_str, len) do{SEGGER_RTT_printf(0,(const char*)log_str);}while(0)


#ifdef __cplusplus
}
#endif



#endif // __LOG_CONFIG_H__
