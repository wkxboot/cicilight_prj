#ifndef APP_CONFIG_H
#define APP_CONFIG_H
// <h> APP_Log 

//==========================================================
// <e> APP_LOG_ENABLED - app_log - Logging
//==========================================================
#ifndef APP_LOG_ENABLED
#define APP_LOG_ENABLED          1
#endif

//***********************************************************/
#ifndef APP_LOG_USE_SEGGER_RTT
#define APP_LOG_USE_SEGGER_RTT   1
#endif


#ifndef APP_LOG_USE_UART
#define APP_LOG_USE_UART         0
#endif

#ifndef APP_LOG_USE_TIMESTAMP  
#define APP_LOG_USE_TIMESTAMP    1
#endif
                   
//==========================================================

// <h> nRF_Segger_RTT 

//==========================================================
// <h> segger_rtt - SEGGER RTT

//==========================================================


// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_UP - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_UP
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP      1024
#endif

// <o> SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS
#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS   2
#endif

// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN
#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN     16
#endif

// <o> SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS - Size of upstream buffer. 
#ifndef SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS
#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS  2
#endif

#include "SEGGER_RTT.h"

// <e> APP_LOG_USES_COLORS - If enabled then ANSI escape code for colors is prefixed to every string
//==========================================================
#ifndef APP_LOG_USES_COLORS
#define APP_LOG_USES_COLORS 1
#endif
#if  APP_LOG_USES_COLORS > 0
// <o> APP_LOG_COLOR_DEFAULT  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_LOG_COLOR_DEFAULT
#define APP_LOG_COLOR_DEFAULT 7
#endif

// <o> APP_LOG_ERROR_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_LOG_ERROR_COLOR
#define APP_LOG_ERROR_COLOR 2
#endif

// <o> APP_LOG_WARNING_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_LOG_WARNING_COLOR
#define APP_LOG_WARNING_COLOR 6
#endif

// <o> APP_LOG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_LOG_INFO_COLOR
#define APP_LOG_INFO_COLOR 3
#endif


// <o> APP_LOG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef APP_LOG_DEBUG_COLOR
#define APP_LOG_DEBUG_COLOR    5
#endif

#endif //APP_LOG_USES_COLORS

// </e>

// <o> APP_LOG_DEFAULT_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef APP_LOG_DEFAULT_LEVEL
#define APP_LOG_DEFAULT_LEVEL   4
#endif

#ifndef APP_LOG_MODULE_NAME  
#define APP_LOG_MODULE_NAME  "[default_module]"
#endif

#ifndef APP_LOG_MODULE_LEVEL  
#define APP_LOG_MODULE_LEVEL   APP_LOG_DEFAULT_LEVEL
#endif

// <<< end of configuration section >>>
#endif //SDK_CONFIG_H

