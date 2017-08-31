#include "app_log.h"


#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[app_log]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#endif

void app_log_init()
{
#if APP_LOG_ENABLED > 0  
 APP_LOG_INIT();
 APP_LOG_DEBUG("app log inited!\r\n");
#endif
}








