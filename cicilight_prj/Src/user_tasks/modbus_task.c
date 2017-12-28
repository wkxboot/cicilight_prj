#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#include "modbus_task.h"
#define APP_LOG_MODULE_NAME   "[modbus_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"

static void modbus_task(void const * argument)
{
   APP_LOG_INFO("++++++MODBUS 任务开始!\r\n");
  /* Select either ASCII or RTU Mode. */
  ( void )eMBInit(MB_RTU, 0x0B, 0, 9600, MB_PAR_NONE );
  /* Enable the Modbus Protocol Stack. */
  ( void )eMBEnable();
  for( ;; )
  {
  /* Call the main polling loop of the Modbus protocol stack. */
  ( void )eMBPoll();
  } 
    
  
}