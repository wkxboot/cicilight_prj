#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#include "sys_led_task.h"
#define APP_LOG_MODULE_NAME   "[sys_led_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"

static void sys_led_task(void const * argument)
{
  APP_LOG_INFO("++++++系统运行灯任务开始！\r\n");
  while(1)
  {
  BSP_running_led_turn_on();
  osDelay(500);
  BSP_running_led_turn_off();
  osDelay(500);
  }
}