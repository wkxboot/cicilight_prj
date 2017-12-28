#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "juicing_task.h"
#define APP_LOG_MODULE_NAME   "[juicing_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

//榨汁任务
static void juice_task(void const * argument)
{
 static osEvent msg;
 static uint32_t timeout=0;
 APP_LOG_INFO("++++++榨汁任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(juice_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage)
 {
 continue ;
 }
 APP_LOG_INFO("榨汁任务收到消息！\r\n");
 if(msg.value.v == JUICE_START_MSG)
 {
  APP_LOG_INFO("榨汁开始！打开榨汁电机！\r\n");
  BSP_juicing_motor_pwr_on(); 
  osDelay(5);
  BSP_juicing_motor_pwr_dwn(); 
  osDelay(500);
  BSP_juicing_motor_pwr_on(); 
  osDelay(5);
  BSP_juicing_motor_pwr_dwn(); 
  osDelay(500);
  BSP_juicing_motor_pwr_on();
  while(timeout<JUICING_TIMEOUT_VALUE-100-1010)//留100ms保证消息发送时间
  {
  msg= osMessageGet(juice_msg_queue_hdl,0); 
  if(msg.status==osEventMessage && msg.value.v == JUICE_STOP_MSG)
  break;
  osDelay(JUICE_INTERVAL_VALUE);
  timeout+=JUICE_INTERVAL_VALUE;
  }
  timeout=0;
  BSP_juicing_motor_pwr_dwn();  
  osSignalSet(sync_task_hdl,JUICE_TIME_OK_SIGNAL);
  APP_LOG_INFO("榨汁结束！关闭榨汁电机！\r\n");
 }  
 }
}