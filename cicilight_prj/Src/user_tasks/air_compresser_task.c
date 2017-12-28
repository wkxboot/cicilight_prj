#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "air_compressor_task.h"
#define APP_LOG_MODULE_NAME   "[main_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

/*压缩机任务*/
static void compressor_task(void const * argument)
{
 static osEvent msg;
 static uint8_t compressor_pwr_on=JUICE_FALSE;
 static int16_t cur_t,setting_t,increase,decrease;
 APP_LOG_INFO("++++++压缩机任务开始！\r\n"); 
 while(1)
 {
 msg=osMessageGet(compressor_msg_queue_hdl,COMPRESSOR_INTERVAL_VALUE);
 if(msg.status==osEventMessage && (msg.value.v==COMPRESSOR_OPEN_MSG || msg.value.v==COMPRESSOR_CLOSE_MSG))
 {
   APP_LOG_INFO("压缩机任务收到消息！\r\n");
 if(msg.value.v==COMPRESSOR_OPEN_MSG)
 {
  APP_LOG_INFO("打开压缩机！\r\n");
  BSP_compressor_pwr_on();
  compressor_pwr_on=JUICE_TRUE; 
 }
 else
 {
 APP_LOG_INFO("关闭压缩机！\r\n");
 BSP_compressor_pwr_dwn();
 compressor_pwr_on=JUICE_FALSE;  
 }
 osDelay(COMPRESSOR_TOMEOUT_VALUE);
 }
 
 get_reg_value(TEMPERATURE_REGINPUT_ADDR,1,(uint16_t*)&cur_t,REGINPUT_MODE);
 get_reg_value(T_SETTING_REGHOLDING_ADDR,1,(uint16_t*)&setting_t,REGHOLDING_MODE);
 get_reg_value(T_INCREASE_REGHOLDING_ADDR,1,(uint16_t*)&increase,REGHOLDING_MODE);
 get_reg_value(T_DECREASE_REGHOLDING_ADDR,1,(uint16_t*)&decrease,REGHOLDING_MODE);
 
 if(cur_t>setting_t+increase && compressor_pwr_on==JUICE_FALSE)
 {
   APP_LOG_INFO("打开压缩机，当前温度：%d°\r\n",cur_t);
   BSP_compressor_pwr_on();
   compressor_pwr_on=JUICE_TRUE;
 }
 else if(cur_t < setting_t-decrease && compressor_pwr_on==JUICE_TRUE)
 {
 APP_LOG_INFO("关闭压缩机，当前温度：%d°\r\n",cur_t);
 BSP_compressor_pwr_dwn();
 compressor_pwr_on=JUICE_FALSE;
 } 
 }
}