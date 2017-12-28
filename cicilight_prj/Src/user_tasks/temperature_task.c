#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "mb_reg.h"
#include "temperature_task.h"
#define APP_LOG_MODULE_NAME   "[temperature_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"
/*温度计任务*/
static void temperature_task(void const * argument)
{
 APP_LOG_INFO("++++++温度计任务开始！\r\n");
 int16_t cur_t=0;
 while(1)
 {
  osDelay(TEMPERATURE_INTERVAL_VALUE);
  
  cur_t=BSP_get_temperature(adc_average[ADC_T_IDX]);
  set_reg_value(TEMPERATURE_REGINPUT_ADDR,1,(uint16_t*)&cur_t,REGINPUT_MODE);

  APP_LOG_INFO("当前温度值：%d度！\r\n",cur_t);
  if(cur_t==BSP_ERR_T_VALUE)
  {
   juice_set_fault_code(FAULT_TEMEPERATURE_SENSOR_ERR);
   APP_LOG_ERROR("温度传感器故障错误！\r\n");
  }
  else if(cur_t> T_WARNING_HIGH)
  {
  juice_set_fault_code(FAULT_TEMEPERATURE_OVER_HIGH_MAX);
  APP_LOG_ERROR("温度值过高错误！\r\n");
  }
  else if(cur_t< T_WARNING_LOW)
  {
  juice_set_fault_code(FAULT_TEMEPERATURE_UNDER_LOW_MIN);
  APP_LOG_ERROR("温度值过低错误！\r\n");
  }

 }
}