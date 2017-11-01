#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"   
#include "string.h"
#include "gpio.h"
#include "tim.h"
#include "adc.h"
#include "juice_common.h"
#include "JJDK_ZK_GZ1.h"
#include "io_check_task.h"
#include "user_tasks.h"

#define APP_LOG_MODULE_NAME   "[io_check_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG 
#include "app_log.h"
  
extern uint16_t adc_sample[];

void io_check_task(void const * argument)
{
  uint8_t io_check=JUICE_FALSE;
  APP_LOG_INFO("++++++IO口检测任务开始！\r\n");
  while(1)
  {   
   osDelay(100);
   if(io_check==JUICE_TRUE)
   {
   //微动开关和光电开关
   if( BSP_is_oh_door_hand_detected()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 1 IS OK!\r\n");
   }
   if(BSP_get_row_pos_sensor_state()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 2 IS OK!\r\n");
   }
    if( BSP_get_column_pos_sensor_state()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 3 IS OK!\r\n");
   }
    if( BSP_is_oh_door_in_top_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 4 IS OK!\r\n");
   }
    if( BSP_is_oh_door_in_bot_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 5 IS OK!\r\n");
   }
    if( BSP_is_cup_in_slot_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("光电开关 6 IS OK!\r\n");
   }
   
   if(BSP_is_row_step_motor_in_rst_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 1 IS OK!\r\n");
   }  
   if(BSP_is_column_step_motor_in_rst_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 2 IS OK!\r\n");
   }
   if( BSP_is_cup_press_ok()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 3 IS OK!\r\n");
   }
   if( BSP_is_cup_presser_in_bot_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 4 IS OK!\r\n");
   }
    if( BSP_is_cup_presser_in_top_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 5 IS OK!\r\n");
   }
    if(  BSP_is_oh_door_clamp_hand()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 6 IS OK!\r\n");
   }
    if( BSP_is_ms_7_in_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 7 IS OK!\r\n");
   }
    if( BSP_is_ms_8_in_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 8 IS OK!\r\n");
   }
    if( BSP_is_ms_9_in_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 9 IS OK!\r\n");
   }
    if( BSP_is_ms_10_in_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("微动开关 10 IS OK!\r\n");
   }
   }
  char cmd_str[CMD_STR_MAX_LEN];
  uint8_t str_len= SEGGER_RTT_HasData(0);
  
  if(str_len==0)
  {
   continue;
  }
  if(str_len > CMD_STR_MAX_LEN)
    str_len=CMD_STR_MAX_LEN;
  
  str_len=SEGGER_RTT_Read(0,cmd_str,str_len);
  for(uint8_t i=str_len;i>0;i--)
  {
   if(cmd_str[i-1]=='\r' || cmd_str[i-1]=='\n'){
   cmd_str[i-1]=0;} 
  }
  
  if(strcmp((const char*)cmd_str,CMD_IO_CHECK_ENABLE)==0)
  {
  io_check=JUICE_TRUE;
  APP_LOG_INFO("使能IO检测！\r\n"); 
  continue;
  }
  if(strcmp((const char*)cmd_str,CMD_IO_CHECK_DISABLE)==0)
  {
  io_check=JUICE_FALSE;
  APP_LOG_INFO("禁止IO检测！\r\n"); 
  continue;
  }
  if(strcmp((const char*)cmd_str,CMD_PWR_DWN_PRESSER)==0)
  {
  BSP_press_motor_pwr_dwn();
  APP_LOG_DEBUG("压杯电机断电！\r\n");
  continue; 
  }
 
  if(strcmp(cmd_str,CMD_PWR_ON_POSITIVE_PRESSER)==0)
  {
  BSP_press_motor_pwr_on_positive();
  APP_LOG_DEBUG("压杯电机正转！\r\n");
  continue;
  }
 
  if(strcmp(cmd_str,CMD_PWR_ON_NEGATIVE_PRESSER)==0)
  {
   BSP_press_motor_pwr_on_negative();
   APP_LOG_DEBUG("压杯电机反转！\r\n");
   continue;
  }
  if(strcmp(cmd_str,CMD_PWR_DWN_OH_DOOR)==0)
  {
  BSP_oh_door_motor_pwr_dwn();
  APP_LOG_DEBUG("升降门电机断电！\r\n");
  continue;
  }
 
 if(strcmp(cmd_str,CMD_PWR_ON_POSITIVE_OH_DOOR)==0)
 {
  BSP_oh_door_motor_pwr_on_positive();
  APP_LOG_DEBUG("升降门电机正转！\r\n");
  continue;
 }
  if(strcmp(cmd_str,CMD_PWR_ON_NEGATIVE_OH_DOOR)==0)
  {
  BSP_oh_door_motor_pwr_on_negative();
  APP_LOG_DEBUG("升降门电机反转！\r\n");
  continue;
  }
 if(strcmp(cmd_str,CMD_PWR_ON_ENV_LAMP)==0)
 {
  BSP_environment_lamp_pwr_on();
  APP_LOG_DEBUG("环境灯打开！\r\n");
  continue;
 }
 if(strcmp(cmd_str,CMD_PWR_DWN_ENV_LAMP)==0)
 {
  BSP_environment_lamp_pwr_dwn();
  APP_LOG_DEBUG("环境灯关闭！\r\n");
  continue;
 }
  if(strcmp(cmd_str,CMD_PWR_ON_COMPRESSOR)==0)
  {
  BSP_compressor_pwr_on();
  APP_LOG_DEBUG("压缩机通电！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_DWN_COMPRESSOR)==0)
  {
  BSP_compressor_pwr_dwn();
  APP_LOG_DEBUG("压缩机断电！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_ON_JUICING)==0)
  {
  BSP_juicing_motor_pwr_on();
  APP_LOG_DEBUG("榨汁电机通电！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_DWN_JUICING)==0)
  {
  BSP_juicing_motor_pwr_dwn();
  APP_LOG_DEBUG("榨汁电机断电！\r\n");
  continue;
  }
  
  if(strcmp(cmd_str,CMD_PWR_ON_POSITIVE_ROW_MOTOR)==0)
  {
  BSP_row_step_motor_pwr_on_positive();
  APP_LOG_DEBUG("行步进电机正转！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_ON_NEGATIVE_ROW_MOTOR)==0)
  {
  BSP_row_step_motor_pwr_on_negative();
  APP_LOG_DEBUG("行步进电机反转！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_DWN_ROW_MOTOR)==0)
  {
  BSP_row_step_motor_pwr_dwn();
  APP_LOG_DEBUG("行步进电机停机！\r\n");
  continue;
  }
  
  if(strcmp(cmd_str,CMD_PWR_ON_POSITIVE_COLUMN_MOTOR)==0)
  {
  BSP_column_step_motor_pwr_on_positive();
  APP_LOG_DEBUG("列步进电机正转！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_ON_NEGATIVE_COLUMN_MOTOR)==0)
  {
  BSP_column_step_motor_pwr_on_negative();
  APP_LOG_DEBUG("列步进电机反转！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_PWR_DWN_COLUMN_MOTOR)==0)
  {
  BSP_column_step_motor_pwr_dwn();
  APP_LOG_DEBUG("列步进电机停机！\r\n");
  continue;
  }
  if(strcmp(cmd_str,CMD_GET_TEMPERATURE)==0)
  {
  for(uint8_t i=0;i<5;i++)
  APP_LOG_INFO("ADC结果值:%d\r\n",adc_sample[i]);
  
  APP_LOG_INFO("温度结果值:%d\r\n",BSP_get_temperature(adc_sample[3]));
  continue;
  }
  if(strcmp(cmd_str,CMD_GET_DRV8711_STATUS)==0)
  {
    extern void get_status();
   get_status();
  continue;
  }

  extern osMessageQId servo2_msg_queue_hdl;
  extern osMessageQId servo1_msg_queue_hdl;
  extern osMessageQId manipulator_msg_queue_hdl;
  
  extern osMessageQId presser_msg_queue_hdl;
  extern osMessageQId oh_door_msg_queue_hdl;
  extern osMessageQId compressor_msg_queue_hdl;
  extern osMessageQId juice_msg_queue_hdl;
  extern osMessageQId rgb_led_msg_queue_hdl;
  
  if(strcmp(cmd_str,MSG_SERVO1_CLOSE)==0)
  {
  APP_LOG_DEBUG("爪子抓紧！\r\n");
  osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_CLOSE_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_SERVO1_OPEN)==0)
  {
  APP_LOG_DEBUG("爪子张开！\r\n");
  osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_OPEN_MSG,0);
  continue;
  }
  
  if(strcmp(cmd_str,MSG_SERVO2_25)==0)
  {
  APP_LOG_DEBUG("机械手臂25°！\r\n");
  osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_25_MSG,0);
  continue;
  }
  
  
 if(strcmp(cmd_str,MSG_SERVO2_90)==0)
  {
  APP_LOG_DEBUG("机械手臂90°！\r\n");
  osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_90_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_SERVO2_180)==0)
  {
  APP_LOG_DEBUG("机械手臂180°！\r\n");
  osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_180_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_MANIPULATOR_RST)==0)
  {
  APP_LOG_DEBUG("机械手臂运动！\r\n");
  osMessagePut(manipulator_msg_queue_hdl,MANIPULATOR_RST_VALUE,0);
  continue;
  }
  
  if(strcmp(cmd_str,MSG_PRESS_CUP)==0)
  {
   APP_LOG_DEBUG("发送压杯消息！\r\n");
   osMessagePut(presser_msg_queue_hdl,PRESSER_PRESS_MSG,0);
  continue;
  }
 
  if(strcmp(cmd_str,MSG_UNPRESS_CUP)==0)
  {
   APP_LOG_DEBUG("发送松杯消息！\r\n");
   osMessagePut(presser_msg_queue_hdl,PRESSER_UNPRESS_MSG,0);
   continue;
  }
 
 if(strcmp(cmd_str,MSG_OPEN_OH_DOOR)==0)
 {
  APP_LOG_DEBUG("发送开升降门消息！\r\n");
  osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_OPEN_MSG,0);
  continue;
 }
  if(strcmp(cmd_str,MSG_CLOSE_OH_DOOR)==0)
  {
  APP_LOG_DEBUG("发送关升降门消息！\r\n");
  osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_CLOSE_MSG,0);
  continue;
  }

  if(strcmp(cmd_str,MSG_TURN_ON_COMPRESSOR)==0)
  {
  APP_LOG_DEBUG("发送压缩机通电信号！\r\n");
  osMessagePut(compressor_msg_queue_hdl,COMPRESSOR_OPEN_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_OFF_COMPRESSOR)==0)
  {
  APP_LOG_DEBUG("发送压缩机断电信号！\r\n");
  osMessagePut(compressor_msg_queue_hdl,COMPRESSOR_CLOSE_MSG,0);
  
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_ON_JUICING)==0)
  {
  APP_LOG_DEBUG("发送榨汁电机通电消息！\r\n");
  osMessagePut(juice_msg_queue_hdl,JUICE_START_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_OFF_JUICING)==0)
  {
  APP_LOG_DEBUG("发送榨汁电机断电消息！\r\n");
  osMessagePut(juice_msg_queue_hdl,JUICE_STOP_MSG,0);
  continue;
  }
  
  
  if(strcmp(cmd_str,MSG_TURN_ON_RAINBOW_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯瀑布消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_RAINBOW_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_ON_INDICATE_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯闪烁消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_INDICATE_MSG,0);
  continue;
  }
  
  if(strcmp(cmd_str,MSG_TURN_ON_STANDBY_RGB_LED)==0)
  {
    APP_LOG_DEBUG("发送彩灯待机黄色常亮消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_STANDBY_MSG,0);
  continue;
  }
  
  if(strcmp(cmd_str,MSG_TURN_ON_JUICING_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯榨汁灯消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_JUICING_MSG,0);
  continue;
  } 
  if(strcmp(cmd_str,MSG_TURN_ON_OK_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯榨汁灯消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_OK_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_ON_MOTION_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯运动灯消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_MOTION_MSG,0);
  continue;
  }
   
  if(strcmp(cmd_str,MSG_TURN_ON_ERROR_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯错误灯消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_ERROR_MSG,0);
  continue;
  }
  if(strcmp(cmd_str,MSG_TURN_OFF_RGB_LED)==0)
  {
  APP_LOG_DEBUG("发送彩灯关灯消息！\r\n");
  osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_CLOSE_MSG,0);
  continue;
  }
  
  
  APP_LOG_DEBUG("非法命令！\r\n");
  }
 }