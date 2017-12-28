#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" 
#define APP_LOG_MODULE_NAME   "[rgb_led_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "rgb_led.h"
static void rgb_led_task(void const * argument)
{
 //RGB LED 榨汁时旋转的颜色数组
 uint32_t wheel_color[JUICE_WHEEL_COLOR_CNT]=
 {RGB_LED_YELLOW_COLOR,RGB_LED_GREEN_COLOR,RGB_LED_BLUE_COLOR,RGB_LED_GREEN_COLOR,RGB_LED_BLUE_COLOR,RGB_LED_YELLOW_COLOR};
 uint8_t msg_v=NULL_MSG;
 uint8_t msg_hold=JUICE_TRUE;
 uint8_t pos,pos_cnt,color_idx;
 osEvent msg;
 APP_LOG_INFO("++++++RGB_LED任务开始！\r\n");
 while(1)
 {
 msg=osMessageGet(rgb_led_msg_queue_hdl,0);
 if(msg.status==osEventMessage)
 {
  APP_LOG_INFO("RGB LED任务收到消息：%d！\r\n",msg.value.v);
  msg_v=msg.value.v; 

  pos=0;
  pos_cnt=0;
  color_idx=0;
  msg_hold=JUICE_FALSE;
 }
 if(msg_v==RGB_LED_INDICATE_MSG )
 { 
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED黄色常亮指示消息，即将榨汁！\r\n");
  }
  single_color(RGB_LED_YELLOW_COLOR,255);
  osDelay(RGB_LED_BLINK_DELAY_VALUE);
  single_color(RGB_LED_BLACK_COLOR,255);
  osDelay(RGB_LED_BLINK_DELAY_VALUE); 
 }
 if(msg_v==RGB_LED_RAINBOW_MSG )
 { 
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED瀑布雨消息！\r\n");
  }
  rainbow(10); 
 }
 else if(msg_v==RGB_LED_JUICING_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  single_color(RGB_LED_BLACK_COLOR,255);//黑色
  APP_LOG_INFO("RGB LED榨汁消息！\r\n");
  }
  overwrite_color(wheel_color[color_idx],pos,255);
  pos++;
  pos_cnt++;
  if(pos>=RGB_LED_NUM_MAX)
  pos=0;
  if(pos_cnt>=RGB_LED_SECTION_CNT)
  {
  pos_cnt=0;
  color_idx++;
  if(color_idx>=JUICE_WHEEL_COLOR_CNT)
  color_idx=0;
  }
  if(pos_cnt==0)
  osDelay(RGB_LED_SECTION_DELAY_VALUE);
  
  osDelay(RGB_LED_OVERWRITE_DELAY_VALUE);                  
 }
 else if(msg_v==RGB_LED_OK_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 榨汁完成蓝常亮消息！\r\n");
  }
  single_color(RGB_LED_BLUE_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE);   
 }
 else if(msg_v==RGB_LED_STANDBY_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 待机黄色常亮消息！\r\n");
  }
  single_color(RGB_LED_YELLOW_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE);   
 }
 else if(msg_v==RGB_LED_MOTION_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 绿色运动常亮消息！\r\n");
  }
  single_color(RGB_LED_GREEN_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE);   
 }
 else if(msg_v==RGB_LED_ERROR_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 红色错误常亮消息！\r\n");
  }
  single_color(RGB_LED_RED_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE);   
 }
 else if(msg_v==RGB_LED_CLOSE_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 断电消息！\r\n");
  }
  single_color(RGB_LED_BLACK_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE);   
 }
 else if(msg_v==NULL_MSG)
 {
  if(msg_hold==JUICE_FALSE)
  {
  msg_hold=JUICE_TRUE;
  APP_LOG_INFO("RGB LED 空消息！\r\n");
  }
  single_color(RGB_LED_BLACK_COLOR,255);
  osDelay(RGB_LED_INTERVAL_VALUE); 
 }
}
}