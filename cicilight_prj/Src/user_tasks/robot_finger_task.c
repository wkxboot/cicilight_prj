#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "robot_finger_task.h"
#define APP_LOG_MODULE_NAME   "[robot_finger_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

//机械爪子舵机1任务
static void robot_finger_task(void const * argument)
{
 osEvent msg;
 APP_LOG_INFO("++++++机械爪子舵机1任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(servo1_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage)
 {
 APP_LOG_INFO("舵机1收到错误消息！\r\n");
 continue ;
 }
 APP_LOG_INFO("舵机1收到消息角度：%d°！\r\n",msg.value.v);
 HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
 MX_TIM2_ReInit_CH4((msg.value.v*11+500)/10);
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
 osDelay(SERVO1_ENABLE_TIME_VALUE);
 osSignalSet(sync_task_hdl,SERVO1_REACH_POS_OK_SIGNAL);
}
}
