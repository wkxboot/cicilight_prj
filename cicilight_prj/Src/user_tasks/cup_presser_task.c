#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "slideway_task.h"
#define APP_LOG_MODULE_NAME   "[cup_presser_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

/*压杯任务*/
static void presser_task(void const * argument)
{
 osEvent msg;
 object_state_t presser;
 presser.active=JUICE_FALSE;
 presser.cur_pos=NULL_POS;
 presser.tar_pos=NULL_POS;
 presser.dir=NULL_DIR;
 presser.run_time=0;
 APP_LOG_INFO("++++++压杯任务开始！\r\n"); 
 while(1)
 {
 msg= osMessageGet(presser_msg_queue_hdl,PRESSER_INTERVAL_VALUE);
 if(msg.status==osEventMessage && (msg.value.v==PRESSER_UNPRESS_MSG || msg.value.v==PRESSER_PRESS_MSG))
 {
  APP_LOG_INFO("压杯任务收到消息！\r\n");
  presser.active=JUICE_TRUE;
  if(msg.value.v==PRESSER_UNPRESS_MSG)
  {
  presser.tar_pos=TOP_POS;
  APP_LOG_INFO("开始释放压杯！\r\n");
  }
  else
  {
  presser.tar_pos=BOT_POS;
  APP_LOG_INFO("开始压杯！\r\n");
  }
 }
 //确定压杯电机当前的位置
 if(presser.active==JUICE_TRUE)
 {
  if(BSP_is_cup_presser_in_top_pos()==JUICE_FALSE && BSP_is_cup_presser_in_bot_pos()==JUICE_FALSE && presser.cur_pos!=MID_POS)
  {
  APP_LOG_INFO("压杯电机到达中间位置！\r\n");
  presser.cur_pos=MID_POS;
  }
  else if(BSP_is_cup_presser_in_top_pos()==JUICE_TRUE && presser.cur_pos!=TOP_POS)
  {
  presser.cur_pos=TOP_POS;
  APP_LOG_INFO("压杯电机到达顶部位置！\r\n");
  }
  else if(BSP_is_cup_presser_in_bot_pos()==JUICE_TRUE && presser.cur_pos!=BOT_POS)
  {
  presser.cur_pos=BOT_POS;
  APP_LOG_INFO("压杯电机到达底部位置！\r\n");
  }
  
 /*检测压杯电机异常*/
  presser.run_time+=PRESSER_INTERVAL_VALUE;
 if(presser.run_time> PRESSER_OC_DELAY_VALUE && juice_is_presser_oc()==JUICE_TRUE)
 {
  APP_LOG_ERROR("压杯运行电流过载，停止电机，发送错误信号！\r\n");
  presser.run_time=0;
  presser.active=JUICE_FALSE;
  BSP_press_motor_pwr_dwn(); 
  if(presser.dir==POSITIVE_DIR)
  {
  juice_set_fault_code(FAULT_CODE_PRESSER_PRESS_OC);
  osSignalSet(sync_task_hdl,PRESSER_REACH_BOT_POS_ERR_SIGNAL);
  }
  else if(presser.dir==NEGATIVE_DIR)
  {
  juice_set_fault_code(FAULT_CODE_PRESSER_UNPRESS_OC);
  osSignalSet(sync_task_hdl,PRESSER_REACH_TOP_POS_ERR_SIGNAL); 
  }
  
  continue;
 }
  
 //如果没有到达目标位置
 if(presser.cur_pos!=presser.tar_pos)
 {
  if(presser.tar_pos==TOP_POS && presser.dir!=POSITIVE_DIR)
  {
   presser.dir=POSITIVE_DIR;
   presser.run_time=0;
   BSP_press_motor_pwr_on_positive();
  }
  else if(presser.tar_pos==BOT_POS && presser.dir!=NEGATIVE_DIR ) 
  {
   presser.dir=NEGATIVE_DIR;
   presser.run_time=0;
   BSP_press_motor_pwr_on_negative();
  } 
 }
 else //如果到达目标位置
 {
   presser.dir=NULL_DIR;
   presser.run_time=0;
   presser.active=JUICE_FALSE; 
   BSP_press_motor_pwr_dwn();
   APP_LOG_INFO("压杯运行到达目标位置，停止电机，发送到位信号！\r\n");
   if(presser.tar_pos==TOP_POS)
   osSignalSet(sync_task_hdl,PRESSER_REACH_TOP_POS_OK_SIGNAL);
   else
   osSignalSet(sync_task_hdl,PRESSER_REACH_BOT_POS_OK_SIGNAL); 
 }
 
 }
 }
}