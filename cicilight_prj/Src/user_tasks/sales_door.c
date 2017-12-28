#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "slideway_task.h"
#define APP_LOG_MODULE_NAME   "[sales_door_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"
static void oh_door_task(void const * argument)
{
 osEvent msg;
 object_state_t oh_door;
 
 oh_door.active=JUICE_FALSE;
 oh_door.cur_pos=NULL_POS;
 oh_door.tar_pos=NULL_POS;
 oh_door.detect=JUICE_FALSE;
 oh_door.dir=NULL_DIR;
 oh_door.detect_timeout=0;
 oh_door.run_time=0;//记录电机每一次启动后的运行的时间
 APP_LOG_INFO("++++++升降门任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(oh_door_msg_queue_hdl,OH_DOOR_INTERVAL_VALUE);
 if(msg.status == osEventMessage && (msg.value.v==OH_DOOR_CLOSE_MSG || msg.value.v==OH_DOOR_OPEN_MSG))
 {
  APP_LOG_INFO("升降门任务收到消息：%d！\r\n",msg.value.v);
  oh_door.detect=JUICE_FALSE;
  oh_door.detect_timeout=0;
  oh_door.active=JUICE_TRUE;
  if(msg.value.v==OH_DOOR_CLOSE_MSG)
  {
  oh_door.tar_pos=BOT_POS;
  oh_door.detect_timeout=OH_DOOR_DETECT_TIMEOUT_VALUE+1;//向下运动先要检查有无异物
  APP_LOG_INFO("关升降门！\r\n");
  }
  else
  {
  oh_door.tar_pos=TOP_POS;
  APP_LOG_INFO("开升降门！\r\n");
  }
 }
 if(oh_door.active==JUICE_TRUE)//升降门是活动的
 {
 //确定升降门的位置
 if(BSP_is_oh_door_in_bot_pos()==JUICE_FALSE && BSP_is_oh_door_in_top_pos()==JUICE_FALSE && oh_door.cur_pos!=MID_POS)
 {
   oh_door.cur_pos=MID_POS;
   APP_LOG_INFO("升降门位置到中间！\r\n");
 }
 if(BSP_is_oh_door_in_bot_pos()==JUICE_TRUE && oh_door.cur_pos!=BOT_POS)
 {
  oh_door.cur_pos=BOT_POS;
  oh_door.detect=JUICE_FALSE;
  APP_LOG_INFO("升降门位置到底部！\r\n");
 }
 if(BSP_is_oh_door_in_top_pos()==JUICE_TRUE )
 {
  if(oh_door.cur_pos!=TOP_POS)
  {
  oh_door.cur_pos=TOP_POS;
  APP_LOG_INFO("升降门位置到顶部！\r\n");
  }
  if(oh_door.detect==JUICE_TRUE)//只有在开门后才开始计算超时时间
  oh_door.detect_timeout+=OH_DOOR_INTERVAL_VALUE;

 }
 /*升降门的异常状态*/
 //1.是否夹手或者对射管探测障碍物
 if((BSP_is_oh_door_hand_detected()==JUICE_TRUE || BSP_is_oh_door_clamp_hand()==JUICE_TRUE)  && (oh_door.dir==NEGATIVE_DIR ||oh_door.detect_timeout > OH_DOOR_DETECT_TIMEOUT_VALUE))
 {
  APP_LOG_WARNING("升降门防夹手检测到异物！\r\n");
  oh_door.detect=JUICE_TRUE;
  oh_door.detect_timeout=0;
  //BSP_oh_door_motor_pwr_on_positive();  
 }
 //2.是否电机电流过载
 if(oh_door.dir!=NULL_DIR)
 oh_door.run_time+= OH_DOOR_INTERVAL_VALUE;
 if(oh_door.run_time > OH_DOOR_OC_DELAY_VALUE && juice_is_oh_door_oc()==JUICE_TRUE)
 {
  APP_LOG_ERROR("升降门电机过流，电机停止！发送错误信号！\r\n");
  oh_door.dir=NULL_DIR;
  oh_door.run_time=0;
  oh_door.active=JUICE_FALSE;
  BSP_oh_door_motor_pwr_dwn();
  if(oh_door.dir==POSITIVE_DIR)
  juice_set_fault_code(FAULT_CODE_OH_DOOR_UP_OC); 
  else if(oh_door.dir==NEGATIVE_DIR)
  juice_set_fault_code(FAULT_CODE_OH_DOOR_DWN_OC);
  if(oh_door.tar_pos==TOP_POS)
  osSignalSet(sync_task_hdl,OH_DOOR_REACH_TOP_POS_ERR_SIGNAL);
  else if(oh_door.tar_pos==TOP_POS)
  osSignalSet(sync_task_hdl,OH_DOOR_REACH_BOT_POS_ERR_SIGNAL); 
  
  continue;//出错返回
 }
 
 /*确定升降门运行的方向*/
  if(oh_door.detect==JUICE_TRUE ) 
  {
   if(oh_door.cur_pos!=TOP_POS) 
   {
     if(oh_door.dir!=POSITIVE_DIR)
     {
     APP_LOG_INFO("升降门探测到异物后开始上升！\r\n");
     oh_door.dir=POSITIVE_DIR;
     oh_door.run_time=0;
     BSP_oh_door_motor_pwr_on_positive();
     }
   }
  else 
  {
   if(oh_door.dir!=NULL_DIR)
   {
   APP_LOG_INFO("升降门探测到异物后开门到位！\r\n");
   oh_door.dir=NULL_DIR;
   oh_door.run_time=0;
   BSP_oh_door_motor_pwr_dwn();
   }
   else if(oh_door.detect_timeout > OH_DOOR_DETECT_TIMEOUT_VALUE)
   {
   APP_LOG_INFO("升降门在没有探测到异物5秒后开始尝试再次关门！\r\n");
   oh_door.detect=JUICE_FALSE;
   oh_door.detect_timeout=0;
   oh_door.run_time=0;
   oh_door.dir=NEGATIVE_DIR;
   BSP_oh_door_motor_pwr_on_negative(); 
   }
  }
  }
  else if(oh_door.cur_pos!=oh_door.tar_pos)
  {
   if(oh_door.tar_pos==BOT_POS && oh_door.dir!=NEGATIVE_DIR)
   {
     APP_LOG_INFO("当前和目标位置不一致，反转！\r\n");
     oh_door.dir=NEGATIVE_DIR;
     oh_door.run_time=0;
     BSP_oh_door_motor_pwr_on_negative();
   }
   else if(oh_door.tar_pos==TOP_POS && oh_door.dir!=POSITIVE_DIR)
   {
   APP_LOG_INFO("当前和目标位置不一致，正转！\r\n");
   oh_door.dir=POSITIVE_DIR;
   oh_door.run_time=0;
   BSP_oh_door_motor_pwr_on_positive();  
   }
 }
 else
 { 
  APP_LOG_INFO("升降门到达目标位置！\r\n");
  APP_LOG_INFO("发送升降门到位信号！\r\n");
  oh_door.dir=NULL_DIR;
  oh_door.active=JUICE_FALSE;
  oh_door.run_time=0;
  BSP_oh_door_motor_pwr_dwn();
  if(oh_door.tar_pos==TOP_POS)
  osSignalSet(sync_task_hdl,OH_DOOR_REACH_TOP_POS_OK_SIGNAL);
  else
  osSignalSet(sync_task_hdl,OH_DOOR_REACH_BOT_POS_OK_SIGNAL); 
 }
 }
 }//end of while(1)
}