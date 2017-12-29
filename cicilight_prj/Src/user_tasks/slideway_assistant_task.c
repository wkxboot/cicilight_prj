#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "slideway_task.h"
#define APP_LOG_MODULE_NAME   "[slideway_assistant]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

extern slideway_t juice_slideway;
extern osMessageQId slideway_msg_queue_hdl;

static void servo_pwr_dwn(slideway_servo_t *ptr_servo)
{
  ptr_servo->motor.dir=NULL_DIR;
  ptr_servo->motor.driver.pwr_dwn();
}
static void servo_pwr_on_positive(slideway_servo_t *ptr_servo)
{
  ptr_servo->motor.driver.pwr_on_positive();
  ptr_servo->motor.dir=POSITIVE_DIR;
}
static void servo_pwr_on_negative(slideway_servo_t *ptr_servo)
{
  ptr_servo->motor.driver.pwr_on_negative();
  ptr_servo->motor.dir=NEGATIVE_DIR;
}
static uint8_t assistant_send_ctl_info(uint16_t type,uint8_t param1,uint8_t param2)
{
  ctl_info_t cmd;
  cmd.type=type;
  cmd.param8[0]=param1;
  cmd.param8[1]=param2;
  osMessagePut(slideway_msg_queue_hdl,*(uint32_t*)&cmd,0);
  return JUICE_TRUE;
}
static uint8_t reset_sensor_update_state(reset_sensor_t *ptr_sensor)
{
 uint8_t  sensor_value;
 uint32_t cur_time;
 sensor_value=ptr_sensor->sensor.driver.sensor_get_8value();
 cur_time=osKernelSysTick();
 if(sensor_value!=ptr_sensor->sensor.value)
 {
  ptr_sensor->sensor.start_time=cur_time;
  ptr_sensor->sensor.value=sensor_value;
 } 
 if(ptr_sensor->sensor.value==ptr_sensor->valid_value &&
   (cur_time-ptr_sensor->sensor.start_time >= ptr_sensor->min_hold_time))
 ptr_sensor->valid=JUICE_TRUE;  
 else
 ptr_sensor->valid=JUICE_FALSE;
 
 return JUICE_FALSE;
}
static uint8_t is_servo_on_reset_pos(slideway_servo_t *ptr_servo)
{
 uint8_t ret=JUICE_FALSE;
 APP_ASSERT(ptr_servo);
 if(ptr_servo->motor.active==JUICE_TRUE)
 {
   /*在伺服系统无效时，寻找检测复位点*/
   if(ptr_servo->active==JUICE_FALSE && \
      ptr_servo->reset_ms.active==JUICE_TRUE)
   {
    reset_sensor_update_state(&ptr_servo->reset_ms);
    if(ptr_servo->reset_ms.valid==JUICE_TRUE)
    {
    ptr_servo->reset_ms.active=JUICE_FALSE;
    ptr_servo->reset_ms.valid=JUICE_FALSE;
    ret=JUICE_TRUE;
    }
   }
 }
return ret; 
}

/*
计算实时速度，速度比等于路程比--2次曲线加速减速
*/
static uint8_t servo_calculate_real_time_pwr(slideway_servo_t *ptr_servo)
{
  uint32_t cur_pos=ptr_servo->encoder.cur;
  
  int8_t pwr;
  int8_t dir=1;
  if(ptr_servo->motor.dir==NULL_DIR)/*停机的时候功率输出为0*/
  {
    pwr=0;
    return pwr;
  }
  if(ptr_servo->motor.dir==NEGATIVE_DIR)
  dir=-1;
  APP_ASSERT(ptr_servo); 
  if(cur_pos*dir < ptr_servo->motor_ctl.acceleration_stop*dir)
  {
    pwr=(cur_pos-ptr_servo->motor_ctl.start)*dir*100/ptr_servo->motor_ctl.acceleration_cnt;
    if(pwr<0)
    {
     /*应该忽略启动抖动*/
     pwr=ptr_servo->motor_ctl.cur_pwr;
    }
    else
    {
     if(pwr>=ptr_servo->motor_ctl.max_pwr)
     pwr=ptr_servo->motor_ctl.max_pwr;
     else if(pwr < ptr_servo->motor_ctl.start_pwr)
     pwr=ptr_servo->motor_ctl.start_pwr;
     else if(ptr_servo->motor_ctl.cur_pwr+ptr_servo->motor_ctl.pwr_step > pwr)
     pwr=ptr_servo->motor_ctl.cur_pwr;
    }
  }
  else if(cur_pos*dir >= ptr_servo->motor_ctl.deceleration_start*dir)
  {
    pwr=(ptr_servo->motor_ctl.stop-cur_pos)*dir*100/ptr_servo->motor_ctl.deceleration_cnt; 
    if(pwr<0)
    {
    /*应该忽略启动抖动*/
    pwr=ptr_servo->motor_ctl.cur_pwr;
    }
    else
    {
     if(pwr>=ptr_servo->motor_ctl.max_pwr)
     pwr=ptr_servo->motor_ctl.max_pwr;
     else if(pwr < ptr_servo->motor_ctl.stop_pwr)
     pwr=ptr_servo->motor_ctl.stop_pwr;
     else if(ptr_servo->motor_ctl.cur_pwr < pwr+ptr_servo->motor_ctl.pwr_step)
     pwr=ptr_servo->motor_ctl.cur_pwr;   
    }
  }
  return pwr;
}
static uint8_t servo_calculate_real_time_pwr_dir(slideway_servo_t *ptr_servo)
{
 if(ptr_servo->motor.active==JUICE_TRUE)
 {
  /*到达误差范围内*/
   if(ptr_servo->motor.dir!=NULL_DIR && IS_SERVO_POS_EQUIVALENT(ptr_servo,ptr_servo->encoder.cur,ptr_servo->motor_ctl.stop))
   {
   servo_pwr_dwn(ptr_servo);
   }
   if(!IS_SERVO_POS_EQUIVALENT(ptr_servo,ptr_servo->encoder.cur,ptr_servo->motor_ctl.stop))/*如果在停车后，当前实时位置与停止点不一致，则再次启动*/
   {
   if(ptr_servo->encoder.cur<ptr_servo->motor_ctl.stop)
   {
   if(ptr_servo->motor.dir!=POSITIVE_DIR)
   servo_pwr_on_positive(ptr_servo); 
   if(ptr_servo->encoder.cur+ptr_servo->motor_ctl.tolerance<ptr_servo->motor_ctl.start)
   {
    APP_LOG_WARNING("当前位置小于start点，发送重新开始消息.\r\n");
    assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_RESTART,ptr_servo->id,0); 
   }
   }
   else 
   {
   if(ptr_servo->motor.dir!=NEGATIVE_DIR)
   servo_pwr_on_negative(ptr_servo);
   if(ptr_servo->encoder.cur > ptr_servo->motor_ctl.start+ptr_servo->motor_ctl.tolerance)
   {
    APP_LOG_WARNING("当前位置大于stop点，发送重新开始消息.\r\n");
    assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_RESTART,ptr_servo->id,0); 
   }
   }
   } 
 }
 return JUICE_TRUE;
}

static uint8_t is_need_to_send_arrive_msg(slideway_servo_t *ptr_servo)
{
 uint8_t ret=JUICE_FALSE;
 if(ptr_servo->arrive==JUICE_FALSE && ptr_servo->motor.dir==NULL_DIR)
 ret=JUICE_TRUE;
 return ret;
}

static uint8_t is_servo_pwr_update(slideway_servo_t *ptr_servo,uint8_t pwr)
{
 uint8_t ret=JUICE_FALSE;
 if(pwr!=ptr_servo->motor_ctl.cur_pwr)
 ret=JUICE_TRUE;
 return ret;
}
static uint8_t servo_update_pwr(slideway_servo_t *ptr_servo)
{
 uint8_t pwr;
 uint32_t max_pwr_value;
 pwr=servo_calculate_real_time_pwr(ptr_servo); 
 if(is_servo_pwr_update(ptr_servo,pwr)==JUICE_TRUE)
 {
  max_pwr_value=ptr_servo->motor_ctl.max_pwr_value;
  ptr_servo->motor_ctl.cur_pwr=pwr;
  ptr_servo->motor.driver.pwr_value(pwr*max_pwr_value/100);
  APP_LOG_INFO("更新功率比：%d %.功率值：%d\r\n",pwr,pwr*max_pwr_value/100);
 }
 return JUICE_TRUE;
}

static uint8_t slideway_assistant_detect_reset_pos(slideway_t *ptr_slideway)
{
  APP_ASSERT(ptr_slideway);
  if(is_servo_on_reset_pos(&ptr_slideway->vertical_servo)==JUICE_TRUE)
  {
  assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ARRIVE_RESET,VERTICAL_SERVO,0); 
  }
  if(is_servo_on_reset_pos(&ptr_slideway->horizontal_servo)==JUICE_TRUE)
  {
  assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ARRIVE_RESET,HORIZONTAL_SERVO,0);  
  }
  return JUICE_TRUE;
}


static uint8_t slideway_assistant_process_pwr_dir(slideway_t *ptr_slideway)
{
  /*垂直方向*/
  servo_calculate_real_time_pwr_dir(&ptr_slideway->vertical_servo);
  if(is_need_to_send_arrive_msg(&ptr_slideway->vertical_servo)==JUICE_TRUE)
  {
   assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ARRIVE_NORMAL,VERTICAL_SERVO,0);
   APP_LOG_INFO("发送垂直方向到达信息.\r\n");
  }
  servo_update_pwr(&ptr_slideway->vertical_servo);
  /*水平方向*/
  servo_calculate_real_time_pwr_dir(&ptr_slideway->horizontal_servo);
  if(is_need_to_send_arrive_msg(&ptr_slideway->horizontal_servo)==JUICE_TRUE)
  {
  assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ARRIVE_NORMAL,HORIZONTAL_SERVO,0);
  APP_LOG_INFO("发送水平方向到达信息.\r\n");
  }
  servo_update_pwr(&ptr_slideway->horizontal_servo);
  return JUICE_TRUE;
}

static uint8_t is_servo_velocity_error(slideway_servo_t *ptr_servo)
{
  uint8_t ret=JUICE_FALSE;
  uint32_t delta;
  uint32_t cur_time,cur_pos;
  
 if(ptr_servo->motor_v.active==JUICE_TRUE)
 {
  cur_time=osKernelSysTick();
  if(cur_time-ptr_servo->motor_v.last_time>=ptr_servo->motor_v.interval)
  {
  cur_pos=ptr_servo->encoder.cur;
  delta=cur_pos > ptr_servo->motor_v.last_sample?cur_pos-ptr_servo->motor_v.last_sample:ptr_servo->motor_v.last_sample-cur_pos;
  ptr_servo->motor_v.cur=delta*ptr_servo->motor_v.unit/(cur_time-ptr_servo->motor_v.last_time);
  ptr_servo->motor_v.last_time=cur_time;
  if(ptr_servo->motor_v.cur<=ptr_servo->motor_v.warning_min || ptr_servo->motor_v.cur>=ptr_servo->motor_v.warning_max)
  {
  ret=JUICE_TRUE;
  APP_LOG_WARNING("检测到堵转或者超速.当前速度：%d.",ptr_servo->motor_v.cur);
  }
  }
 }
 return ret;
}

static uint8_t slideway_assistant_process_error(slideway_t *ptr_slideway)
{
  if(is_servo_velocity_error(&ptr_slideway->vertical_servo)==JUICE_TRUE)
  {
   assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ERROR,VERTICAL_SERVO,FAULT_CODE_M_VERTICAL_MOTOR_STALL);
   APP_LOG_WARNING("向主控任务发送速度出错消息.");
  }
  if(is_servo_velocity_error(&ptr_slideway->horizontal_servo)==JUICE_TRUE)
  {
   assistant_send_ctl_info(SLIDEWAY_MSG_SERVO_ERROR,HORIZONTAL_SERVO,FAULT_CODE_M_HORIZONTAL_MOTOR_STALL);
   APP_LOG_WARNING("向主控任务发送速度出错消息.");
  }
 return JUICE_TRUE;
}


static void slideway_assistant_task(void const * argument)
{
 slideway_t *ptr_slideway=&juice_slideway;
 APP_LOG_DEBUG("########机械手辅助任务开始.");
 while(1)
 {
  slideway_assistant_detect_reset_pos(ptr_slideway); 
  slideway_assistant_process_pwr_dir(ptr_slideway);
  slideway_assistant_process_error(ptr_slideway);
  osDelay(SLIDEWAY_ASSISTANT_TASK_INTERVAL);
 } 
}