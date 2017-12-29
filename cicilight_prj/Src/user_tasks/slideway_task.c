#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "user_tasks.h"
#include "mb_reg.h"
#include "slideway_task.h"
#define APP_LOG_MODULE_NAME   "[main_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"



slideway_t juice_slideway;

extern osMessageQId sync_msg_queue_hdl;
extern osMessageQId slideway_msg_queue_hdl;
extern uint8_t juice_set_fault_code(uint8_t err_code);
extern uint8_t juice_get_fault_code();


static void slideway_param_init(slideway_t *ptr_slideway)
{
juice_pos_t *ptr_pos=&ptr_slideway->juice_pos;
/*位置参数*/
for(uint8_t i=0;i<VERTICAL_CNT;i++)
{
  for(uint8_t j=0;j<HORIZONTAL_CNT;j++)
  {
  /*初始化抓杯时停靠点坐标*/
  ptr_pos->top_array[i][j].vertical=VERTICAL_ENCODER_BASE_PULSES+(VERTICAL_CUP_BASE_MM+i*VERTICAL_CUP_CLEARANCE_MM+VERTICAL_DOCKSITE_CLEARANCE_MM)*VERTICAL_ENCODER_PULSES_PER_MM;
  ptr_pos->top_array[i][j].horizontal=HORIZONTAL_ENCODER_BASE_PULSES+(HORIZONTAL_ENCODER_RESOLUTION/HORIZONTAL_CUP_CNT)*j+(j==HORIZONTAL_CNT-1?2:0);
  /*初始化抓杯时停下降坐标*/
  ptr_pos->bot_array[i][j].vertical=ptr_pos->top_array[i][j].vertical -VERTICAL_PUT_DWN_MM*VERTICAL_ENCODER_PULSES_PER_MM;
  ptr_pos->bot_array[i][j].horizontal=ptr_pos->top_array[i][j].horizontal;
  /*初始化抓杯时停提升坐标*/
  ptr_pos->liftup_array[i][j].vertical=ptr_pos->bot_array[i][j].vertical+VERTICAL_LIFT_UP_MM*VERTICAL_ENCODER_PULSES_PER_MM;
  ptr_pos->liftup_array[i][j].horizontal=ptr_pos->top_array[i][j].horizontal;
  }
}
 ptr_pos->juicing.vertical=VERTICAL_ENCODER_BASE_PULSES+VERTICAL_JUICING_MM*VERTICAL_ENCODER_PULSES_PER_MM;
 ptr_pos->juicing.horizontal=HORIZONTAL_ENCODER_BASE_PULSES+HORIZONTAL_JUICING_MM*HORIZONTAL_ENCODER_PULSES_PER_MM;

 ptr_pos->slot.vertical=VERTICAL_ENCODER_BASE_PULSES+VERTICAL_SLOT_MM*VERTICAL_ENCODER_PULSES_PER_MM;
 ptr_pos->slot.horizontal=HORIZONTAL_ENCODER_BASE_PULSES+HORIZONTAL_SLOT_MM*HORIZONTAL_ENCODER_PULSES_PER_MM;

 ptr_pos->standby.vertical=VERTICAL_ENCODER_BASE_PULSES+VERTICAL_STANDBY_MM*VERTICAL_ENCODER_PULSES_PER_MM;
 ptr_pos->standby.horizontal=HORIZONTAL_ENCODER_BASE_PULSES+HORIZONTAL_STANDBY_MM*HORIZONTAL_ENCODER_PULSES_PER_MM;

 ptr_pos->reset.vertical=VERTICAL_ENCODER_BASE_PULSES+VERTICAL_RESET_MM*VERTICAL_ENCODER_PULSES_PER_MM-VERTICAL_RESET_PULSES;
 ptr_pos->reset.horizontal=HORIZONTAL_ENCODER_BASE_PULSES+HORIZONTAL_RESET_MM*HORIZONTAL_ENCODER_PULSES_PER_MM-HORIZONTAL_RESET_PULSES;
/*其他参数*/
 ptr_slideway->active=JUICE_FALSE;
 ptr_slideway->expect_arrives=0;
 
 ptr_slideway->vertical_servo.active=JUICE_FALSE;
 ptr_slideway->vertical_servo.arrive=JUICE_FALSE;
 ptr_slideway->vertical_servo.normal_pwr=VERTICAL_SERVO_NORMAL_PWR;
 ptr_slideway->vertical_servo.show_pwr=VERTICAL_SERVO_SHOW_PWR;
 ptr_slideway->vertical_servo.encoder.cur=VERTICAL_ENCODER_BASE_PULSES;
 ptr_slideway->vertical_servo.encoder.dir=NULL_DIR;
 ptr_slideway->vertical_servo.encoder.resolution=VERTICAL_ENCODER_RESOLUTION;
 ptr_slideway->vertical_servo.id=VERTICAL_SERVO_ID;
 ptr_slideway->vertical_servo.motor.active=JUICE_TRUE;
 ptr_slideway->vertical_servo.motor.dir=NULL_DIR;
 ptr_slideway->vertical_servo.motor_ctl.active=JUICE_TRUE;
 ptr_slideway->vertical_servo.motor_ctl.acceleration_cnt=VERTICAL_SERVO_ACCELERATION_PULSES_CNT;
 ptr_slideway->vertical_servo.motor_ctl.deceleration_cnt=VERTICAL_SERVO_DECELERATION_PULSES_CNT;
 ptr_slideway->vertical_servo.motor_ctl.tolerance=VERTICAL_SERVO_TOLERANCE_PULSES;
 ptr_slideway->vertical_servo.motor_ctl.max_pwr_value=VERTICAL_SERVO_MAX_VALUE;
 ptr_slideway->vertical_servo.motor_ctl.pwr_step=VERTICAL_SERVO_PWR_STEP;
 ptr_slideway->vertical_servo.motor_ctl.start=VERTICAL_ENCODER_BASE_PULSES;
 ptr_slideway->vertical_servo.motor_ctl.tar=VERTICAL_ENCODER_BASE_PULSES;
 ptr_slideway->vertical_servo.motor_ctl.stop=VERTICAL_ENCODER_BASE_PULSES;
 ptr_slideway->vertical_servo.motor_ctl.max_pwr=ptr_slideway->vertical_servo.normal_pwr;
 ptr_slideway->vertical_servo.motor_v.active=JUICE_FALSE;
 ptr_slideway->vertical_servo.motor_v.delay=VERTICAL_SERVO_MONITOR_DELAY;
 ptr_slideway->vertical_servo.motor_v.unit= VERTICAL_SERVO_MONITOR_UNIT;
 ptr_slideway->vertical_servo.motor_v.warning_max=VERTICAL_SERVO_MONITOR_MAX_WARNING;
 ptr_slideway->vertical_servo.motor_v.warning_min=VERTICAL_SERVO_MONITOR_MIN_WARNING;
}


/*计算极限刹车点*/
static uint32_t servo_calculate_limit_brake_pos(slideway_servo_t *ptr_servo)
{
  int8_t delta_pwr,dir=1;
  uint32_t brake_dis,brake_pos;
  /*计算刹车距离*/
  if(ptr_servo->motor_ctl.cur_pwr <= ptr_servo->motor_ctl.stop_pwr)
  brake_dis=0;
  else
  {
  delta_pwr=ptr_servo->motor_ctl.cur_pwr-ptr_servo->motor_ctl.stop_pwr;
  brake_dis=delta_pwr*ptr_servo->motor_ctl.deceleration_cnt/100;
  }
  /*计算刹车点*/
  if(ptr_servo->motor.dir==NEGATIVE_DIR)//反转
  dir=-1;
  brake_pos=ptr_servo->encoder.cur+brake_dis*dir;  
  return brake_pos;
}
/*计算停车点*/
static uint8_t servo_calculate_stop_pos(slideway_servo_t *ptr_servo,uint32_t brake_pos)
{
  int8_t dir=1;
  if(ptr_servo->motor.dir==NEGATIVE_DIR)//反转
  dir=-1;
  if(brake_pos*dir < ptr_servo->motor_ctl.tar*dir)
  {
   ptr_servo->motor_ctl.stop= ptr_servo->motor_ctl.tar;
   APP_LOG_INFO("设置停止点==目标点：%d.\r\n",ptr_servo->motor_ctl.stop);
  }
  else
  {
   ptr_servo->motor_ctl.stop= brake_pos;
   APP_LOG_INFO("设置停止点!=目标点：%d.\r\n",ptr_servo->motor_ctl.stop);
  } 
  return JUICE_TRUE;
}
/*计算加减速点*/
static uint8_t servo_calculate_acc_dec_pos(slideway_servo_t *ptr_servo)
{
  uint8_t dir=1;
  uint32_t dis,dis_acc_dec;
  if(ptr_servo->motor.dir==NEGATIVE_DIR)
  dir=-1;
  
  dis=(ptr_servo->motor_ctl.stop-ptr_servo->motor_ctl.start)*dir;
  dis_acc_dec=ptr_servo->motor_ctl.acceleration_cnt+ptr_servo->motor_ctl.deceleration_cnt;
  if(dis > dis_acc_dec)
  {
   ptr_servo->motor_ctl.acceleration_stop=ptr_servo->motor_ctl.start+ptr_servo->motor_ctl.acceleration_cnt*dir;
   ptr_servo->motor_ctl.deceleration_start=ptr_servo->motor_ctl.stop-ptr_servo->motor_ctl.deceleration_cnt*dir;
  }
  else
  {
   ptr_servo->motor_ctl.acceleration_stop=ptr_servo->motor_ctl.start+((ptr_servo->motor_ctl.acceleration_cnt*dis/dis_acc_dec)*dir);
   ptr_servo->motor_ctl.deceleration_start=ptr_servo->motor_ctl.acceleration_stop;
  }
  APP_LOG_INFO("计算的加速停止点为：%d ;减速开始点为：%d.\r\n",ptr_servo->motor_ctl.acceleration_stop,ptr_servo->motor_ctl.deceleration_start);
  return JUICE_TRUE;
}
/*
 由循环位置计算连续位置
*/
static uint32_t slideway_calculate_horizontal_pos(slideway_t *ptr_slideway,uint32_t rotary_pos)
{
  uint32_t base_pos,pre_pos,next_pos,brake_pos;
  APP_ASSERT(ptr_slideway);
  
  base_pos=ptr_slideway->horizontal_servo.encoder.cur/ptr_slideway->horizontal_servo.encoder.resolution*ptr_slideway->horizontal_servo.encoder.resolution;
  pre_pos=rotary_pos+base_pos;
  next_pos=pre_pos+ptr_slideway->horizontal_servo.encoder.resolution; 
  brake_pos=servo_calculate_limit_brake_pos(&ptr_slideway->horizontal_servo);
  if(brake_pos > (pre_pos+next_pos)/2)
  {
   APP_LOG_INFO("水平伺服计算的目标点next_pos：%d.\r\n",next_pos); 
   return next_pos;
  }
  APP_LOG_INFO("水平伺服计算的目标点pre_pos：%d.\r\n",pre_pos);
  return pre_pos;
}
/*从从控制信息里取位置标签*/
static uint8_t get_pos_tag_from_ctl_info(ctl_info_t *ptr_cmd,uint8_t *ptr_v,uint8_t *ptr_h)
{    
  *ptr_v=ptr_cmd->param8[0]-1;
  *ptr_h=ptr_cmd->param8[1]-1; 
  return JUICE_TRUE;
}
static uint8_t servo_calculate_process_ctl_pos(slideway_servo_t *ptr_servo)
{
  uint32_t lb_pos;
  APP_ASSERT(ptr_servo);
  /*计算伺服当前速度下的极限刹车点*/
  lb_pos=servo_calculate_limit_brake_pos(ptr_servo);
  /*计算伺服当前速度下的实际停车点*/
  servo_calculate_stop_pos(ptr_servo,lb_pos);
  /*计算伺服当前速度下的加速点和减速点*/
  servo_calculate_acc_dec_pos(ptr_servo);  
  return JUICE_TRUE;
}
static uint8_t servo_start(slideway_servo_t *ptr_servo,uint32_t pos)
{  
  /*设置伺服系统目标点*/
  ptr_servo->motor_ctl.tar=pos;
  /*计算需要的过程控制点*/
  servo_calculate_process_ctl_pos(ptr_servo);
  return JUICE_TRUE; 
}

static uint8_t  slideway_start(slideway_t *ptr_slideway,uint32_t vertical,uint32_t horizontal)
{ 
  APP_ASSERT(ptr_slideway);
  /*处理垂直方向伺服系统参数*/
  servo_start(&ptr_slideway->vertical_servo,vertical);
  /*处理水平方向伺服系统参数*/
  servo_start(&ptr_slideway->horizontal_servo,horizontal); 
  return JUICE_TRUE;
}
static uint8_t slideway_clear_arrives(slideway_t *ptr_slideway)
{
 APP_ASSERT(ptr_slideway); 
 ptr_slideway->expect_arrives=0;
 ptr_slideway->vertical_servo.arrive=JUICE_FALSE;
 ptr_slideway->horizontal_servo.arrive=JUICE_FALSE;
 return JUICE_TRUE;
}
static uint8_t slideway_expect_arrives(slideway_t *ptr_slideway,uint8_t arrives)
{
 APP_ASSERT(ptr_slideway); 
 slideway_clear_arrives(ptr_slideway);
 ptr_slideway->expect_arrives=arrives; 
 return JUICE_TRUE;
}
static uint8_t is_slideway_arrived(slideway_t *ptr_slideway)
{
 uint8_t ret=JUICE_FALSE;
 uint8_t arrives=0;
 APP_ASSERT(ptr_slideway); 
 if(ptr_slideway->vertical_servo.arrive==JUICE_TRUE)
  arrives|=VERTICAL_SERVO_ID;
 if(ptr_slideway->horizontal_servo.arrive==JUICE_TRUE)
  arrives|=HORIZONTAL_SERVO_ID;
 if(arrives ^ ptr_slideway->expect_arrives==0)
 ret=JUICE_TRUE;
 return ret;
}
static uint8_t slideway_get_cup_top_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd,uint32_t *ptr_v,uint32_t *ptr_h)
{
  uint8_t v_tag,h_tag;
  get_pos_tag_from_ctl_info(ptr_cmd,&v_tag,&h_tag); 
  /*重新计算水平位置*/
 *ptr_h=slideway_calculate_horizontal_pos(ptr_slideway,ptr_slideway->juice_pos.top_array[v_tag][h_tag].horizontal);
 *ptr_v=ptr_slideway->juice_pos.top_array[v_tag][h_tag].vertical;
 return JUICE_TRUE;
}
static uint8_t slideway_get_cup_bot_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd,uint32_t *ptr_v,uint32_t *ptr_h)
{
  uint8_t v_tag,h_tag;
  get_pos_tag_from_ctl_info(ptr_cmd,&v_tag,&h_tag); 
  /*重新计算水平位置*/
  *ptr_h=slideway_calculate_horizontal_pos(ptr_slideway,ptr_slideway->juice_pos.bot_array[v_tag][h_tag].horizontal);
  *ptr_v=ptr_slideway->juice_pos.bot_array[v_tag][h_tag].vertical;
  return JUICE_TRUE;
}
static uint8_t slideway_get_cup_lift_up_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd,uint32_t *ptr_v,uint32_t *ptr_h)
{
  uint8_t v_tag,h_tag;
  get_pos_tag_from_ctl_info(ptr_cmd,&v_tag,&h_tag); 
  /*重新计算水平位置*/
  *ptr_h=slideway_calculate_horizontal_pos(ptr_slideway,ptr_slideway->juice_pos.liftup_array[v_tag][h_tag].horizontal);
  *ptr_v=ptr_slideway->juice_pos.liftup_array[v_tag][h_tag].vertical;
  return JUICE_TRUE;
}
static uint8_t servo_set_normal_max_pwr(slideway_servo_t *ptr_servo)
{
 APP_ASSERT(ptr_servo);
 ptr_servo->motor_ctl.max_pwr=ptr_servo->normal_pwr;
 return JUICE_TRUE;
}
static uint8_t servo_set_show_max_pwr(slideway_servo_t *ptr_servo)
{
 APP_ASSERT(ptr_servo);
 ptr_servo->motor_ctl.max_pwr=ptr_servo->show_pwr;
 return JUICE_TRUE;
}

static uint8_t slideway_get_juicing_pos(slideway_t *ptr_slideway,uint32_t *ptr_v,uint32_t *ptr_h)
{
  *ptr_h=ptr_slideway->juice_pos.juicing.horizontal;
  *ptr_v=ptr_slideway->juice_pos.juicing.vertical;
  return JUICE_TRUE;
}
static uint8_t slideway_get_slot_pos(slideway_t *ptr_slideway,uint32_t *ptr_v,uint32_t *ptr_h)
{
  *ptr_h=ptr_slideway->juice_pos.slot.horizontal;
  *ptr_v=ptr_slideway->juice_pos.slot.vertical;
  return JUICE_TRUE;
}

static uint8_t slideway_get_standby_pos(slideway_t *ptr_slideway,uint32_t *ptr_v,uint32_t *ptr_h)
{
  *ptr_h=ptr_slideway->juice_pos.standby.horizontal;
  *ptr_v=ptr_slideway->juice_pos.standby.vertical;
  return JUICE_TRUE;
}
static uint8_t slideway_get_reset_pos(slideway_t *ptr_slideway,uint32_t *ptr_v,uint32_t *ptr_h)
{
  *ptr_h=ptr_slideway->juice_pos.reset.horizontal;
  *ptr_v=ptr_slideway->juice_pos.reset.vertical;
  return JUICE_TRUE;
}

static uint8_t slideway_process_cup_top_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID|HORIZONTAL_SERVO_ID);
  servo_set_normal_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_cup_top_pos(ptr_slideway,ptr_cmd,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
static uint8_t slideway_process_cup_bot_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID|HORIZONTAL_SERVO_ID);
  servo_set_normal_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_cup_bot_pos(ptr_slideway,ptr_cmd,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
static uint8_t slideway_process_lift_up_cup_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID|HORIZONTAL_SERVO_ID);
  servo_set_normal_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_cup_lift_up_pos(ptr_slideway,ptr_cmd,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}

static uint8_t slideway_process_standby_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID);
  servo_set_show_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_standby_pos(ptr_slideway,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
static uint8_t slideway_process_juicing_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID);
  servo_set_show_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_juicing_pos(ptr_slideway,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
static uint8_t slideway_process_slot_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID);
  servo_set_show_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_slot_pos(ptr_slideway,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
static uint8_t slideway_process_reset_pos(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint32_t vertical,horizontal;
  /*无效相关标志*/
  ptr_slideway->active=JUICE_FALSE;
  ptr_slideway->vertical_servo.active=JUICE_FALSE;
  ptr_slideway->horizontal_servo.active=JUICE_FALSE;
  slideway_expect_arrives(ptr_slideway,VERTICAL_SERVO_ID);
  servo_set_normal_max_pwr(&ptr_slideway->horizontal_servo);
  slideway_get_reset_pos(ptr_slideway,&vertical,&horizontal);
  slideway_start(ptr_slideway,vertical,horizontal);
  return JUICE_TRUE;
}
/*
 重新开始。同步起始点和当前位置点，由于位置错误或者到达stop点。
*/
static uint8_t servo_restart(slideway_servo_t *ptr_servo)
{
 ptr_servo->motor_ctl.start=ptr_servo->encoder.cur;
 servo_calculate_process_ctl_pos(ptr_servo);
 return JUICE_TRUE;
}

static uint8_t get_servo_id_from_ctl_info(ctl_info_t *ptr_cmd)
{
 return ptr_cmd->param8[0];
}

static uint8_t  slideway_process_servo_arrive_normal(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint8_t servo_tag;
  APP_ASSERT(ptr_slideway);
  APP_ASSERT(ptr_cmd);
  servo_tag=get_servo_id_from_ctl_info(ptr_cmd);
  if(servo_tag == VERTICAL_SERVO_ID)
  {
  /*如果当前停止点就是目标点，则到达。*/
  if(ptr_slideway->vertical_servo.motor_ctl.stop==ptr_slideway->vertical_servo.motor_ctl.tar)
  {
  ptr_slideway->vertical_servo.arrive=JUICE_TRUE;
  APP_LOG_INFO("垂直方向伺服到达.\r\n");
  }
  else
  {
  APP_LOG_INFO("垂直方向伺服临时到达，准备再次启动.\r\n");
  servo_restart(&ptr_slideway->vertical_servo); 
  }
  }
  else if(servo_tag == HORIZONTAL_SERVO_ID)
  {
  if(ptr_slideway->horizontal_servo.motor_ctl.stop==ptr_slideway->horizontal_servo.motor_ctl.tar)
  {
  ptr_slideway->horizontal_servo.arrive=JUICE_TRUE;
  APP_LOG_INFO("水平方向伺服到达.\r\n");
  }
  else
  {
  APP_LOG_INFO("水平方向伺服临时到达，准备再次启动.\r\n");
  servo_restart(&ptr_slideway->vertical_servo);  
  }
  }
  if(is_slideway_arrived(ptr_slideway))
  {
   APP_LOG_INFO("所有方向伺服结束.\r\n");
   /*向榨汁任务发送消息到达指定位置*/
   osMessagePut(sync_msg_queue_hdl,MANIPULATOR_REACH_POS_OK_SIGNAL,0);
  }
  return JUICE_TRUE;  
}

/*开始刹车*/
static uint8_t servo_start_brake(slideway_servo_t *ptr_servo)
{
 uint32_t lb_pos; 
 lb_pos=servo_calculate_limit_brake_pos(ptr_servo);
 /*把极限刹车点当做目标点*/
 servo_start(ptr_servo,lb_pos);
 return JUICE_TRUE;
}
static uint8_t get_fault_code_from_ctl_info(ctl_info_t *ptr_cmd)
{
 return ptr_cmd->param8[1];
}

static uint8_t slideway_process_servo_arrive_reset(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint8_t servo_tag;
  APP_ASSERT(ptr_slideway);
  APP_ASSERT(ptr_cmd);
  servo_tag=get_servo_id_from_ctl_info(ptr_cmd);
  if(servo_tag == VERTICAL_SERVO_ID)
  {
  APP_LOG_WARNING("垂直方向到达复位点.\r\n");
  ptr_slideway->vertical_servo.active=JUICE_TRUE;
  }
  else if(servo_tag == HORIZONTAL_SERVO_ID)
  {
  APP_LOG_WARNING("水平方向到达复位点.\r\n");
  ptr_slideway->horizontal_servo.active=JUICE_TRUE;
  }
  if(ptr_slideway->vertical_servo.active  ==JUICE_TRUE  && \
     ptr_slideway->horizontal_servo.active==JUICE_TRUE)
  {
   APP_LOG_WARNING("所有方向到达复位点.向榨汁任务发送到达复位点消息.\r\n");
   ptr_slideway->active=JUICE_TRUE;
   /*水平方向开始刹车*/
   APP_LOG_WARNING("水平方向开始刹车.\r\n");
   servo_start_brake(&ptr_slideway->horizontal_servo);
   osMessagePut(sync_msg_queue_hdl,MANIPULATOR_RESET_OK_SIGNAL,0);
  } 
  return JUICE_TRUE;  
}

static uint8_t slideway_process_servo_error(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint8_t fault_code;
  uint8_t servo_tag;
  slideway_servo_t *ptr_servo_err=NULL;
  slideway_servo_t *ptr_servo_ok=NULL;
  APP_ASSERT(ptr_slideway);
  APP_ASSERT(ptr_cmd);
  fault_code=get_fault_code_from_ctl_info(ptr_cmd);
  servo_tag=get_servo_id_from_ctl_info(ptr_cmd);
  juice_set_fault_code(fault_code);
  APP_LOG_ERROR("故障电机类型：%d  故障码：%d \r\n",servo_tag,fault_code);
  
  if(servo_tag == VERTICAL_SERVO_ID)
  {
  ptr_servo_err=&ptr_slideway->vertical_servo;
  ptr_servo_ok =&ptr_slideway->horizontal_servo;
  }
  else if(servo_tag == HORIZONTAL_SERVO_ID)
  {
  ptr_servo_err=&ptr_slideway->horizontal_servo; 
  ptr_servo_ok=&ptr_slideway->vertical_servo; 
  }
  APP_ASSERT(ptr_servo_err);
  APP_ASSERT(ptr_servo_ok);
  /*出错的伺服马达立即无效*/
  ptr_servo_err->motor.active=JUICE_FALSE;
  /*没有出错的伺服立即刹车*/
  servo_start_brake(ptr_servo_ok); 
  /*向榨汁任务发送到达错误消息*/
  osMessagePut(sync_msg_queue_hdl,MANIPULATOR_REACH_POS_ERR_SIGNAL,0);
  return JUICE_TRUE;
}
/*机械手开始刹车*/
static uint8_t slideway_start_brake(slideway_t *ptr_slideway)
{
 APP_ASSERT(ptr_slideway); 
 servo_start_brake(&ptr_slideway->vertical_servo); 
 servo_start_brake(&ptr_slideway->horizontal_servo);
return JUICE_TRUE; 
}
static uint8_t slideway_process_servo_restart(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  uint8_t servo_id;
  APP_ASSERT(ptr_slideway);
  APP_ASSERT(ptr_cmd);
  servo_id=get_servo_id_from_ctl_info(ptr_cmd);
  if(servo_id==VERTICAL_SERVO_ID)
  servo_restart(&ptr_slideway->vertical_servo);
  else if(servo_id==HORIZONTAL_SERVO_ID)
  servo_restart(&ptr_slideway->horizontal_servo);
  return JUICE_TRUE;
}
static uint8_t slideway_process_stop(slideway_t *ptr_slideway,ctl_info_t *ptr_cmd)
{
  APP_ASSERT(ptr_slideway);
  APP_ASSERT(ptr_cmd);
  slideway_start_brake(ptr_slideway);
  return JUICE_TRUE;
}

/*
//快速开平方
float my_sqrt(float x)
{
 float xhalf = 0.5f*x;
 int i = *(int*)&x; //get bits for floating VALUE 
 i = 0x5f375a86 - (i >> 1); // gives initial guess y0
 x = *(float*)&i; // convert bits BACK to float
 x = x*(1.5f - xhalf*x*x); // Newton step, repeating increases accuracy
 return 1/x;  
}
*/

static void slideway_task(void const * argument)
{
  osEvent msg;
  ctl_info_t cmd;
 /*uint32_t vertical,horizontal;*/
 APP_LOG_INFO("++++++机械滑台和旋转任务开始！\r\n"); 
 while(1)
 {
 msg=osMessageGet(slideway_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage )
 continue;
 cmd=*(ctl_info_t*)&msg.value.v;
 switch(cmd.type)
 {
 case SLIDEWAY_MSG_GOTO_RESET:
   APP_LOG_WARNING("机械手收到复位位置命令.\r\n");
   slideway_process_reset_pos(&juice_slideway,&cmd);  
   break;
 case SLIDEWAY_MSG_GOTO_STANDBY:
   APP_LOG_WARNING("机械手收到待机位置命令.\r\n");
   slideway_process_standby_pos(&juice_slideway,&cmd);  
   break;
 case SLIDEWAY_MSG_GOTO_JUICING:
   APP_LOG_WARNING("机械手收到榨汁口位置命令.\r\n");
   slideway_process_juicing_pos(&juice_slideway,&cmd);  
   break;
 case SLIDEWAY_MSG_PUT_INTO_SLOT:
   APP_LOG_WARNING("机械手收到榨汁口底部位置命令.\r\n");
   slideway_process_slot_pos(&juice_slideway,&cmd); 
   break;
 case SLIDEWAY_MSG_GOTO_CUP_TOP:  
   APP_LOG_WARNING("机械手收到果杯上方位置命令.\r\n");
   slideway_process_cup_top_pos(&juice_slideway,&cmd); 
   break;
 case SLIDEWAY_MSG_GOTO_CUP_BOT:
   APP_LOG_WARNING("机械手收到果杯底部位置命令.\r\n");
   slideway_process_cup_bot_pos(&juice_slideway,&cmd); 
   break;
 case SLIDEWAY_MSG_LIFT_UP_CUP:
   APP_LOG_WARNING("机械手收到果杯提升位置命令！\r\n");
   slideway_process_lift_up_cup_pos(&juice_slideway,&cmd); 
   break;     
 case SLIDEWAY_MSG_SERVO_ARRIVE_NORMAL:
   APP_LOG_INFO("机械手收到到达消息.\r\n");
   slideway_process_servo_arrive_normal(&juice_slideway,&cmd);
   break;
  case SLIDEWAY_MSG_SERVO_ARRIVE_RESET:
   APP_LOG_WARNING("机械手收到复位点消息.\r\n");
   slideway_process_servo_arrive_reset(&juice_slideway,&cmd);
   break;
 case SLIDEWAY_MSG_SERVO_ERROR:
   APP_LOG_ERROR("机械手收到速度出错消息.\r\n");
   slideway_process_servo_error(&juice_slideway,&cmd);
   break;
 case SLIDEWAY_MSG_SERVO_RESTART:
   APP_LOG_ERROR("机械手收到速度出错消息.\r\n");
   slideway_process_servo_restart(&juice_slideway,&cmd);
   break;
 case SLIDEWAY_MSG_STOP:
   APP_LOG_ERROR("机械手收到停止消息.\r\n");
   slideway_process_stop(&juice_slideway,&cmd);
   break;
 default:
   APP_LOG_WARNING("机械手任务传递了错误参数！\r\n");
 }                  
}
}