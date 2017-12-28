#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"    
#include "gpio.h"
#include "tim.h"
#include "adc.h"
#include "mb_reg.h"
#include "juice_common.h"
#include "JJDK_ZK_GZ1.h"
#include "user_tasks.h"
#include "rgb_led.h"
#include "mb.h"
#include "app_error.h"
#if defined(IO_CHECK_TASK_ENABLE) && IO_CHECK_TASK_ENABLE >0
#include "io_check_task.h"
#endif
#define APP_LOG_MODULE_NAME   "[main_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"

 //任务的handle;
osThreadId sync_task_hdl;   
osThreadId adc_task_hdl;
osThreadId rgb_led_task_hdl;
osThreadId running_led_task_hdl;
osThreadId oh_door_task_hdl;
osThreadId servo1_task_hdl;
osThreadId servo2_task_hdl;
osThreadId manipulator_task_hdl;
osThreadId presser_task_hdl;
osThreadId compressor_task_hdl;
osThreadId juice_task_hdl;
osThreadId temperature_task_hdl;
osThreadId modbus_task_hdl;
#if defined(IO_CHECK_TASK_ENABLE) && IO_CHECK_TASK_ENABLE >0
osThreadId io_check_task_hdl;
#endif

//消息队列id
osMessageQId sync_msg_queue_hdl;
osMessageQId rgb_led_msg_queue_hdl;
osMessageQId running_led_msg_queue_hdl;
osMessageQId adc_msg_queue_hdl;
osMessageQId oh_door_msg_queue_hdl;
osMessageQId manipulator_msg_queue_hdl;
osMessageQId servo1_msg_queue_hdl;
osMessageQId servo2_msg_queue_hdl;
osMessageQId presser_msg_queue_hdl;
osMessageQId compressor_msg_queue_hdl;
osMessageQId juice_msg_queue_hdl;


//定时器id
osTimerId SLAVE_MB_timer_hdl;

//信号量
osMutexId err_code_mutex_id;


uint8_t juice_set_fault_code(uint8_t err_code)
{
  uint8_t ret=JUICE_TRUE;
  osStatus status;
  uint16_t reg_value;
  
  status=osMutexWait(err_code_mutex_id,ERR_CODE_MUTEX_TIMEOUT_VALUE);//获取设置错误码的信号量
  if(status==osOK)
  {
  get_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR,1,&reg_value,REGHOLDING_MODE);
  reg_value&=0xFF00;
  reg_value|=err_code;
  set_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR,1,&reg_value,REGHOLDING_MODE); 
  osMutexRelease(err_code_mutex_id);
  APP_LOG_INFO("获取设置错误码信号量成功，并设置了错误码！\r\n");
  }
  else
  {
  ret=JUICE_FALSE;
  APP_LOG_ERROR("获取设置错误码信号量失败，设置错误码失败！\r\n"); 
  }
 
  return ret;
}
/*
 * @brief -- 
 * @param -- 
 * @return -- 
 * @details --
 * @see --
*/

uint8_t juice_get_fault_code()
{
  uint16_t err_code;
  get_reg_value( JUICE_FAULT_CODE_REGHOLDING_ADDR,1,&err_code,REGHOLDING_MODE); 
  err_code&=0x00FF;
  return err_code;
}



static uint8_t juice_set_progress(uint8_t stage)//设置榨汁进度值
{
 uint16_t reg_value;
 osStatus status;
 uint8_t ret=JUICE_TRUE;
 status=osMutexWait(err_code_mutex_id,ERR_CODE_MUTEX_TIMEOUT_VALUE);//获取设置错误码的信号量
 if(status==osOK)
 {
 get_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR,1,&reg_value, REGHOLDING_MODE);
 reg_value&=0x00FF;//清除进度值
 reg_value|=(stage<<8);//赋值新的进度值
 set_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR,1,&reg_value,REGHOLDING_MODE); 
 osMutexRelease(err_code_mutex_id);
 APP_LOG_INFO("获取设置错误码信号量成功，并设置了进度值！\r\n");
 }
 else
 {
 ret=JUICE_FALSE;
 APP_LOG_ERROR("获取设置错误码信号量失败，设置进度值失败！\r\n"); 
 }
 
 return ret;
}

static void juice_all_completed()
{
 APP_LOG_INFO("设置进度为全部流程结束，空闲中！\r\n"); 
 juice_set_progress(PROGRESS_ALL_COMPLETED);  
}
static void juice_transaction_excuting()
{
 APP_LOG_INFO("设置进度为正在执行中！\r\n"); 
 juice_set_progress(PROGRESS_TRANSACTION_EXECUTING); 
}
static void juice_transaction_fault()
{
 APP_LOG_ERROR("设置进度为榨汁未完成，交易失败！\r\n"); 
 juice_set_progress(PROGRESS_TRANSACTION_FAULT);
}
static void juice_transaction_completed()
{ 
 APP_LOG_INFO("设置进度为榨汁完成，交易成功！\r\n"); 
 juice_set_progress(PROGRESS_TRANSACTION_COMPLETED);
}
static void juice_fault_after_transaction_completed()
{
 uint16_t err_code;
 juice_set_progress(PROGRESS_FAULT_AFTER_TRANSACTION_COMPLETED);
 err_code=juice_get_fault_code();
 APP_LOG_ERROR("设置进度为榨汁完成交易成功后，后续流程出错！\r\n"); 
 APP_LOG_ERROR("当前的错误码为：0x%2X！\r\n",err_code); 
}

static uint8_t juice_get_operation_param(uint8_t *ptr_row_pos,uint8_t *ptr_column_pos,uint16_t *ptr_opt)
{
  uint16_t pos,opt;
  uint8_t row_pos,column_pos;
  
  if(ptr_row_pos == NULL ||ptr_column_pos == NULL || ptr_opt == NULL)
  return  JUICE_FALSE;
  if(get_reg_value(JUICE_POS_REGHOLDING_ADDR,1,&pos,REGHOLDING_MODE)==JUICE_FALSE)
  return JUICE_FALSE;
  if(get_reg_value(OPERATION_TYPE_REGHOLDING_ADDR,1,&opt,REGHOLDING_MODE)==JUICE_FALSE)
  return JUICE_FALSE;
  row_pos=pos>>8;
  column_pos=pos&0xff;
  
 if(opt==REG_VALUE_OPERATION_TYPE_SHOW)
 {
  APP_LOG_INFO("开始机械手表演动作！目标位置x：%d y：%d.\r\n",row_pos,column_pos); 
 }
 else
 {
  APP_LOG_INFO("开始机械手抓杯榨汁动作！目标位置x：%d y：%d.\r\n",row_pos,column_pos);  
 }
 *ptr_row_pos=row_pos;
 *ptr_column_pos=column_pos;
 *ptr_opt=opt;
 
 return JUICE_TRUE;
}

osEvent juice_wait_signals(int32_t wait_signals,uint32_t timeout)
{
 osEvent signals;
 uint32_t pre_time,next_time;
 
 pre_time=osKernelSysTick();
 next_time=pre_time;
 while(next_time-pre_time<timeout)
 {
 signals=osSignalWait(SYNC_ALL_SIGNALS,timeout-(next_time-pre_time));
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & wait_signals))
 return signals;
 next_time=osKernelSysTick();
 }
 signals.status=osEventTimeout;
 return signals;
}


static void main_task(void const * argument)
{
 uint8_t row_sensor_pos,column_sensor_pos;
 uint16_t opt;
 uint32_t cup_timeout;
 osEvent msg,signals;
 
 APP_LOG_INFO("++++++榨汁同步任务开始！\r\n");
 while(1)
 {
 APP_LOG_INFO("待机，发送彩灯黄色常亮消息！\r\n");
 osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_STANDBY_MSG,0);
 APP_LOG_INFO("榨汁同步任务空闲中...准备睡眠，等待榨汁信号！\r\n");
 msg= osMessageGet(sync_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage || (msg.status==osEventMessage && msg.value.v!=SYNC_START_MSG))
 {
 continue ;
 } 
 juice_wait_signals(SYNC_ALL_SIGNALS,10);//清除所有信号
 
 if(juice_get_operation_param(&row_sensor_pos,&column_sensor_pos,&opt)==JUICE_FALSE)
 {
 juice_transaction_fault(); 
 APP_LOG_ERROR("错误，获取运行参数错误！\r\n");
 continue;
 }
 juice_transaction_excuting();
 if(juice_get_fault_code())//如果有错误码主要是温度错误和榨汁口有未拿走的果杯！
 {
 juice_transaction_fault();
 continue;
 }
 APP_LOG_INFO("运行，发送彩灯绿色常亮消息！\r\n");
 osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_MOTION_MSG,0); 
 
 APP_LOG_INFO("检测并准备关闭升降门！\r\n");
 osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_CLOSE_MSG,0);//关闭升降门
 signals=juice_wait_signals(OH_DOOR_REACH_BOT_POS_OK_SIGNAL|OH_DOOR_REACH_BOT_POS_ERR_SIGNAL,OH_DOOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & OH_DOOR_REACH_BOT_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_OH_DOOR_DWN_TIMEOUT);
 APP_LOG_ERROR("错误！关闭升降门超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！关闭升降门失败！\r\n");  
 } 
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到关闭升降门到位信号！\r\n");

 APP_LOG_INFO("机械手爪子复位抓紧！\r\n");
 osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_CLOSE_MSG,0);
 signals=juice_wait_signals(SERVO1_REACH_POS_OK_SIGNAL,SERVO1_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手爪子抓紧信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手爪子复位到位信号！\r\n");

 APP_LOG_INFO("机械手手臂复位到90°！\r\n");
 osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_90_MSG,0);
 signals=juice_wait_signals(SERVO2_REACH_POS_OK_SIGNAL,SERVO2_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手手臂转到90°信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手手臂复位90°到位信号！\r\n");
 
 APP_LOG_INFO("向果汁杯杯身上方运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,row_sensor_pos<<8|column_sensor_pos,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);
 APP_LOG_ERROR("错误！向果汁杯上方运动超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！向果汁杯上方运动失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");
 if(opt==REG_VALUE_OPERATION_TYPE_SHOW)//如果是机械手运动表演，就完成到这一步。
 {
 juice_all_completed();
 APP_LOG_INFO("机械手表演到达位置！\r\n");
 continue;
 }
 
 APP_LOG_INFO("机械手手臂转到角度25°！\r\n");
 osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_25_MSG,0);
 juice_wait_signals(SERVO2_REACH_POS_OK_SIGNAL,SERVO2_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手手臂转到25°信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手手臂25°到位信号！\r\n");
 APP_LOG_INFO("机械手爪子张开！\r\n");
 osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_OPEN_MSG,0);
 juice_wait_signals(SERVO1_REACH_POS_OK_SIGNAL,SERVO1_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手爪子张开信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手爪子张开到位信号！\r\n");
 APP_LOG_INFO("向果汁杯杯身中间运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,(row_sensor_pos-2)<<8|column_sensor_pos,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);
 APP_LOG_ERROR("错误！向果汁杯杯身中间运动超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！向果汁杯杯身中间运动失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");
 APP_LOG_INFO("机械手爪子抓紧杯子！\r\n");
 osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_CLOSE_MSG,0);
 juice_wait_signals(SERVO1_REACH_POS_OK_SIGNAL,SERVO1_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手爪子抓紧杯子信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手爪子抓紧到位信号！\r\n");
 APP_LOG_INFO("提起抓住的果汁杯！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,row_sensor_pos<<8|column_sensor_pos,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);//内部系统错误
 APP_LOG_ERROR("错误！提起抓住的果汁杯超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！提起抓住的果汁杯失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");
 APP_LOG_INFO("机械手手臂转到角度90°！\r\n");
 osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_90_MSG,0);
 juice_wait_signals(SERVO2_REACH_POS_OK_SIGNAL,SERVO2_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手手臂转到90°信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手手臂90°到位信号！\r\n");
 APP_LOG_INFO("向榨汁口运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,SENSOR_POS_IN_ROW_JUICE_PORT<<8|SENSOR_POS_IN_COLUMN_JUICE_PORT,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);//内部系统错误
 APP_LOG_ERROR("错误！向榨汁口运动超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！向榨汁口运动失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");   
 APP_LOG_INFO("机械手手臂转到角度180°！\r\n");
 osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_180_MSG,0);
 juice_wait_signals(SERVO2_REACH_POS_OK_SIGNAL,SERVO2_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手手臂转到180°信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }  
 APP_LOG_INFO("同步任务收到机械手手臂180°到位信号！\r\n");
 APP_LOG_INFO("向果汁杯卡槽运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,SENSOR_POS_IN_ROW_CUP_SLOT<<8|SENSOR_POS_IN_COLUMN_JUICE_PORT,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);//内部系统错误
 APP_LOG_ERROR("错误！向果汁杯卡槽运动超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！向果汁杯卡槽运动失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");
 APP_LOG_INFO("机械手爪子张开！\r\n");
 osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_OPEN_MSG,0);
 juice_wait_signals(SERVO1_REACH_POS_OK_SIGNAL,SERVO1_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手爪子张开信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手爪子张开到位信号！\r\n");
 APP_LOG_INFO("返回到榨汁口运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,SENSOR_POS_IN_ROW_JUICE_PORT<<8|SENSOR_POS_IN_COLUMN_JUICE_PORT,0);
 signals=juice_wait_signals(MANIPULATOR_REACH_POS_OK_SIGNAL|MANIPULATOR_REACH_POS_ERR_SIGNAL,MANIPULATOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & MANIPULATOR_REACH_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_MANIPULATOR_TIMEOUT);//内部系统错误
 APP_LOG_ERROR("错误！返回到榨汁口运动超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！返回到榨汁口运动失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手到位信号！\r\n");
 APP_LOG_INFO("机械手爪子抓紧！\r\n");
 osMessagePut(servo1_msg_queue_hdl,SERVO1_ANGLE_CLOSE_MSG,0);
 juice_wait_signals(SERVO1_REACH_POS_OK_SIGNAL,SERVO1_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手爪子抓紧信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手爪子抓紧到位信号！\r\n");
 APP_LOG_INFO("机械手手臂转到角度90°！\r\n");
 osMessagePut(servo2_msg_queue_hdl,SERVO2_ANGLE_90_MSG,0);
 juice_wait_signals(SERVO2_REACH_POS_OK_SIGNAL,SERVO2_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！系统未收到机械手手臂转到90°到位信号！\r\n");
 juice_transaction_fault(); 
 continue;
 }
 APP_LOG_INFO("同步任务收到机械手手臂90°到位信号！\r\n");
 /*
 if(BSP_is_cup_in_slot_pos()!=JUICE_TRUE)
 {
 juice_set_fault_code(FAULT_CODE_CUP_NOT_EXIST);//果汁杯没有放在杯槽里
 APP_LOG_ERROR("错误！果汁杯没有放在杯槽里！\r\n");
 juice_transaction_fault(); 
 continue; 
 }
 APP_LOG_INFO("同步任务收到果杯在榨汁槽信号！\r\n");
 */
 APP_LOG_INFO("即将榨汁，发送提示即将榨汁闪烁消息！\r\n");
 osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_INDICATE_MSG,0); 
 
 APP_LOG_INFO("机械手向默认停止位置运动！\r\n");
 osMessagePut(manipulator_msg_queue_hdl,DEFAULT_ROW_SENSOR_POS<<8|DEFAULT_COLUMN_SENSOR_POS,0);//不再处理机械手的状态,已经不能影响榨汁交易，但出错有错误码
 
 
 APP_LOG_INFO("压杯电机开始压杯！\r\n");
 osMessagePut(presser_msg_queue_hdl,PRESSER_PRESS_MSG,0);
 signals=juice_wait_signals(PRESSER_REACH_BOT_POS_OK_SIGNAL|PRESSER_REACH_BOT_POS_ERR_SIGNAL,PRESSER_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & PRESSER_REACH_BOT_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_PRESSER_PRESS_TIMEOUT);
 APP_LOG_ERROR("错误！压杯电机到达下限位置超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！压杯失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }



 /*
 if(BSP_is_cup_press_ok()!=JUICE_TRUE)
 {
 juice_set_fault_code(FAULT_CODE_CUP_NOT_EXIST);//果汁杯没有按压在杯槽里
 APP_LOG_ERROR("错误！果汁杯没有按压在杯槽里！\r\n");
 juice_transaction_fault(); 
 continue; 
 }
 */
 
 osDelay(2000);
 APP_LOG_INFO("榨汁开始，发送彩灯榨汁消息！\r\n");
 osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_JUICING_MSG,0);
 
 APP_LOG_INFO("同步任务收到压杯到达下限位点信号！\r\n");
 APP_LOG_INFO("榨汁电机启动工作！\r\n");  
 osMessagePut(juice_msg_queue_hdl,PRESSER_PRESS_MSG,0);
 signals=juice_wait_signals(JUICE_TIME_OK_SIGNAL,JUICING_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_INTERNAL_ERR);//内部系统错误
 APP_LOG_ERROR("错误！内部未收到榨汁时间到信号！\r\n");
 juice_transaction_fault();
 continue;
 }  
 APP_LOG_INFO("榨汁完成，发送彩灯完成消息！\r\n");
 osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_OK_MSG,0); 
 
 APP_LOG_INFO("同步任务收到榨汁时间到达信号！\r\n"); 
 
 osDelay(2000);
 APP_LOG_INFO("压杯电机开始释放杯子！\r\n");
 osMessagePut(presser_msg_queue_hdl,PRESSER_UNPRESS_MSG,0);
 signals=juice_wait_signals(PRESSER_REACH_TOP_POS_OK_SIGNAL|PRESSER_REACH_TOP_POS_ERR_SIGNAL,PRESSER_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & PRESSER_REACH_TOP_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_PRESSER_UNPRESS_TIMEOUT);
 APP_LOG_ERROR("错误！压杯电机到达上限位置超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！释放杯子失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 }   
 APP_LOG_INFO("同步任务收到压杯到达上限位点信号！\r\n");  
 
 
 APP_LOG_INFO("打开升降门！\r\n");
 osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_OPEN_MSG,0);
 signals=juice_wait_signals(OH_DOOR_REACH_TOP_POS_OK_SIGNAL|OH_DOOR_REACH_TOP_POS_ERR_SIGNAL,OH_DOOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & OH_DOOR_REACH_TOP_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_OH_DOOR_UP_TIMEOUT);
 APP_LOG_ERROR("错误！打开升降门超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！打开升降门失败！\r\n");
 }
 juice_transaction_fault(); 
 continue;
 } 
 APP_LOG_INFO("同步任务收到升降门到达上限位点信号！\r\n");
 
 juice_transaction_completed();//设置进度为交易成功
 
/*
 while(cup_timeout<CUP_TIMEOUT_VALUE)//检测果汁杯是否被取走，或者超时未取走
 {
  if(BSP_is_cup_in_slot_pos()==JUICE_FALSE)
  { 
  APP_LOG_INFO("检测到果杯被取走！等待5秒钟后确认！");
  osDelay(CUP_TAKE_AWAY_DELAY_VALUE);
  if(BSP_is_cup_in_slot_pos()==JUICE_FALSE)
  {
  APP_LOG_ERROR("确认果汁杯被取走！可以关门");
  break;
  }
  else
  {
  APP_LOG_ERROR("果杯又被放下，继续等待被取走！");
  }
  
  }             
 osDelay(CUP_DETECT_INTERVAL_VALUE);
 cup_timeout+=CUP_DETECT_INTERVAL_VALUE;
 }
 cup_timeout=0;
 */
 
 
 osDelay(5000);
 APP_LOG_INFO("关闭升降门！\r\n");
 osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_CLOSE_MSG,0);
 signals=juice_wait_signals(OH_DOOR_REACH_BOT_POS_OK_SIGNAL|OH_DOOR_REACH_BOT_POS_ERR_SIGNAL,OH_DOOR_TIMEOUT_VALUE);
 if(signals.status==osEventTimeout || (signals.status==osEventSignal && signals.value.signals & OH_DOOR_REACH_TOP_POS_ERR_SIGNAL ))
 {
 if(signals.status==osEventTimeout)
 {
 juice_set_fault_code(FAULT_CODE_OH_DOOR_DWN_TIMEOUT);
 APP_LOG_ERROR("错误！关闭升降门超时！\r\n");
 }
 else
 {
 APP_LOG_ERROR("错误！关闭升降门失败！\r\n");  
 }
 juice_fault_after_transaction_completed(); 
 continue;
 } 
 APP_LOG_INFO("同步任务收到升降门到达下限位点信号！\r\n");
 

 /*
 if(BSP_is_cup_in_slot_pos()==JUICE_TRUE)
 {
 juice_set_fault_code(FAULT_CODE_CUP_NOT_TAKE_AWAY);
 APP_LOG_ERROR("果汁杯超时间未取走！\r\n");
 continue;
 }
 APP_LOG_INFO("同步任务在关闭升降门后，确认里面无果杯信号，榨汁全部流程结束！\r\n");
 
*/ 
 juice_all_completed();
 }
}