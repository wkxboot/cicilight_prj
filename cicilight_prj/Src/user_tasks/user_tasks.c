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
//ADC结果数组
uint16_t adc_sample[5];
uint16_t adc_average[5];
uint16_t adc_result[5];

//任务函数
static void sync_task(void const * argument);
static void adc_task(void const * argument);
static void rgb_led_task(void const * argument);
static void running_led_task(void const * argument);
static void oh_door_task(void const * argument);
static void servo1_task(void const * argument);
static void servo2_task(void const * argument);
static void manipulator_task(void const * argument);
static void presser_task(void const * argument);
static void compressor_task(void const * argument);
static void juice_task(void const * argument);
static void modbus_task(void const * argument);
static void temperature_task(void const * argument);

//内部函数
static void tasks_environment_init();
static uint8_t manipulator_get_sensor_row_pos(uint8_t juice_row_pos);
static uint8_t manipulator_get_sensor_column_pos(uint8_t juice_column_pos);

static uint8_t juice_is_oh_door_oc();//升降门过载
static uint8_t juice_is_24v_oc();//24V过载
static uint8_t juice_is_column_step_motor_stall();//步进电机反向电动势过载
static uint8_t juice_is_presser_oc();//压杯电机过载
static uint8_t juice_get_operation_param(uint8_t *ptr_row_pos,uint8_t *ptr_column_pos,uint16_t *ptr_opt);//获取运行参数目标位置
static uint8_t juice_set_progress(uint8_t stage);//设置榨汁进度值
static uint8_t juice_set_fault_code(uint8_t err_code);//设置榨汁错误码
static uint8_t juice_get_fault_code();//获取错误码


static void juice_all_completed();

static void juice_transaction_excuting();
static void juice_transaction_fault();
static void juice_transaction_completed();
static void juice_fault_after_transaction_completed();

static void MX_TIM2_ReInit_CH3(uint16_t pulse);
static void MX_TIM2_ReInit_CH4(uint16_t pulse);
static void MX_TIM8_ReInit(uint16_t f);
static void manipulator_pwm_frequency(uint8_t stop_signal,uint16_t interval);

//内存申请失败回调
void vApplicationMallocFailedHook(void)
{
APP_ERROR_HANDLER(APP_ERROR_NO_MEM); 
}

//创建用户任务
void app_create_user_tasks(void)
{
 APP_LOG_DEBUG("系统任务运行参数初始化...\r\n");
 tasks_environment_init();
 APP_LOG_DEBUG("创建任务...\r\n"); 
 osThreadDef(sync_task, sync_task, osPriorityNormal, 0, 128);
 sync_task_hdl = osThreadCreate(osThread(sync_task), NULL);
  
 osThreadDef(adc_task, adc_task, osPriorityNormal, 0, 128);
 adc_task_hdl = osThreadCreate(osThread(adc_task), NULL);

 osThreadDef(rgb_led_task, rgb_led_task, osPriorityNormal, 0, 128);
 rgb_led_task_hdl = osThreadCreate(osThread(rgb_led_task), NULL);
  
 osThreadDef(running_led_task, running_led_task, osPriorityNormal, 0, 128);
 running_led_task_hdl = osThreadCreate(osThread(running_led_task), NULL);

 osThreadDef(oh_door_task, oh_door_task, osPriorityNormal, 0, 256);
 oh_door_task_hdl = osThreadCreate(osThread(oh_door_task), NULL); 
  
 osThreadDef(servo1_task, servo1_task, osPriorityNormal, 0, 128);
 servo1_task_hdl = osThreadCreate(osThread(servo1_task), NULL); 
  
 osThreadDef(servo2_task, servo2_task, osPriorityNormal, 0, 128);
 servo2_task_hdl = osThreadCreate(osThread(servo2_task), NULL);
 
 osThreadDef(manipulator_task, manipulator_task, osPriorityNormal, 0, 256);
 manipulator_task_hdl = osThreadCreate(osThread(manipulator_task), NULL);
 
 osThreadDef(presser_task, presser_task, osPriorityNormal, 0, 128);
 presser_task_hdl = osThreadCreate(osThread(presser_task), NULL);
 
 osThreadDef(compressor_task, compressor_task, osPriorityNormal, 0, 128);
 compressor_task_hdl = osThreadCreate(osThread(compressor_task), NULL);
 
 osThreadDef(juice_task, juice_task, osPriorityNormal, 0, 128);
 juice_task_hdl = osThreadCreate(osThread(juice_task), NULL);
  
 osThreadDef(temperature_task, temperature_task, osPriorityNormal, 0, 128);
 temperature_task_hdl = osThreadCreate(osThread(temperature_task), NULL);
 
 osThreadDef(modbus_task, modbus_task, osPriorityNormal, 0, 256);
 modbus_task_hdl = osThreadCreate(osThread(modbus_task), NULL);
 
#if defined(IO_CHECK_TASK_ENABLE) &&  IO_CHECK_TASK_ENABLE > 0  
 osThreadDef(io_check_task, io_check_task, osPriorityNormal, 0, 128);
 io_check_task_hdl = osThreadCreate(osThread(io_check_task), NULL);
#endif
 }

//MODBUS通信定时器超时回调
void SLAVE_MB_timer_expired_callback(void const * argument);

static void tasks_environment_init()
{
  mb_reg_init();
  //消息队列 
  osMessageQDef(sync_msg_queue, 16, uint16_t);
  sync_msg_queue_hdl = osMessageCreate(osMessageQ(sync_msg_queue), NULL); 
  
  osMessageQDef(adc_msg_queue, 16, uint16_t);
  adc_msg_queue_hdl = osMessageCreate(osMessageQ(adc_msg_queue), NULL); 
  
  osMessageQDef(rgb_led_msg_queue, 16, uint16_t);
  rgb_led_msg_queue_hdl = osMessageCreate(osMessageQ(rgb_led_msg_queue), NULL); 
  
  osMessageQDef(running_led_msg_queue, 16, uint16_t);
  running_led_msg_queue_hdl = osMessageCreate(osMessageQ(running_led_msg_queue), NULL); 
  
  osMessageQDef(oh_door_msg_queue, 16, uint16_t);
  oh_door_msg_queue_hdl = osMessageCreate(osMessageQ(oh_door_msg_queue), NULL); 
  
  osMessageQDef(manipulator_msg_queue, 16, uint16_t);
  manipulator_msg_queue_hdl = osMessageCreate(osMessageQ(manipulator_msg_queue), NULL); 
  
  osMessageQDef(servo1_msg_queue, 16, uint16_t);
  servo1_msg_queue_hdl = osMessageCreate(osMessageQ(servo1_msg_queue), NULL); 
  
  osMessageQDef(servo2_msg_queue, 16, uint16_t);
  servo2_msg_queue_hdl = osMessageCreate(osMessageQ(servo2_msg_queue), NULL); 
  
  osMessageQDef(presser_msg_queue, 16, uint16_t);
  presser_msg_queue_hdl = osMessageCreate(osMessageQ(presser_msg_queue), NULL); 
  
  osMessageQDef(compressor_msg_queue, 16, uint16_t);
  compressor_msg_queue_hdl = osMessageCreate(osMessageQ(compressor_msg_queue), NULL); 
  
  osMessageQDef(juice_msg_queue, 16, uint16_t);
  juice_msg_queue_hdl = osMessageCreate(osMessageQ(juice_msg_queue), NULL);
  /* Create the timer(s) */
  /* definition and creation of SLAVE_MB_timer */
  osTimerDef(SLAVE_MB_timer, SLAVE_MB_timer_expired_callback);
  SLAVE_MB_timer_hdl = osTimerCreate(osTimer(SLAVE_MB_timer), osTimerOnce, NULL);
  
  //信号量
  osMutexDef(err_code_mutex);
  err_code_mutex_id=osMutexCreate(osMutex(err_code_mutex));
  
  //硬件参数初始化
  BSP_row_step_motor_init();
  BSP_column_step_motor_init();
}

static void modbus_task(void const * argument)
{
   APP_LOG_INFO("++++++MODBUS 任务开始!\r\n");
  /* Select either ASCII or RTU Mode. */
  ( void )eMBInit(MB_RTU, 0x0B, 0, 9600, MB_PAR_NONE );
  /* Enable the Modbus Protocol Stack. */
  ( void )eMBEnable();
  for( ;; )
  {
  /* Call the main polling loop of the Modbus protocol stack. */
  ( void )eMBPoll();
  } 
    
  
}


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

static void running_led_task(void const * argument)
{
  APP_LOG_INFO("++++++系统运行灯任务开始！\r\n");
  while(1)
  {
  BSP_running_led_turn_on();
  osDelay(500);
  BSP_running_led_turn_off();
  osDelay(500);
  }
}
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

//机械爪子舵机1任务
static void servo1_task(void const * argument)
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

//机械手臂舵机2任务
static void servo2_task(void const * argument)
{
 osEvent msg;
 APP_LOG_INFO("++++++机械手臂舵机2任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(servo2_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage)
 {
  APP_LOG_INFO("手臂舵机2收到错误消息！\r\n");
  continue ;
 }
 APP_LOG_DEBUG("手臂舵机2收到消息角度：%d°！\r\n",msg.value.v);
 HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
 MX_TIM2_ReInit_CH3((msg.value.v*11+500)/10);
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
 osDelay(SERVO2_ENABLE_TIME_VALUE);
 osSignalSet(sync_task_hdl,SERVO2_REACH_POS_OK_SIGNAL);
 }
}

//机械手滑台任务
static void manipulator_task(void const * argument)
{
 manipulator_state_t manipulator;

 osEvent msg;
 uint8_t row_sensor_pos,column_sensor_pos;
 uint8_t row_sensor_state,column_sensor_state;
 //参数初始话
 manipulator.row.active=JUICE_TRUE;
 manipulator.row.cur_pos=SENSOR_POS_IN_ROW_NULL;
 manipulator.row.tar_pos=SENSOR_POS_IN_ROW_NULL;
 manipulator.row.dir=NULL_DIR;
 manipulator.row.run_time=0;
 manipulator.row.sensor_hold_time=0;
 
 manipulator.column.active=JUICE_TRUE;
 manipulator.column.cur_pos=SENSOR_POS_IN_COLUMN_NULL;
 manipulator.column.tar_pos=SENSOR_POS_IN_COLUMN_NULL;
 manipulator.column.dir=NULL_DIR;
 manipulator.column.run_time=0;
 manipulator.column.sensor_hold_time=0;
 
 manipulator.msg_send=JUICE_TRUE;
 
 APP_LOG_INFO("++++++机械手滑台任务开始！\r\n");
 manipulator.row.sensor_state=BSP_get_row_pos_sensor_state();
 manipulator.column.sensor_state=BSP_get_column_pos_sensor_state();
 while(1)
 {
 msg=osMessageGet(manipulator_msg_queue_hdl,0);
 if(msg.status==osEventMessage )
 {
 APP_LOG_INFO("机械手滑台任务收到消息！\r\n");
 row_sensor_pos=msg.value.v>>8;
 column_sensor_pos=msg.value.v&0xff;
 
 manipulator.row.tar_pos= row_sensor_pos; 
 manipulator.column.tar_pos=column_sensor_pos;
 manipulator.row.active= JUICE_TRUE; 
 manipulator.column.active=JUICE_TRUE;
 manipulator.msg_send=JUICE_FALSE;
 manipulator.row.run_time=0;
 manipulator.column.run_time=0;
 manipulator.row.sensor_hold_time=0;
 manipulator.column.sensor_hold_time=0; 
 
 APP_LOG_INFO("目标位置sensor pos x：%d  y：%d！\r\n",row_sensor_pos,column_sensor_pos);
 if(manipulator.row.tar_pos==SENSOR_POS_IN_ROW_RST && manipulator.column.tar_pos==SENSOR_POS_IN_COLUMN_RST)
 {
 manipulator.row.cur_pos=SENSOR_POS_IN_ROW_NULL;
 manipulator.column.cur_pos=SENSOR_POS_IN_COLUMN_NULL;
 }
 }
 
 //传感器状态
 row_sensor_state=BSP_get_row_pos_sensor_state();
 //行步进电机确定当前的位置
 if(BSP_is_row_step_motor_in_rst_pos()==JUICE_TRUE && manipulator.row.cur_pos!=SENSOR_POS_IN_ROW_RST && manipulator.row.dir!=NULL_DIR)
 {
  manipulator.row.cur_pos=SENSOR_POS_IN_ROW_RST;
  manipulator.row.sensor_state=row_sensor_state;
  APP_LOG_DEBUG("行步进电机到达复位点！\r\n");
 }
 
 if(row_sensor_state!=manipulator.row.sensor_state)//和原来的状态不一样
 {
  manipulator.row.sensor_hold_time+=MANIPULATOR_INTERVAL_VALUE;
  if(manipulator.row.sensor_hold_time>=MANIPULATOR_ROW_SENSOR_TIMEOUT_VALUE)
  {
   manipulator.row.sensor_hold_time=0;
   manipulator.row.sensor_state=row_sensor_state;
   
   if(manipulator.row.cur_pos!=SENSOR_POS_IN_ROW_NULL)//如果已经确定了当前的位置，才可以继续确定当前的位置。只有复位才能第一次确定当前的位置！
   {
   if(manipulator.row.dir==POSITIVE_DIR)
   {
   manipulator.row.cur_pos++;
   APP_LOG_WARNING("行步进电机位置 +1 当前位置：%d！\r\n",manipulator.row.cur_pos);
   }
   else if(manipulator.row.dir==NEGATIVE_DIR)
   {
   manipulator.row.cur_pos--;
   APP_LOG_WARNING("行步进电机位置 -1 当前位置：%d！\r\n",manipulator.row.cur_pos);  
   }
   else
   {
   manipulator.row.cur_pos=SENSOR_POS_IN_ROW_NULL;
   manipulator.row.tar_pos=SENSOR_POS_IN_ROW_NULL;
   APP_LOG_WARNING("行步进电机位置被外部强制改变！位置置空！当前位置：%d！\r\n",manipulator.row.cur_pos);  
   }
  }
  else
  {
    APP_LOG_WARNING("行步进电机位置为空，忽略位置变动！\r\n"); 
  }  
  }
 }
 else if(manipulator.row.sensor_hold_time > 0)
 {
 manipulator.row.sensor_hold_time=0;
 APP_LOG_WARNING("行步进电机传感器抖动，忽略位置变动！\r\n"); 
 }
 
 //行步进电机确定当前运行方向
 if(manipulator.row.tar_pos < manipulator.row.cur_pos && manipulator.row.dir!=NEGATIVE_DIR)
 {
  manipulator.row.dir=NEGATIVE_DIR;
  BSP_row_step_motor_pwr_on_negative();//反向运动
  APP_LOG_INFO("行目标位置 < 当前位置，设置行方向 = NEGATIVE_DIR！行步进电机反转！\r\n");
 }
 else if(manipulator.row.tar_pos > manipulator.row.cur_pos && manipulator.row.dir!=POSITIVE_DIR)
 {
  manipulator.row.dir=POSITIVE_DIR;
  BSP_row_step_motor_pwr_on_positive();//正向运动
  APP_LOG_INFO("行目标位置 > 当前位置，设置行方向 = POSITIVE_DIR！行步进电机正转！\r\n");
 }
 else if(manipulator.row.tar_pos == manipulator.row.cur_pos &&  manipulator.row.dir!=NULL_DIR)
 {
  manipulator.row.dir=NULL_DIR;
  BSP_row_step_motor_pwr_dwn();//停机
  APP_LOG_INFO("行目标位置 == 当前位置，设置行方向 = DIR_NULL！行步进电机停机！\r\n"); 
 }
 
 //列传感器状态
  column_sensor_state=BSP_get_column_pos_sensor_state();
 //列步进电机确定当前的位置
 if(BSP_is_column_step_motor_in_rst_pos()==JUICE_TRUE && manipulator.column.cur_pos!=SENSOR_POS_IN_COLUMN_RST && manipulator.column.dir!=NULL_DIR)
 {
  manipulator.column.cur_pos=SENSOR_POS_IN_COLUMN_RST;
  manipulator.column.sensor_state=column_sensor_state;
  APP_LOG_INFO("列步进电机到达复位点！\r\n");
 }
 
 if(column_sensor_state!=manipulator.column.sensor_state)//和原来的状态不一样
 {
   manipulator.column.sensor_hold_time+=MANIPULATOR_INTERVAL_VALUE;
   if(manipulator.column.sensor_hold_time>=MANIPULATOR_COLUMN_SENSOR_TIMEOUT_VALUE)
   {
    manipulator.column.sensor_hold_time=0;
    manipulator.column.sensor_state=column_sensor_state;
   if(manipulator.column.cur_pos!=SENSOR_POS_IN_COLUMN_NULL)//如果已经确定了当前的位置，才可以继续确定当前的位置。只有复位才能第一次确定当前的位置！
   {
   if(manipulator.column.dir==POSITIVE_DIR)
   {
   manipulator.column.cur_pos++;
   APP_LOG_WARNING("列步进电机位置 +1 当前位置：%d！\r\n",manipulator.column.cur_pos);
   }
   else if(manipulator.column.dir==NEGATIVE_DIR)
   {
   manipulator.column.cur_pos--;
   APP_LOG_WARNING("列步进电机位置 -1 当前位置：%d！\r\n",manipulator.column.cur_pos);  
   }
   else
   {
   manipulator.row.cur_pos=SENSOR_POS_IN_COLUMN_NULL;
   manipulator.row.tar_pos=SENSOR_POS_IN_COLUMN_NULL;
   APP_LOG_WARNING("列步进电机位置被外部强制改变！位置置空！当前位置：%d！\r\n",manipulator.column.cur_pos);  
   }
   }
   else
   {
   APP_LOG_WARNING("行步进电机位置为空，忽略位置变动！\r\n"); 
   }
   }
 }
 else if(manipulator.column.sensor_hold_time>0)
 {
  manipulator.column.sensor_hold_time=0;
  APP_LOG_WARNING("列步进电机传感器抖动，忽略位置变动！\r\n");  
 }
 
 if(manipulator.column.tar_pos < manipulator.column.cur_pos && manipulator.column.dir!=NEGATIVE_DIR)
 {
  manipulator.column.dir=NEGATIVE_DIR;
  manipulator.column.run_time=0;
  BSP_column_step_motor_pwr_on_negative();//反向运动
  APP_LOG_INFO("列目标位置 < 当前位置，设置列方向 = NEGATIVE_DIR！列步进电机反转！\r\n");
 }
 else if(manipulator.column.tar_pos > manipulator.column.cur_pos && manipulator.column.dir!=POSITIVE_DIR)
 {
  manipulator.column.dir=POSITIVE_DIR;
  manipulator.column.run_time=0;
  BSP_column_step_motor_pwr_on_positive();//正向运动
  APP_LOG_INFO("列目标位置 > 当前位置，设置列方向 = POSITIVE_DIR！列步进电机正转！\r\n");
 }
 else if(manipulator.column.tar_pos == manipulator.column.cur_pos && manipulator.column.dir!=NULL_DIR)
 {
  manipulator.column.dir=NULL_DIR;
  manipulator.column.run_time=0;
  BSP_column_step_motor_pwr_dwn();//停机
  APP_LOG_INFO("列目标位置 == 当前位置，设置列方向 = DIR_NULL！列步进电机停机！\r\n"); 
 }

 //运行时间
 if(manipulator.row.dir!=NULL_DIR)
 manipulator.row.run_time+=MANIPULATOR_INTERVAL_VALUE;
 if(manipulator.column.dir!=NULL_DIR) 
 manipulator.column.run_time+=MANIPULATOR_INTERVAL_VALUE;
 
 //全部到位发送到位消息
 if( manipulator.msg_send==JUICE_FALSE && manipulator.row.tar_pos == manipulator.row.cur_pos && manipulator.column.tar_pos == manipulator.column.cur_pos )
 {
  osSignalSet(sync_task_hdl,MANIPULATOR_REACH_POS_OK_SIGNAL);
  manipulator.msg_send=JUICE_TRUE;
  APP_LOG_INFO("机械手全部到达目标位置，发送到位信号！\r\n"); 
 }
 
 //运动时检测错误  
 if(manipulator.column.run_time >= MANIPULATOR_COLUMN_MOTOR_STALL_TIMEOUT &&\
   (juice_is_column_step_motor_stall()==JUICE_TRUE || BSP_is_column_step_motor_fault()==JUICE_TRUE))
 {
 manipulator.column.active=JUICE_FALSE;
 juice_set_fault_code(FAULT_CODE_COLUMN_STEP_MOTOR_FAULT);
 APP_LOG_ERROR("列步进电机故障！\r\n"); 
 }
 if(juice_is_24v_oc()==JUICE_TRUE && (manipulator.column.active==JUICE_TRUE || manipulator.row.active==JUICE_TRUE))
 {
 manipulator.column.active=JUICE_FALSE;
 manipulator.row.active=JUICE_FALSE;
 juice_set_fault_code(FAULT_CODE_24V_OC);
 APP_LOG_ERROR("24V电流过载！\r\n"); 
 }

 if(manipulator.msg_send==JUICE_FALSE && (manipulator.row.active==JUICE_FALSE||manipulator.column.active==JUICE_FALSE))
 {
  APP_LOG_ERROR("行步进电机和列步进电机停机！\r\n"); 
  APP_LOG_ERROR("机械手错误，发送未到达目标位信号！\r\n"); 
  manipulator.row.dir=NULL_DIR;
  manipulator.column.dir=NULL_DIR;
  manipulator.row.active=JUICE_FALSE;
  manipulator.column.active=JUICE_FALSE;
  manipulator.row.run_time=0;
  manipulator.column.run_time=0;
  manipulator.row.tar_pos=manipulator.row.cur_pos;
  manipulator.column.tar_pos=manipulator.column.cur_pos;

  BSP_row_step_motor_pwr_dwn();//停机
  BSP_column_step_motor_pwr_dwn();//停机
  osSignalSet(sync_task_hdl,MANIPULATOR_REACH_POS_ERR_SIGNAL);
  manipulator.msg_send=JUICE_TRUE;
 }
  //机械手速度步进
 manipulator_pwm_frequency(manipulator.msg_send,MANIPULATOR_INTERVAL_VALUE);
 osDelay(MANIPULATOR_INTERVAL_VALUE);
 }
 }


//压杯任务
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
  

//压缩机任务
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

//榨汁任务
static void juice_task(void const * argument)
{
 static osEvent msg;
 static uint32_t timeout=0;
 APP_LOG_INFO("++++++榨汁任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(juice_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage)
 {
 continue ;
 }
 APP_LOG_INFO("榨汁任务收到消息！\r\n");
 if(msg.value.v == JUICE_START_MSG)
 {
  APP_LOG_INFO("榨汁开始！打开榨汁电机！\r\n");
  BSP_juicing_motor_pwr_on(); 
  osDelay(5);
  BSP_juicing_motor_pwr_dwn(); 
  osDelay(1000);
  BSP_juicing_motor_pwr_on(); 
  while(timeout<JUICING_TIMEOUT_VALUE-100)//留100ms保证消息发送时间
  {
  msg= osMessageGet(juice_msg_queue_hdl,0); 
  if(msg.status==osEventMessage && msg.value.v == JUICE_STOP_MSG)
  break;
  osDelay(JUICE_INTERVAL_VALUE);
  timeout+=JUICE_INTERVAL_VALUE;
  }
  timeout=0;
  BSP_juicing_motor_pwr_dwn();  
  osSignalSet(sync_task_hdl,JUICE_TIME_OK_SIGNAL);
  APP_LOG_INFO("榨汁结束！关闭榨汁电机！\r\n");
 }  
 }
}
//温度计任务
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
 
static void adc_process_sample_average()
{
 adc_result[0]=adc_average[ADC_PRESSER_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//压杯电机
 adc_result[1]=adc_average[ADC_OH_DOOR_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//升降门
 adc_result[2]=adc_average[ADC_24V_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//24V
 adc_result[3]=BSP_get_temperature(adc_average[ADC_T_IDX]);//温度
 adc_result[4]=adc_average[ADC_BEMF_IDX]*3300*ADC_BEMF_DIV/4096;//BEMF
#if  1
 APP_LOG_INFO("压杯电流：%d mA.\r\n",adc_result[0]);
 APP_LOG_INFO("升降门电流：%d mA.\r\n",adc_result[1]);
 APP_LOG_INFO("24V电流：%d mA.\r\n",adc_result[2]);
 APP_LOG_INFO("温度值：%d ℃.\r\n",(int16_t)adc_result[3]);
 APP_LOG_INFO("BEMF：%d mV.\r\n",adc_result[4]);
#endif
}
 
static uint8_t juice_is_presser_oc()
{
 if(adc_result[0] > ADC_PRESSER_OC_THRESHOLD_mAMPERE)
 { 
 APP_LOG_ERROR("压杯电机过载：%d mA！\r\n",adc_result[0]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}
static uint8_t juice_is_oh_door_oc()
{
 if(adc_result[1] > ADC_OH_DOOR_OC_THRESHOLD_mAMPERE)
 {
 APP_LOG_ERROR("升降门过载:%d mA！\r\n",adc_result[1]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}
static uint8_t juice_is_24v_oc()
{
 if(adc_result[2] > ADC_24V_OC_THRESHOLD_mAMPERE)
 {
 APP_LOG_ERROR("24V过载:%d mA！\r\n",adc_result[2]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}

static uint8_t juice_is_column_step_motor_stall()//bemf 反向电动势
{
 if(adc_result[4] < ADC_BEMF_THRESHOLD_mVOLTAGE)
 {
 APP_LOG_ERROR("60步进电机堵转：%d mV！\r\n",adc_result[4]);
 return JUICE_TRUE;
 }
 return JUICE_FALSE;
}

static void adc_task(void const * argument)
{
 uint32_t adc_cusum[ADC_CNT]={0};
 uint32_t adc_timeout=0;
 uint16_t adc_times=0;
 APP_LOG_INFO("++++++ADC任务开始！\r\n");

 while(1)
 {
 while(adc_timeout<ADC_TIMEOUT_VALUE)
 {
 //adc
 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_sample,ADC_CNT);
 osDelay(ADC_TASK_RUN_INTERVAL_VALUE);
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_cusum[i]+=adc_sample[i];  
 }
 adc_timeout+=ADC_TASK_RUN_INTERVAL_VALUE;
 adc_times++;
 }
 APP_LOG_INFO("ADC取样时间：%d ms，取样次数：%d.\r\n",adc_timeout,adc_times);
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_average[i]=adc_cusum[i]/adc_times; 
 //APP_LOG_DEBUG("累加值[%d]：%d.\r\n",i,adc_cusum[i]);
 //APP_LOG_INFO("平均值[%d]：%d.\r\n",i,adc_average[i]);
 } 
 adc_process_sample_average();
 
 adc_timeout=0;
 adc_times=0; 
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_cusum[i]=0;  
 }
 }
}

//根据行坐标点得到行传感器位置
static uint8_t manipulator_get_sensor_row_pos(uint8_t juice_row_pos)
{
  uint8_t sensor_pos = (juice_row_pos-1)*4+6;

  return sensor_pos;
}

//根据列坐标点得到列传感器位置
static uint8_t manipulator_get_sensor_column_pos(uint8_t juice_column_pos)
{
 uint8_t sensor_pos=(juice_column_pos-1)*2+4;
 
 return sensor_pos;
}

/**
* @brief 设置错误码 
* @param -- 
* @return -- 
* @details --
* @see --
*/

static uint8_t juice_set_fault_code(uint8_t err_code)
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

static uint8_t juice_get_fault_code()
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
 *ptr_row_pos=manipulator_get_sensor_row_pos(row_pos);
 *ptr_column_pos=manipulator_get_sensor_column_pos(column_pos);
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


static void sync_task(void const * argument)
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
 /*
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
*/
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
 
 /*
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
 
*/
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

/* USER CODE BEGIN Application */
static void MX_TIM2_ReInit_CH3(uint16_t pulse)
{
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USER CODE BEGIN Application */
static void MX_TIM2_ReInit_CH4(uint16_t pulse)
{
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void manipulator_pwm_frequency(uint8_t stop_signal,uint16_t interval)
{
 static uint16_t timeout=0;
 static uint16_t f=MANIPULATOR_START_FREQUENCY;
 
 if(stop_signal==JUICE_TRUE )
 {
  if(f!=MANIPULATOR_START_FREQUENCY)
  {   
  f=MANIPULATOR_START_FREQUENCY;
  timeout=0;
  MX_TIM8_ReInit(f);
  APP_LOG_INFO("机械手频率复位！当前频率：%d kHz！\r\n",f);
  }
  return ;
 }
 if(f==MANIPULATOR_EXPIRED_FREQUENCY)
 return;
 
 timeout+=interval;
 if(timeout>=MANIPULATOR_STEP_FREQUENCY_TIMEOUT)//(MANIPULATOR_EXPIRED_FREQUENCY-MANIPULATOR_START_FREQUENCY)/MANIPULATOR_STEP_FREQUENCY)
 {
  timeout=0;
  if(f+MANIPULATOR_STEP_FREQUENCY>=MANIPULATOR_EXPIRED_FREQUENCY)
  {
  f=MANIPULATOR_EXPIRED_FREQUENCY;
  APP_LOG_INFO("机械手完成步进启动！当前频率：%d kHz！\r\n",f);
  }
  else
  {
  f+=MANIPULATOR_STEP_FREQUENCY;
  APP_LOG_INFO("机械手正在步进启动中！当前频率：%d kHz！\r\n",f);
  }
  
  MX_TIM8_ReInit(f);
 }
}


static void MX_TIM8_ReInit(uint16_t f)
{
  uint16_t period;
  period=1000/f;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = period;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}





