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
//static uint8_t manipulator_get_sensor_row_pos(uint8_t juice_row_pos);
//static uint8_t manipulator_get_sensor_column_pos(uint8_t juice_column_pos);

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
 if(timeout>=MANIPULATOR_STEP_FREQUENCY_TIMEOUT_VALUE)//(MANIPULATOR_EXPIRED_FREQUENCY-MANIPULATOR_START_FREQUENCY)/MANIPULATOR_STEP_FREQUENCY)
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





