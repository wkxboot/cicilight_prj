#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"    
#include "gpio.h"
#include "tim.h"
#include "mb_msg.h"
#include "mb_reg.h"
#include "JJDK_ZK_GZ1.h"
#include "user_tasks.h"
#include "rgb_led.h"
#include "app_log.h"
    
#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[freertos]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#endif


 //任务的handle;
osThreadId juice_sync_task_hdl;   
osThreadId juice_asyn_task_hdl;  
osThreadId sensor_info_task_hdl;
osThreadId rgb_led_task_hdl;
//事件组
EventGroupHandle_t sensor_event;
EventGroupHandle_t sync_event;
//消息队列id
osMessageQId juice_asyn_msg_queue_hdl;

//定时器id
osTimerId SLAVE_MB_timer_hdl;
osTimerId row_step_motor_timer_hdl;
osTimerId column_step_motor_timer_hdl;

static void juice_sync_task(void const * argument);
static void juice_asyn_task(void const * argument);
static void sensor_info_task(void const * argument);
static void rgb_led_task(void const * argument);
//volatile sensor_row_rst,sensor_column_rst,sensor_row_tar,sensor_row_rst

void task_environment_init();
void servo_arm_pos(uint16_t angles);
void servo_grabber_open();
void servo_grabber_close();


//创建用户任务
 void app_create_user_tasks(void)
 {
  task_environment_init();
  
  osThreadDef(juice_sync_task, juice_sync_task, osPriorityNormal, 0, 128);
  juice_sync_task_hdl = osThreadCreate(osThread(juice_sync_task), NULL);
  
  osThreadDef(juice_asyn_task, juice_asyn_task, osPriorityNormal, 0, 128);
  juice_asyn_task_hdl = osThreadCreate(osThread(juice_asyn_task), NULL);
  
  osThreadDef(sensor_info_task, sensor_info_task, osPriorityNormal, 0, 128);
  sensor_info_task_hdl = osThreadCreate(osThread(sensor_info_task), NULL);

  osThreadDef(rgb_led_task, rgb_led_task, osPriorityNormal, 0, 128);
  rgb_led_task_hdl = osThreadCreate(osThread(rgb_led_task), NULL);

 }





void SLAVE_MB_timer_expired_callback(void const * argument);

void row_step_motor_timer_expired_callback(void const * argument)
{
}
void column_step_motor_timer_expired_callback(void const * argument)
{
}
void task_environment_init()
{
   /* definition and creation of ew_queue_hdl */
  osMessageQDef(juice_asyn_msg_queue, 16, uint16_t);
  juice_asyn_msg_queue_hdl = osMessageCreate(osMessageQ(juice_asyn_msg_queue), NULL); 
  
  
  /* Create the timer(s) */
  /* definition and creation of SLAVE_MB_timer */
  osTimerDef(SLAVE_MB_timer, SLAVE_MB_timer_expired_callback);
  SLAVE_MB_timer_hdl = osTimerCreate(osTimer(SLAVE_MB_timer), osTimerOnce, NULL);
  /* definition and creation of row_step_motor_timer */
  osTimerDef(row_step_motor_timer, row_step_motor_timer_expired_callback);
  row_step_motor_timer_hdl = osTimerCreate(osTimer(row_step_motor_timer), osTimerOnce, NULL);
  /* definition and creation of column_step_motor_timer */
  osTimerDef(column_step_motor_timer, column_step_motor_timer_expired_callback);
  column_step_motor_timer_hdl = osTimerCreate(osTimer(column_step_motor_timer), osTimerOnce, NULL);
  
  
 //创建事件组
   sensor_event= xEventGroupCreate();
   sync_event= xEventGroupCreate();
   
   //硬件参数初始化
   BSP_row_step_motor_init();
   BSP_column_step_motor_init();
}
static void rgb_led_task(void const * argument)
{
  APP_LOG_DEBUG("RGB_LED task sart!\r\n");
  while(1)
  {
  //rainbow(10); 
   single_color(0xff<<16|0xff<<8|0xff,255);
   
   //微动开关和光电开关
   if( BSP_is_ps_1_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 1 IS OK!\r\n");
   }
   if( BSP_is_ps_2_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 2 IS OK!\r\n");
   }
    if( BSP_is_ps_3_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 3 IS OK!\r\n");
   }
    if( BSP_is_ps_4_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 4 IS OK!\r\n");
   }
    if( BSP_is_ps_5_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 5 IS OK!\r\n");
   }
    if( BSP_is_ps_6_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("PS 6 IS OK!\r\n");
   }
   
     if( BSP_is_ms_3_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 3 IS OK!\r\n");
   }
   if( BSP_is_ms_4_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 4 IS OK!\r\n");
   }
    if( BSP_is_ms_5_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 5 IS OK!\r\n");
   }
    if( BSP_is_ms_6_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 6 IS OK!\r\n");
   }
    if( BSP_is_ms_7_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 7 IS OK!\r\n");
   }
    if( BSP_is_ms_8_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 8 IS OK!\r\n");
   }
       if( BSP_is_ms_9_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 9 IS OK!\r\n");
   }
    if( BSP_is_ms_10_on_tar_pos()==BSP_TRUE)
   {
    APP_LOG_DEBUG("MS 10 IS OK!\r\n");
   }
   
  uint8_t cmd;
  cmd=SEGGER_RTT_GetKey();
  if(cmd!=0xFF)
  {
  switch(cmd)
  {
  case 'a':
  BSP_press_motor_pwr_dwn();
  APP_LOG_DEBUG("BSP_press_motor_pwr_dwn! \r\n");
  break;
  case 'b':
  BSP_press_motor_pwr_on_positive();
  APP_LOG_DEBUG("BSP_press_motor_pwr_on_positive! \r\n");
  break;
  case 'c':
   BSP_press_motor_pwr_on_negative();
   APP_LOG_DEBUG("BSP_press_motor_pwr_on_negative! \r\n");
  break;
  case 'd':
  BSP_oh_door_motor_pwr_dwn();
  APP_LOG_DEBUG("BSP_oh_door_motor_pwr_dwn! \r\n");
  break;
  case 'e':
   BSP_oh_door_motor_pwr_on_positive();
  APP_LOG_DEBUG("BSP_oh_door_motor_pwr_on_positive! \r\n");
  break;
  case 'f':
   BSP_oh_door_motor_pwr_on_negative();
   APP_LOG_DEBUG("BSP_oh_door_motor_pwr_on_negative! \r\n");
  break;
  case 'g':
   BSP_environment_lamp_pwr_on();
   APP_LOG_DEBUG("BSP_environment_lamp_pwr_on! \r\n");
  break;
  case 'h':
   BSP_environment_lamp_pwr_dwn();
   APP_LOG_DEBUG("BSP_environment_lamp_pwr_dwn! \r\n");
  break;
  case 'i':
  BSP_compressor_pwr_on();
   APP_LOG_DEBUG("BSP_compressor_pwr_on! \r\n");
  break;
  case 'j':
  BSP_compressor_pwr_dwn();
   APP_LOG_DEBUG("BSP_compressor_pwr_dwn! \r\n");
   break;
  case 'k':
  BSP_juicing_motor_pwr_on();
  APP_LOG_DEBUG("BSP_juicing_motor_pwr_on! \r\n");
  break;
  case 'l':
  BSP_juicing_motor_pwr_dwn();
  APP_LOG_DEBUG("BSP_juicing_motor_pwr_dwn! \r\n");
   break;
  default:
   APP_LOG_DEBUG("illegal CMD!\r\n");

  }
  }
   osDelay(100);
  }
}

static void sensor_info_task(void const * argument)
{
  EventBits_t bits;
 while(1)
 {
 
/*步进电机column 传感器 */
if(BSP_is_column_step_motor_on_rst_pos()== BSP_TRUE)
{
  bits=xEventGroupWaitBits(sensor_event,SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_RST_EVT,pdTRUE, pdFALSE, 0);
  if(bits & SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_RST_EVT)
 {
  xEventGroupSetBits(sync_event,SYNC_COLUMN_STEP_MOTOR_ON_RST_POS_EVT);
 }
}

/*步进电机column 传感器 目标点*/
if(BSP_is_column_step_motor_on_tar_pos()== BSP_TRUE)
{
 bits=xEventGroupWaitBits(sensor_event,SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_TAR_EVT,pdTRUE, pdFALSE, 0);
 if(bits & SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_TAR_EVT)
 {
  xEventGroupSetBits(sync_event,SYNC_COLUMN_STEP_MOTOR_ON_TAR_POS_EVT);
 }
}
/*步进电机row 传感器 复位点*/
if(BSP_is_row_step_motor_on_rst_pos()== BSP_TRUE)
{
 bits=xEventGroupWaitBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_RST_EVT,pdTRUE, pdFALSE, 0);
  if(bits & SENSOR_CHECK_ROW_STEP_MOTOR_ON_RST_EVT)
  {
   xEventGroupSetBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_RST_POS_EVT);
 }
}

/*步进电机row 传感器 目标点*/
if(BSP_is_row_step_motor_on_tar_pos()== BSP_TRUE)
{
  bits=xEventGroupWaitBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT,pdTRUE, pdFALSE, 0);
  if(bits & SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT)
  {
   xEventGroupSetBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_TAR_POS_EVT);
 }
}
osDelay(50);
}
}


void juice_asyn_set_opt_type()
{
  uint16_t reg_value;  
  reg_value= get_reg_value(OPERATION_TYPE_REGHOLDING_ADDR, 1,REGHOLDING_MODE); 
  BSP_set_opt_type(reg_value);
  
}

void juice_asyn_start_sync_task()
{
 uint16_t reg_value; 
 uint8_t row,column;
 reg_value= get_reg_value(JUICE_POS_REGHOLDING_ADDR, 1,REGHOLDING_MODE);
 row=reg_value>>8;
 column=reg_value; 
 if(row<=REG_VALUE_JUICE_ROW_MAX_POS  && column <=REG_VALUE_JUICE_COLUMN_MAX_POS)//
 {
 osSignalSet(juice_sync_task_hdl,SYNC_START_EVT);//发送信号启动同步任务
 }
 else
 {
 APP_LOG_DEBUG("pos value is iilegal! ignored!\r\n");
 }
}

void juice_asyn_set_fault_code()
{
 APP_LOG_DEBUG("excute set fault code!\r\n");
  /*
 uint16_t reg_value; 
 reg_value= get_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR, 1,REGHOLDING_MODE);
 BSP_set_fault_code(reg_value); 
*/  
}


static void juice_asyn_task(void const * argument)
{
 osEvent event;
 APP_LOG_DEBUG("juice_asyn_task START!\r\n");
 while(1)
 {
 event= osMessageGet(juice_asyn_msg_queue_hdl,osWaitForever);
 if(event.status!=osEventMessage)
 {
   continue ;
 }
  //continue;
 switch(event.value.v)
 { 
 case MSG_SET_OPT_TYPE:
 {
  APP_LOG_DEBUG("recv msg juice opt reg value:%d\r\n"); 
 juice_asyn_set_opt_type(); 
  }
 break;
 case MSG_GO_TO_TAR_POS:
 {
   APP_LOG_DEBUG("recv msg juice pos reg value:%d\r\n"); 
   juice_asyn_start_sync_task();
 }
 break;
 case MSG_SET_FAULT_CODE:
 {
 APP_LOG_DEBUG("recv msg juice pos reg value:%d\r\n"); 
 juice_asyn_set_fault_code();
 }
 break;
 }
 }
}




void sync_rst()
{
 BSP_row_step_motor_pwr_on_negative(BSP_ROW_STEP_MOTOR_RST_POS_STEPS);//向行复位点运行
 BSP_column_step_motor_pwr_on_negative(BSP_COLUMN_STEP_MOTOR_RST_POS_STEPS);//向列复位点运行
 
 xEventGroupSetBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_RST_EVT|SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_RST_EVT);
 xEventGroupWaitBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_RST_POS_EVT|SYNC_COLUMN_STEP_MOTOR_ON_RST_POS_EVT,pdTRUE, pdTRUE, osWaitForever );//等待复位完成
}

void sync_busy()
{
 //set_juice_progress(JUICE_PROGRESS_EXECUTING); 
}

uint32_t sync_get_row_pos_steps(uint8_t pos)//每个行位置相对于复位点的步数
{
  uint32_t steps;
 if(pos > REG_VALUE_JUICE_ROW_MAX_POS || pos < REG_VALUE_JUICE_ROW_1_POS)//非法行数据，强制为位置1
 steps=BSP_ROW_STEPS_BETWEEN_RST_AND_ROW1;
 else
 steps=BSP_ROW_STEPS_BETWEEN_RST_AND_ROW1+(pos-1)*BSP_STEPS_PER_ROW;//此位置相对于行复位点的步数 pos-1 因为没有0行

 return steps; 
}

uint32_t sync_get_column_pos_steps(uint8_t pos)//每个列位置相对于复位点的步数
{
  uint32_t steps;
 if(pos > REG_VALUE_JUICE_COLUMN_MAX_POS || pos < REG_VALUE_JUICE_COLUMN_1_POS)//非法列数据，强制为位置1
 steps=BSP_COLUMN_STEPS_BETWEEN_RST_AND_COLUMN1;
 else
 steps=BSP_COLUMN_STEPS_BETWEEN_RST_AND_COLUMN1+(pos-1)*BSP_STEPS_PER_COLUMN;//此位置相对于行复位点的步数 pos-1 因为没有0列

 return steps; 
}

uint32_t sync_get_row_tar_steps()//此位置相对于行0点的步数
{
uint32_t reg_value;
uint32_t steps;
uint8_t pos;
reg_value=get_reg_value( JUICE_POS_REGHOLDING_ADDR, 1, REGHOLDING_MODE);
pos=reg_value>>8;
steps=sync_get_row_pos_steps(pos);
return steps;
}
uint32_t sync_get_column_tar_steps()
{
uint32_t reg_value;
uint32_t steps;
uint8_t pos;
reg_value=get_reg_value( JUICE_POS_REGHOLDING_ADDR, 1, REGHOLDING_MODE);  
pos=reg_value;
steps=sync_get_column_pos_steps(pos);
return steps;
}

static void sync_put_up_cup(uint32_t *ptr_cur_steps)
{
  BSP_row_step_motor_pwr_on_positive(BSP_PUT_UP_CUP_STEPS);
 *ptr_cur_steps+=BSP_PUT_UP_CUP_STEPS;//当前步数更新
 
  xEventGroupSetBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT);//使能信号检测
  xEventGroupWaitBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_TAR_POS_EVT,pdTRUE, pdFALSE, osWaitForever );//等待到位
}
static void sync_put_down_cup(uint32_t *ptr_cur_steps)
{
  BSP_row_step_motor_pwr_on_negative(BSP_PUT_UP_CUP_STEPS);
  *ptr_cur_steps-=BSP_PUT_DOWN_CUP_STEPS;//当前步数更新
 
  xEventGroupSetBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT);//使能信号检测
  xEventGroupWaitBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_TAR_POS_EVT,pdTRUE, pdFALSE, osWaitForever );//等待到位
}


static void sync_go_to_row_pos(uint32_t *ptr_cur_steps,uint8_t pos)
{
  uint32_t pos_steps;
  //运行到x点
 pos_steps=sync_get_row_pos_steps(pos);
 if(pos_steps > *ptr_cur_steps)
 BSP_row_step_motor_pwr_on_positive(pos_steps-*ptr_cur_steps);
 else if(pos_steps < *ptr_cur_steps)
 BSP_row_step_motor_pwr_on_negative(*ptr_cur_steps-pos_steps);  
 
 if(pos_steps != *ptr_cur_steps)
 *ptr_cur_steps=pos_steps;//当前步数更新
}
static void sync_go_to_column_pos(uint32_t *ptr_cur_steps,uint8_t pos)
{
 uint32_t pos_steps;
  //运行到y点
 pos_steps=sync_get_column_pos_steps(pos);
 if(pos_steps > *ptr_cur_steps)
 BSP_column_step_motor_pwr_on_positive(pos_steps-*ptr_cur_steps);
 else if(pos_steps < *ptr_cur_steps)
 BSP_column_step_motor_pwr_on_negative(*ptr_cur_steps-pos_steps);  
 
 if(pos_steps != *ptr_cur_steps)
 *ptr_cur_steps=pos_steps;//当前步数更新 
}

static void sync_go_to_pos(uint8_t row_pos,uint8_t column_pos,uint32_t *ptr_cur_row_steps,uint32_t *ptr_cur_column_steps)
{
 sync_go_to_row_pos(ptr_cur_row_steps,row_pos);
 sync_go_to_column_pos(ptr_cur_column_steps,column_pos);
 
 xEventGroupSetBits(sensor_event,SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT|SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_TAR_EVT);//使能信号检测
 xEventGroupWaitBits(sync_event,SYNC_ROW_STEP_MOTOR_ON_TAR_POS_EVT|SYNC_COLUMN_STEP_MOTOR_ON_TAR_POS_EVT,pdTRUE, pdTRUE, osWaitForever );//等待全部到位
}


static void juice_sync_task(void const * argument)
{
 static uint32_t cur_row_steps=0,cur_column_steps=0;
 //static osEvent ret; 
 APP_LOG_DEBUG("juice_sync_task START!\r\n");
 osDelay(2000);
 /*
  //调试角度
  servo_arm_pos(0);
  servo_arm_pos(90);
 servo_arm_pos(180);
 servo_arm_pos(90);
 while(1);
 //调试角度
 */
 
 servo_arm_pos(90);
 servo_grabber_close();

 sync_rst();
 
// sync_go_to_pos(1,1,&cur_row_steps,&cur_column_steps);
 
 
 
while(1)
{

  sync_busy();
  //osDelay(3000);
  APP_LOG_DEBUG("RESTART ------------!\r\n");
  sync_go_to_pos(3,2,&cur_row_steps,&cur_column_steps);//3 1
  //osDelay(1000);
  //sync_put_up_cup(&cur_row_steps);//提起机械手
  //osDelay(1000);
  servo_arm_pos(20);//放到位子上方
  //osDelay(1000);
  servo_grabber_open();//张开
  //osDelay(1000);
  sync_put_down_cup(&cur_row_steps);//下放机械手
  //osDelay(1000);
  servo_grabber_close();//抓紧
  //osDelay(1000);
  sync_put_up_cup(&cur_row_steps);//提起机械手
  //osDelay(1000);
  servo_arm_pos(90);//旋转90度
  //osDelay(1000);
  sync_go_to_pos(1,1,&cur_row_steps,&cur_column_steps);//1 4
  //osDelay(1000);
  servo_arm_pos(180);//旋转90度
  //osDelay(1000);
  sync_put_down_cup(&cur_row_steps);//下放机械手
  //osDelay(1000);
  servo_grabber_open();//张开
  //osDelay(1000);
  sync_put_up_cup(&cur_row_steps);//提起机械手
  //osDelay(1000);
  servo_grabber_close();//抓紧
  //osDelay(1000);
  servo_arm_pos(90);//逆时针旋转90度
  //流程完成 重新抓取3,1
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

void servo_arm_pos(uint16_t angles)
{
  uint16_t time;
  time=160;
  switch(angles)
  {
 case 0:
   time =55;
  break; 
  case 20:
   time =77;
  break; 
  case 90:
  time = 150;
  break;
 case 180:
   time=250;
  break;
  }
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
  MX_TIM2_ReInit_CH4(time);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
  osDelay(1000);  
}

void servo_grabber_close()
{
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_3);
  MX_TIM2_ReInit_CH3(150);//150 抓紧
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_3);
  osDelay(1000);    
}
void servo_grabber_open()
{
  HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_3);
  MX_TIM2_ReInit_CH3(115);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_3);
  osDelay(1000); 
}








