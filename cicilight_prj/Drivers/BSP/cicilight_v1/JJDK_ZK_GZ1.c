
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "tim.h"
#include "mb_reg.h"
#include "drv8711.h"
#include "jjdk_zk_gz1.h"

#include "app_log.h"
    
#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[jjdk]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#endif

uint32_t row_step_motor_tar_steps;
uint32_t column_step_motor_tar_steps;

volatile uint32_t row_step_motor_run_steps;
volatile uint32_t column_step_motor_run_steps;

//内部函数声明
static void BSP_row_step_motor_pwr_dwn();


void BSP_row_step_motor_init()
{
  APP_LOG_DEBUG("bsp row step motor inti!\r\n");

  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE);//高电广 松刹轿 低电广 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_EN_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_EN_POS_Pin, BSP_ROW_STEP_MOTOR_EN_ENABLE_PIN_STATE);
}
void BSP_column_step_motor_init()
{

    APP_LOG_DEBUG("bsp row step motor inti!\r\n");

    HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE);   
    osDelay(2);
    HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_RESET_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_RESET_POS_Pin, BSP_COLUMN_STEP_MOTOR_RESET_ENABLE_PIN_STATE);
    osDelay(2);  
    HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_RESET_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_RESET_POS_Pin, BSP_COLUMN_STEP_MOTOR_RESET_DISABLE_PIN_STATE);
    osDelay(2);  
    HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_ENABLE_PIN_STATE);   
    osDelay(2);
    begin(50, 5, 64) ; 
    get_status();
    set_enable();
}



uint8_t BSP_is_ms_3_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_3_POS_GPIO_Port,BSP_MS_3_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_4_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_4_POS_GPIO_Port,BSP_MS_4_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}

uint8_t BSP_is_ms_5_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_5_POS_GPIO_Port,BSP_MS_5_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_6_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_6_POS_GPIO_Port,BSP_MS_6_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_7_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_7_POS_GPIO_Port,BSP_MS_7_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_8_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_8_POS_GPIO_Port,BSP_MS_8_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_9_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_9_POS_GPIO_Port,BSP_MS_9_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ms_10_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_MS_10_POS_GPIO_Port,BSP_MS_10_POS_Pin);
 if( pinstate == BSP_MS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ps_1_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_1_POS_GPIO_Port,BSP_PS_1_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}

uint8_t BSP_is_ps_2_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_2_POS_GPIO_Port,BSP_PS_2_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ps_3_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_3_POS_GPIO_Port,BSP_PS_3_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ps_4_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_4_POS_GPIO_Port,BSP_PS_4_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ps_5_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_5_POS_GPIO_Port,BSP_PS_5_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}
uint8_t BSP_is_ps_6_on_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_PS_6_POS_GPIO_Port,BSP_PS_6_POS_Pin);
 if( pinstate == BSP_PS_ON_TAR_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret; 
}

//压杯电机
void BSP_press_motor_pwr_on_positive()
{
  HAL_GPIO_WritePin(BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE);
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE); 
}
void BSP_press_motor_pwr_on_negative()
{
  HAL_GPIO_WritePin(BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE); 
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE);
}

void BSP_press_motor_pwr_dwn()
{
 HAL_GPIO_WritePin(BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE);
 HAL_GPIO_WritePin(BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_PRESS_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE);
}
//升降门电机
void BSP_oh_door_motor_pwr_on_positive()
{
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE);
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE); 
}
void BSP_oh_door_motor_pwr_on_negative()
{
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE); 
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE);
}

void BSP_oh_door_motor_pwr_dwn()
{
 HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE);
 HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE);
}

//环境灯
void BSP_environment_lamp_pwr_on()
{
 HAL_GPIO_WritePin(BSP_ENVIRONMENT_RELAY_POS_GPIO_Port,BSP_ENVIRONMENT_RELAY_POS_Pin, BSP_ENVIRONMENT_LAMP_PWR_ON_PIN_STATE);  
}
void BSP_environment_lamp_pwr_dwn()
{
 HAL_GPIO_WritePin(BSP_ENVIRONMENT_RELAY_POS_GPIO_Port,BSP_ENVIRONMENT_RELAY_POS_Pin, BSP_ENVIRONMENT_LAMP_PWR_DWN_PIN_STATE);  
}
//压缩机
void BSP_compressor_pwr_on()
{
 HAL_GPIO_WritePin(BSP_COMPRESSOR_RELAY_POS_GPIO_Port,BSP_COMPRESSOR_RELAY_POS_Pin, BSP_COMPRESSOR_PWR_ON_PIN_STATE);  
}
void BSP_compressor_pwr_dwn()
{
 HAL_GPIO_WritePin(BSP_COMPRESSOR_RELAY_POS_GPIO_Port,BSP_COMPRESSOR_RELAY_POS_Pin, BSP_COMPRESSOR_PWR_DWN_PIN_STATE);  
}
//榨汁电机
void BSP_juicing_motor_pwr_on()
{
 HAL_GPIO_WritePin(BSP_JUICING_MOTOR_RELAY_POS_GPIO_Port,BSP_JUICING_MOTOR_RELAY_POS_Pin, BSP_JUICING_MOTOR_PWR_ON_PIN_STATE);  
}
void BSP_juicing_motor_pwr_dwn()
{
 HAL_GPIO_WritePin(BSP_JUICING_MOTOR_RELAY_POS_GPIO_Port,BSP_JUICING_MOTOR_RELAY_POS_Pin, BSP_JUICING_MOTOR_PWR_DWN_PIN_STATE);  
}

static void BSP_set_row_step_motor_run_steps(uint32_t steps)
{
    taskENTER_CRITICAL();   
    row_step_motor_run_steps=steps;
    taskEXIT_CRITICAL();  
}
static void BSP_set_column_step_motor_run_steps(uint32_t steps)
{
    taskENTER_CRITICAL();   
    column_step_motor_run_steps=steps;
    taskEXIT_CRITICAL();  
}
static uint32_t BSP_get_row_step_motor_run_steps()
{
 return  row_step_motor_run_steps; 
}

static uint32_t BSP_get_column_step_motor_run_steps()
{
 return  column_step_motor_run_steps; 
}

uint8_t BSP_is_row_step_motor_on_rst_dir()
{
  uint32_t steps;
  uint8_t ret;
  steps=BSP_get_row_step_motor_run_steps();
 if( steps > (BSP_ROW_STEP_MOTOR_RST_POS_STEPS/2))
   ret=BSP_TRUE ;
 else
   ret=BSP_FALSE;
 
 return ret;
}
uint8_t BSP_is_column_step_motor_on_rst_dir()
{
  uint32_t steps;
  uint8_t ret;
  steps=BSP_get_column_step_motor_run_steps();
 if( steps > (BSP_COLUMN_STEP_MOTOR_RST_POS_STEPS/2))
   ret=BSP_TRUE ;
 else
   ret=BSP_FALSE;
 
 return ret;
}

uint8_t BSP_is_row_step_motor_on_rst_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_ROW_STEP_MOTOR_RST_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_RST_POS_Pin);
 if( pinstate == BSP_ROW_STEP_MOTOR_ON_RST_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret;  
}
uint8_t BSP_is_column_step_motor_on_rst_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_COLUMN_STEP_MOTOR_RST_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_RST_POS_Pin);
 if( pinstate == BSP_COLUMN_STEP_MOTOR_ON_RST_POS_PIN_STATE)
   ret= BSP_TRUE;
 else
   ret=BSP_FALSE;
 
 return ret;  
}


uint8_t BSP_is_row_step_motor_on_tar_pos()
{
  uint8_t ret;
  uint32_t steps;
  steps=BSP_get_row_step_motor_run_steps();
  if(steps == 0)
  ret=BSP_TRUE;
  else
  ret=BSP_FALSE;
  
  return ret;
}
uint8_t BSP_is_column_step_motor_on_tar_pos()
{
  uint8_t ret;
  uint32_t steps;
  steps=BSP_get_column_step_motor_run_steps();
  if(steps == 0)
  ret=BSP_TRUE;
  else
  ret=BSP_FALSE;
  
  return ret;
}

void BSP_set_opt_type(uint16_t opt_type)
{
  
  
}
static void BSP_row_step_motor_pwr_dwn()
{
  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);  
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车  
}

void BSP_row_step_motor_pwr_on_positive(uint32_t steps)
{
  BSP_row_step_motor_pwr_dwn();//停机
  if(steps==BSP_ROW_STEP_MOTOR_RST_POS_STEPS && BSP_is_row_step_motor_on_rst_pos()==BSP_TRUE)//在行复位点就不运行
  return;  
  BSP_set_row_step_motor_run_steps(steps); 
  
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_DIR_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_DIR_POS_Pin, BSP_ROW_STEP_MOTOR_DIR_POSITIVE_PIN_STATE);//set 向上运动, );
  osDelay(BSP_BRAKE_RELEASE_TIMEOUT_VALUE);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);  
}
void BSP_row_step_motor_pwr_on_negative(uint32_t steps)
{
  BSP_row_step_motor_pwr_dwn();//停机
  if(steps==BSP_ROW_STEP_MOTOR_RST_POS_STEPS && BSP_is_row_step_motor_on_rst_pos()==BSP_TRUE)//在行复位点就不运行
  return;  
  BSP_set_row_step_motor_run_steps(steps);
 
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_DIR_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_DIR_POS_Pin, BSP_ROW_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE);//reset 向下运动, );
  osDelay(BSP_BRAKE_RELEASE_TIMEOUT_VALUE);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);  
}

static void BSP_column_step_motor_pwr_dwn()
{
   HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_ENABLE_PIN_STATE);
   HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);  
}

 void BSP_column_step_motor_pwr_on_positive(uint32_t steps)
{
  BSP_column_step_motor_pwr_dwn();
  if(steps==BSP_COLUMN_STEP_MOTOR_RST_POS_STEPS && BSP_is_column_step_motor_on_rst_pos()==BSP_TRUE)//在列复位点就不运行
  return;     
  BSP_set_column_step_motor_run_steps(steps);
  
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE);
  osDelay(3);
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_DIR_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_DIR_POS_Pin, BSP_COLUMN_STEP_MOTOR_DIR_POSITIVE_PIN_STATE);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);  
}
void BSP_column_step_motor_pwr_on_negative(uint32_t steps)
{
  BSP_column_step_motor_pwr_dwn();
  if(steps==BSP_COLUMN_STEP_MOTOR_RST_POS_STEPS && BSP_is_column_step_motor_on_rst_pos()==BSP_TRUE)//在列复位点就不运行
  return;     
  BSP_set_column_step_motor_run_steps(steps);
  
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE);
  osDelay(3);
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_DIR_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_DIR_POS_Pin, BSP_COLUMN_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);  
}



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
 {
 //列步进电机中断
  void BSP_COLUMN_STEP_MOTOR_PWM_ISR();
  BSP_COLUMN_STEP_MOTOR_PWM_ISR(); 
 }
 else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
 {
  //行步进电机中断
   void BSP_ROW_STEP_MOTOR_PWM_ISR(); 
   BSP_ROW_STEP_MOTOR_PWM_ISR();  
 }
}
/*
 *row步进电机中断处理
 */
void BSP_ROW_STEP_MOTOR_PWM_ISR()
{
 if(row_step_motor_run_steps > 0)
 {
  row_step_motor_run_steps--;
  if(row_step_motor_run_steps==0)
  {
   BSP_row_step_motor_pwr_dwn();
  }
 }
 //复位点
 if( BSP_is_row_step_motor_on_rst_pos() == BSP_TRUE && BSP_is_row_step_motor_on_rst_dir() == BSP_TRUE)
 {
  BSP_row_step_motor_pwr_dwn(); 
  row_step_motor_run_steps=0;//重新开始
 }
 }

/*
 *column步进电机中断处理
 */
void BSP_COLUMN_STEP_MOTOR_PWM_ISR()
{
 if(column_step_motor_run_steps > 0)// column_step_motor_tar_steps)
 {
  column_step_motor_run_steps--;
 // if(column_step_motor_run_steps == BSP_STEP_MOTOR_DELAY_STEPS)
 // {
 //   HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
    
    
    
  //}
  if(column_step_motor_run_steps==0)
  {
   BSP_column_step_motor_pwr_dwn();
  }
  

 }
 //复位点
 if( BSP_is_column_step_motor_on_rst_pos()==BSP_TRUE && BSP_is_column_step_motor_on_rst_dir()==BSP_TRUE)
 {
  BSP_column_step_motor_pwr_dwn(); 
  column_step_motor_run_steps=0;//重新开始
 }
}