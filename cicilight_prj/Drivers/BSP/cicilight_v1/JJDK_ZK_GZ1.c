
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "tim.h"
#include "mb_reg.h"
#include "drv8711.h"
#include "ntc_3950.h"
#include "juice_common.h"
#include "jjdk_zk_gz1.h"
#define APP_LOG_MODULE_NAME   "[BSP]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#include "app_log.h"

extern struct STATUS_Register 	G_STATUS_REG;

//外部函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//内部函数声明



void BSP_row_step_motor_init()
{
  APP_LOG_DEBUG("BSP 行步进电机初始化！\r\n");

  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE);//高电广 松刹轿 低电广 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_EN_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_EN_POS_Pin, BSP_ROW_STEP_MOTOR_EN_ENABLE_PIN_STATE);
}
void BSP_column_step_motor_init()
{

  APP_LOG_DEBUG("BSP 列不进电机初始化！\r\n");

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

//BSP位置检测
uint8_t BSP_is_row_step_motor_in_rst_pos()//ms1
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_ROW_STEP_MOTOR_RST_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_RST_POS_Pin);
 if( pinstate == BSP_ROW_STEP_MOTOR_ON_RST_POS_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret;  
}
uint8_t BSP_is_column_step_motor_in_rst_pos()//ms2
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_COLUMN_STEP_MOTOR_RST_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_RST_POS_Pin);
 if( pinstate == BSP_COLUMN_STEP_MOTOR_ON_RST_POS_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret;  
}

uint8_t BSP_is_cup_press_ok()//ms3
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_CUP_PRESS_OK_POS_GPIO_Port,BSP_CUP_PRESS_OK_POS_Pin);
 if( pinstate == BSP_CUP_PRESS_OK_POS_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret; 
}
uint8_t BSP_is_cup_presser_in_bot_pos()//ms4
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_CUP_PRESSER_IN_BOT_POS_GPIO_Port,BSP_CUP_PRESSER_IN_BOT_POS_Pin);
 if( pinstate == BSP_CUP_PRESSER_IN_BOT_POS_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret; 
}

uint8_t BSP_is_cup_presser_in_top_pos()//ms5
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_CUP_PRESSER_IN_TOP_POS_GPIO_Port,BSP_CUP_PRESSER_IN_TOP_POS_Pin);
 if( pinstate == BSP_CUP_PRESSER_IN_TOP_POS_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret; 
}



uint8_t BSP_is_oh_door_clamp_hand()//ms6
{
 GPIO_PinState pinstate;
 uint8_t ret;

 pinstate= HAL_GPIO_ReadPin(BSP_OH_DOOR_CLAMP_HAND_POS_GPIO_Port,BSP_OH_DOOR_CLAMP_HAND_POS_Pin);
 if( pinstate == BSP_OH_DOOR_CLAMP_HAND_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret;   
}

uint8_t BSP_is_ms_7_in_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;

 pinstate= HAL_GPIO_ReadPin(BSP_MS_7_POS_GPIO_Port,BSP_MS_7_POS_Pin);
 if( pinstate == BSP_MS_IN_TAR_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret;   
}

uint8_t BSP_is_ms_8_in_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;

 pinstate= HAL_GPIO_ReadPin(BSP_MS_8_POS_GPIO_Port,BSP_MS_8_POS_Pin);
 if( pinstate == BSP_MS_IN_TAR_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret;   
}

uint8_t BSP_is_ms_9_in_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;

 pinstate= HAL_GPIO_ReadPin(BSP_MS_9_POS_GPIO_Port,BSP_MS_9_POS_Pin);
 if( pinstate == BSP_MS_IN_TAR_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret;   
}
uint8_t BSP_is_ms_10_in_tar_pos()
{
 GPIO_PinState pinstate;
 uint8_t ret;

 pinstate= HAL_GPIO_ReadPin(BSP_MS_10_POS_GPIO_Port,BSP_MS_10_POS_Pin);
 if( pinstate == BSP_MS_IN_TAR_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret;   
}
uint8_t BSP_is_column_step_motor_fault()//只检测OCP和OTP错误
{
 GPIO_PinState pinstate;
 uint8_t ret=JUICE_FALSE;
 
 return JUICE_FALSE;
 pinstate= HAL_GPIO_ReadPin(BSP_COLUMN_STEP_MOTOR_FAULT_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_FAULT_POS_Pin);
 if( pinstate == BSP_COLUMN_STEP_MOTOR_FAULT_PIN_STATE)
 {
  ReadAllRegisters();
 if(G_STATUS_REG.AOCP || G_STATUS_REG.BOCP ||G_STATUS_REG.OTS)
 {
 ret= JUICE_TRUE;
 BSP_column_step_motor_init();
 }
 }
 
 return ret; 
}

uint8_t BSP_is_column_step_motor_stall()
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_COLUMN_STEP_MOTOR_STALL_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_STALL_POS_Pin);
 if( pinstate == BSP_COLUMN_STEP_MOTOR_STALL_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret; 
}


uint8_t BSP_is_oh_door_hand_detected()//ps1
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_OH_DOOR_HAND_DETECT_POS_GPIO_Port,BSP_OH_DOOR_HAND_DETECT_POS_Pin);
 if( pinstate == BSP_OH_DOOR_HAND_DETECTED_PIN_STATE)
   ret= JUICE_TRUE;
 else
   ret=JUICE_FALSE;
 
 return ret; 
}

uint8_t BSP_is_oh_door_in_bot_pos()//ps2
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_OH_DOOR_IN_BOT_POS_GPIO_Port,BSP_OH_DOOR_IN_BOT_POS_Pin);
 if( pinstate == BSP_OH_DOOR_IN_BOT_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret; 
}
uint8_t BSP_is_oh_door_in_top_pos()//ps3
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_OH_DOOR_IN_TOP_POS_GPIO_Port,BSP_OH_DOOR_IN_TOP_POS_Pin);
 if( pinstate == BSP_OH_DOOR_IN_BOT_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret; 
}

uint8_t BSP_get_row_pos_sensor_state()//ps4读取行检测开关状态
{
 GPIO_PinState pinstate;
 
 pinstate= HAL_GPIO_ReadPin(BSP_ROW_POS_SENSOR_GPIO_Port,BSP_ROW_POS_SENSOR_Pin);
 return pinstate;  
}

uint8_t BSP_get_column_pos_sensor_state()//ps5读取列检测到开关状态
{
 GPIO_PinState pinstate;
 
 pinstate= HAL_GPIO_ReadPin(BSP_COLUMN_POS_SENSOR_GPIO_Port,BSP_COLUMN_POS_SENSOR_Pin);
 return pinstate;  
}

uint8_t BSP_is_cup_in_slot_pos()//ps6 果杯存在检测
{
 GPIO_PinState pinstate;
 uint8_t ret;
 
 pinstate= HAL_GPIO_ReadPin(BSP_CUP_POS_GPIO_Port,BSP_CUP_POS_Pin);
 if( pinstate == BSP_CUP_IN_SLOT_POS_PIN_STATE)
 ret= JUICE_TRUE;
 else
 ret=JUICE_FALSE;
 
 return ret; 
}
//系统运行灯
void BSP_running_led_turn_on()
{
 HAL_GPIO_WritePin(BSP_RUN_LED_POS_GPIO_Port,BSP_RUN_LED_POS_Pin ,BSP_RUN_LED_TURN_ON_PIN_STATE);
}
void BSP_running_led_turn_off()
{
 HAL_GPIO_WritePin(BSP_RUN_LED_POS_GPIO_Port,BSP_RUN_LED_POS_Pin ,BSP_RUN_LED_TURN_OFF_PIN_STATE);
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
  APP_LOG_INFO("升降门正转！\r\n");
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE);
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE); 
}
void BSP_oh_door_motor_pwr_on_negative()
{
  APP_LOG_INFO("升降门反转！\r\n");
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE); 
  osDelay(BSP_RELAY_RELEASE_TIMEOUT_VALUE);//等待释放
  HAL_GPIO_WritePin(BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port,BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin, BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE);
}

void BSP_oh_door_motor_pwr_dwn()
{
 APP_LOG_INFO("升降门断电！\r\n");
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

int8_t BSP_get_temperature(uint16_t adc_value)
{
  int8_t t;
  t= ntc_3950_get_t(adc_value);
  if(t==NTC_ERROR_T_VALUE)
  return BSP_ERR_T_VALUE;
  
  return t;
}


void BSP_row_step_motor_pwr_dwn()
{
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);  
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车  
}

void BSP_row_step_motor_pwr_on_positive()
{
  BSP_row_step_motor_pwr_dwn();//停机
  
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_DIR_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_DIR_POS_Pin, BSP_ROW_STEP_MOTOR_DIR_POSITIVE_PIN_STATE);//set 向上运动, );
  osDelay(BSP_BRAKE_RELEASE_TIMEOUT_VALUE);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  
}
void BSP_row_step_motor_pwr_on_negative()
{
  BSP_row_step_motor_pwr_dwn();//停机
 
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin, BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE);//高电平 松刹车 低电平 拉紧刹车
  HAL_GPIO_WritePin(BSP_ROW_STEP_MOTOR_DIR_POS_GPIO_Port,BSP_ROW_STEP_MOTOR_DIR_POS_Pin, BSP_ROW_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE);//reset 向下运动, );
  osDelay(BSP_BRAKE_RELEASE_TIMEOUT_VALUE);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  
}

 void BSP_column_step_motor_pwr_dwn()
{
   HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_ENABLE_PIN_STATE);
   HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);  
}

 void BSP_column_step_motor_pwr_on_positive()
{
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE);
  osDelay(3);
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_DIR_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_DIR_POS_Pin, BSP_COLUMN_STEP_MOTOR_DIR_POSITIVE_PIN_STATE);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  
}
void BSP_column_step_motor_pwr_on_negative()
{ 
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port,BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin, BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE);
  osDelay(3);
  HAL_GPIO_WritePin(BSP_COLUMN_STEP_MOTOR_DIR_POS_GPIO_Port, BSP_COLUMN_STEP_MOTOR_DIR_POS_Pin, BSP_COLUMN_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  
}
