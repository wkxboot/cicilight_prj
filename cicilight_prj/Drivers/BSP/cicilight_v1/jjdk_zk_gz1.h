#ifndef  __JJDK_ZK_GZ1_H__
#define  __JJDK_ZK_GZ1_H__

#define  BSP_TRUE                                         0
#define  BSP_FALSE                                        1


//行步进电机 86
#define  BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE        GPIO_PIN_RESET
#define  BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE       GPIO_PIN_SET

#define  BSP_ROW_STEP_MOTOR_EN_ENABLE_PIN_STATE           GPIO_PIN_SET 
#define  BSP_ROW_STEP_MOTOR_EN_DISABLE_PIN_STATE          GPIO_PIN_RESET

#define  BSP_ROW_STEP_MOTOR_DIR_POSITIVE_PIN_STATE        GPIO_PIN_SET
#define  BSP_ROW_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE        GPIO_PIN_RESET

#define  BSP_ROW_STEP_MOTOR_ON_RST_POS_PIN_STATE          GPIO_PIN_RESET

//列步进电机 8711

#define  BSP_COLUMN_STEP_MOTOR_DIR_POSITIVE_PIN_STATE     GPIO_PIN_SET  
#define  BSP_COLUMN_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE     GPIO_PIN_RESET

#define  BSP_COLUMN_STEP_MOTOR_SLEEP_ENABLE_PIN_STATE     GPIO_PIN_RESET
#define  BSP_COLUMN_STEP_MOTOR_SLEEP_DISABLE_PIN_STATE    GPIO_PIN_SET

#define  BSP_COLUMN_STEP_MOTOR_RESET_ENABLE_PIN_STATE     GPIO_PIN_SET
#define  BSP_COLUMN_STEP_MOTOR_RESET_DISABLE_PIN_STATE    GPIO_PIN_RESET

#define  BSP_COLUMN_STEP_MOTOR_CS_ENABLE_PIN_STATE        GPIO_PIN_SET
#define  BSP_COLUMN_STEP_MOTOR_CS_DISABLE_PIN_STATE       GPIO_PIN_RESET

#define  BSP_COLUMN_STEP_MOTOR_ON_RST_POS_PIN_STATE       GPIO_PIN_RESET

//压杯电机
#define  BSP_PRESS_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE    GPIO_PIN_SET
#define  BSP_PRESS_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE   GPIO_PIN_RESET
#define  BSP_PRESS_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE    GPIO_PIN_SET
#define  BSP_PRESS_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE   GPIO_PIN_RESET
//果杯检测
#define  BSP_CUP_PRESS_OK_POS_PIN_STATE                         GPIO_PIN_RESET
#define  BSP_CUP_IN_SLOT_POS_PIN_STATE                          GPIO_PIN_RESET
#define  BSP_CUP_PRESSER_IN_BOT_POS_PIN_STATE                   GPIO_PIN_RESET
#define  BSP_CUP_PRESSER_IN_TOP_POS_PIN_STATE                   GPIO_PIN_RESET

//榨汁电机 
#define  BSP_JUICING_MOTOR_PWR_ON_PIN_STATE                     GPIO_PIN_SET
#define  BSP_JUICING_MOTOR_PWR_DWN_PIN_STATE                    GPIO_PIN_RESET


//升降门电机
#define  BSP_OH_DOOR_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE  GPIO_PIN_SET
#define  BSP_OH_DOOR_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE GPIO_PIN_RESET
#define  BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE  GPIO_PIN_SET
#define  BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE GPIO_PIN_RESET

#define  BSP_OH_DOOR_HAND_DETECTED_PIN_STATE                    GPIO_PIN_RESET
#define  BSP_OH_DOOR_CLAMP_HAND_PIN_STATE                       GPIO_PIN_RESET
#define  BSP_OH_DOOR_IN_BOT_POS_PIN_STATE                       GPIO_PIN_RESET
#define  BSP_OH_DOOR_IN_TOP_POS_PIN_STATE                       GPIO_PIN_RESET

//压缩机
#define  BSP_COMPRESSOR_PWR_ON_PIN_STATE                        GPIO_PIN_SET
#define  BSP_COMPRESSOR_PWR_DWN_PIN_STATE                       GPIO_PIN_RESET
//环境灯
#define  BSP_ENVIRONMENT_LAMP_PWR_ON_PIN_STATE                  GPIO_PIN_SET            
#define  BSP_ENVIRONMENT_LAMP_PWR_DWN_PIN_STATE                 GPIO_PIN_RESET
//运行灯
#define  BSP_RUN_LED_TURN_ON_PIN_STATE                          GPIO_PIN_SET
#define  BSP_RUN_LED_TURN_OFF_PIN_STATE                         GPIO_PIN_RESET
//微动开关
#define  BSP_MS_IN_TAR_POS_PIN_STATE                            GPIO_PIN_RESET
//光电开关
#define  BSP_PS_IN_TAR_POS_PIN_STATE                            GPIO_PIN_RESET
//输入端口
#define  BSP_COLUMN_STEP_MOTOR_FAULT_PIN_STATE                  GPIO_PIN_RESET
#define  BSP_COLUMN_STEP_MOTOR_STALL_PIN_STATE                  GPIO_PIN_RESET

//超时
#define  BSP_RELAY_RELEASE_TIMEOUT_VALUE                        50
#define  BSP_BRAKE_RELEASE_TIMEOUT_VALUE                        20
#define  BSP_SENSOR_POS_ROW_TIMEOUT_VALUE                       10
#define  BSP_SENSOR_POS_COLUMN_TIMEOUT_VALUE                    10

//温度错误
#define  BSP_ERR_T_VALUE                                        0x7F


//微动开关
uint8_t BSP_is_row_step_motor_in_rst_pos();
uint8_t BSP_is_column_step_motor_in_rst_pos();
uint8_t BSP_is_cup_press_ok();
uint8_t BSP_is_cup_presser_in_bot_pos();
uint8_t BSP_is_cup_presser_in_top_pos();
uint8_t BSP_is_oh_door_clamp_hand();
uint8_t BSP_is_ms_7_in_tar_pos();
uint8_t BSP_is_ms_8_in_tar_pos();
uint8_t BSP_is_ms_9_in_tar_pos();
uint8_t BSP_is_ms_10_in_tar_pos();

//光电开关
uint8_t BSP_is_oh_door_hand_detected();
uint8_t BSP_get_row_pos_sensor_state();   //读取行检测开关状态
uint8_t BSP_get_column_pos_sensor_state();//列检测到开关
uint8_t BSP_is_oh_door_in_top_pos();
uint8_t BSP_is_oh_door_in_bot_pos();
uint8_t BSP_is_cup_in_slot_pos();
//输入端口
uint8_t BSP_is_column_step_motor_fault();
uint8_t BSP_is_column_step_motor_stall();

//adc
int8_t BSP_get_temperature(uint16_t adc_value);

void BSP_row_step_motor_init();
void BSP_column_step_motor_init();
void BSP_set_opt_type(uint16_t opt_type);

void BSP_press_motor_pwr_dwn();
void BSP_press_motor_pwr_on_positive();
void BSP_press_motor_pwr_on_negative();

void BSP_oh_door_motor_pwr_dwn();
void BSP_oh_door_motor_pwr_on_positive();
void BSP_oh_door_motor_pwr_on_negative();

void BSP_environment_lamp_pwr_on();
void BSP_environment_lamp_pwr_dwn();

void BSP_compressor_pwr_on();
void BSP_compressor_pwr_dwn();

void BSP_juicing_motor_pwr_on();
void BSP_juicing_motor_pwr_dwn();

void BSP_row_step_motor_pwr_dwn();
void BSP_row_step_motor_pwr_on_positive();
void BSP_row_step_motor_pwr_on_negative();

void BSP_column_step_motor_pwr_dwn();
void BSP_column_step_motor_pwr_on_positive();
void BSP_column_step_motor_pwr_on_negative();

void BSP_running_led_turn_on();
void BSP_running_led_turn_off();











#endif