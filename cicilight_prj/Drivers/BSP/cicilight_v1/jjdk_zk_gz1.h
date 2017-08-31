#ifndef  __JJDK_ZK_GZ1_H__
#define  __JJDK_ZK_GZ1_H__

#define  BSP_ROW_STEP_MOTOR_LEN_PER_CIRCLE                 60 //60mm 转一圈长度
#define  BSP_COLUMN_STEP_MOTOR_LEN_PER_CIRCLE              60 //60mm 转一圈长度


#define  BSP_ROW_STEP_MOTOR_STEPS_PER_CIRCLE              12800   //每圈步数
#define  BSP_COLUMN_STEP_MOTOR_STEPS_PER_CIRCLE           (64*200)//每圈步数

#define  BSP_ROW_STEP_MOTOR_STEPS_PER_MM                  (BSP_ROW_STEP_MOTOR_STEPS_PER_CIRCLE/BSP_ROW_STEP_MOTOR_LEN_PER_CIRCLE )     //每毫米步数              
#define  BSP_COLUMN_STEP_MOTOR_STEPS_PER_MM               (BSP_COLUMN_STEP_MOTOR_STEPS_PER_CIRCLE/BSP_COLUMN_STEP_MOTOR_LEN_PER_CIRCLE)//每毫米步数        

#define  BSP_LEN_PER_ROW                                   250//mm 行距
#define  BSP_LEN_PER_COLUMN                                180//mm 列距

#define  BSP_CUP_HIGHT                                     (100-55)//mm 杯子露出高度-(抓取位置高度,从上到下)
#define  BSP_PUT_UP_CUP_LEN                                100//75//mm 杯子提起高度90
#define  BSP_PUT_DOWN_CUP_LEN                              100//75//mm 杯子下放高度
#define  BSP_GRABBER_HIGHT                                 365//mm 机械手高度
#define  BSP_GRABBER_LEN                                   50//mm 机械手长度（只算底座）
#define  BSP_ROW_LEN_BETWEEN_RST_AND_ROW1                  400//mm 第一排到复位点距离     
#define  BSP_COLUMN_LEN_BETWEEN_RST_AND_COLUMN1            240//55 //mm第一列到复位点距离
#define  BSP_COLUMN_OFFSET_LEN_BETWEEN_GRABBER_AND_COLUMN  40//每一列中心点和机械手之间的偏移值(70度对应偏移40mm)



#define  BSP_STEPS_PER_ROW                                (BSP_LEN_PER_ROW*BSP_ROW_STEP_MOTOR_STEPS_PER_MM)
#define  BSP_STEPS_PER_COLUMN                             (BSP_LEN_PER_COLUMN*BSP_COLUMN_STEP_MOTOR_STEPS_PER_MM)

#define  BSP_PUT_UP_CUP_STEPS                             (BSP_PUT_UP_CUP_LEN*BSP_ROW_STEP_MOTOR_STEPS_PER_MM)
#define  BSP_PUT_DOWN_CUP_STEPS                           (BSP_PUT_DOWN_CUP_LEN*BSP_ROW_STEP_MOTOR_STEPS_PER_MM)


#define  BSP_ROW_STEPS_BETWEEN_RST_AND_ROW1               ((BSP_ROW_LEN_BETWEEN_RST_AND_ROW1-BSP_GRABBER_HIGHT+BSP_CUP_HIGHT+BSP_PUT_UP_CUP_LEN)*BSP_ROW_STEP_MOTOR_STEPS_PER_MM)
#define  BSP_COLUMN_STEPS_BETWEEN_RST_AND_COLUMN1         ((BSP_COLUMN_LEN_BETWEEN_RST_AND_COLUMN1-BSP_COLUMN_OFFSET_LEN_BETWEEN_GRABBER_AND_COLUMN-BSP_GRABBER_LEN)*BSP_COLUMN_STEP_MOTOR_STEPS_PER_MM)

#define  BSP_BRAKE_RELEASE_TIMEOUT_VALUE                  100
#define  BSP_RELAY_RELEASE_TIMEOUT_VALUE                  100


#define  BSP_ROW_STEP_MOTOR_RST_POS_STEPS                 0xFFFFFFFF
#define  BSP_COLUMN_STEP_MOTOR_RST_POS_STEPS              0xFFFFFFFF

#define  BSP_TRUE                                         1
#define  BSP_FALSE                                        0


#define  BSP_DIR_POSITIVE                                 1
#define  BSP_DIR_NEGATIVE                                 2

//行步进电机 86
#define  BSP_ROW_STEP_MOTOR_BRAKE_ENABLE_PIN_STATE        GPIO_PIN_RESET
#define  BSP_ROW_STEP_MOTOR_BRAKE_DISABLE_PIN_STATE       GPIO_PIN_SET

#define  BSP_ROW_STEP_MOTOR_EN_ENABLE_PIN_STATE           GPIO_PIN_SET 
#define  BSP_ROW_STEP_MOTOR_EN_DISABLE_PIN_STATE          GPIO_PIN_RESET

#define  BSP_ROW_STEP_MOTOR_DIR_POSITIVE_PIN_STATE        GPIO_PIN_SET
#define  BSP_ROW_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE        GPIO_PIN_RESET

#define  BSP_ROW_STEP_MOTOR_ON_RST_POS_PIN_STATE          GPIO_PIN_RESET

//列步进电机 8711

#define  BSP_COLUMN_STEP_MOTOR_DIR_POSITIVE_PIN_STATE     GPIO_PIN_RESET  
#define  BSP_COLUMN_STEP_MOTOR_DIR_NEGATIVE_PIN_STATE     GPIO_PIN_SET

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


//榨汁电机 
#define  BSP_JUICING_MOTOR_PWR_ON_PIN_STATE                     GPIO_PIN_SET
#define  BSP_JUICING_MOTOR_PWR_DWN_PIN_STATE                    GPIO_PIN_RESET
//升降门电机
#define  BSP_OH_DOOR_MOTOR_DIR_POSITIVE_ENABLE_RELAY_PIN_STATE  GPIO_PIN_SET
#define  BSP_OH_DOOR_MOTOR_DIR_POSITIVE_DISABLE_RELAY_PIN_STATE GPIO_PIN_RESET
#define  BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_ENABLE_RELAY_PIN_STATE  GPIO_PIN_SET
#define  BSP_OH_DOOR_MOTOR_DIR_NEGATIVE_DISABLE_RELAY_PIN_STATE GPIO_PIN_RESET

//压缩机
#define  BSP_COMPRESSOR_PWR_ON_PIN_STATE                        GPIO_PIN_SET
#define  BSP_COMPRESSOR_PWR_DWN_PIN_STATE                       GPIO_PIN_RESET
//环境灯
#define  BSP_ENVIRONMENT_LAMP_PWR_ON_PIN_STATE                  GPIO_PIN_SET            
#define  BSP_ENVIRONMENT_LAMP_PWR_DWN_PIN_STATE                 GPIO_PIN_RESET

//微动开关
#define  BSP_MS_ON_TAR_POS_PIN_STATE                            GPIO_PIN_RESET
//光电开关
#define  BSP_PS_ON_TAR_POS_PIN_STATE                            GPIO_PIN_RESET







void BSP_row_step_motor_init();
void BSP_column_step_motor_init();

void BSP_set_opt_type(uint16_t opt_type);

uint8_t BSP_is_row_step_motor_on_rst_pos();
uint8_t BSP_is_row_step_motor_on_tar_pos();
uint8_t BSP_is_row_step_motor_on_rst_dir();
uint8_t BSP_is_column_step_motor_on_rst_pos();
uint8_t BSP_is_column_step_motor_on_tar_pos();
uint8_t BSP_is_column_step_motor_on_rst_dir();

//微动开关
uint8_t BSP_is_ms_3_on_tar_pos();
uint8_t BSP_is_ms_4_on_tar_pos();
uint8_t BSP_is_ms_5_on_tar_pos();
uint8_t BSP_is_ms_6_on_tar_pos();
uint8_t BSP_is_ms_7_on_tar_pos();
uint8_t BSP_is_ms_8_on_tar_pos();
uint8_t BSP_is_ms_9_on_tar_pos();
uint8_t BSP_is_ms_10_on_tar_pos();

//光电开关
uint8_t BSP_is_ps_1_on_tar_pos();
uint8_t BSP_is_ps_2_on_tar_pos();
uint8_t BSP_is_ps_3_on_tar_pos();
uint8_t BSP_is_ps_4_on_tar_pos();
uint8_t BSP_is_ps_5_on_tar_pos();
uint8_t BSP_is_ps_6_on_tar_pos();





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

void BSP_row_step_motor_pwr_on_positive(uint32_t steps);
void BSP_row_step_motor_pwr_on_negative(uint32_t steps);

void BSP_column_step_motor_pwr_on_positive(uint32_t steps);
void BSP_column_step_motor_pwr_on_negative(uint32_t steps);













#endif