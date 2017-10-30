#ifndef  __USER_TASKS_H__
#define  __USER_TAKSS_H__

//机械手默认保持的位置
#define  DEFAULT_ROW_SENSOR_POS                          10//默认机械手位置行
#define  DEFAULT_COLUMN_SENSOR_POS                       8//默认机械手位置列
//超时定义
#define  ADC_TIMEOUT_VALUE                               250//adc采样持续时间
#define  ADC_TASK_RUN_INTERVAL_VALUE                     25 //采样间隔
#define  MANIPULATOR_TIMEOUT_VALUE                       30000//机械手到达目标位置超时时间
#define  MANIPULATOR_ROW_SENSOR_TIMEOUT_VALUE            10//机械手行传感器抖动超时
#define  MANIPULATOR_COLUMN_SENSOR_TIMEOUT_VALUE         10//机械手列传感器抖动超时
#define  MANIPULATOR_INTERVAL_VALUE                      10//机械手传感器采样间隔/运行间隔
#define  MANIPULATOR_COLUMN_MOTOR_STALL_TIMEOUT          2000//启动2秒后检测   
#define  MANIPULATOR_START_FREQUENCY                     1 //khz
#define  MANIPULATOR_EXPIRED_FREQUENCY                   20//khz
#define  MANIPULATOR_STEP_FREQUENCY                      1 //khz
#define  MANIPULATOR_STEP_FREQUENCY_TIMEOUT              100//ms

#define  PRESSER_INTERVAL_VALUE                          20//压杯电机任务运行间隔
#define  PRESSER_OC_DELAY_VALUE                          100//压杯电机启动后开始检查过载的延时时间
#define  PRESSER_TIMEOUT_VALUE                           5000//压杯电机任务超时时间
#define  COMPRESSOR_INTERVAL_VALUE                       20//压缩机任务运行间隔
#define  COMPRESSOR_TOMEOUT_VALUE                        5000//手动操作压缩机时，受控时间

#define  RGB_LED_SECTION_DELAY_VALUE                     0   //每段灯亮起之间的等待时间
#define  RGB_LED_BLINK_DELAY_VALUE                       350 //全部灯闪烁的间隔时间
#define  RGB_LED_OVERWRITE_DELAY_VALUE                   30  //后一个灯亮起需要等待的时间
#define  RGB_LED_INTERVAL_VALUE                          500 //RGB LED空闲时运行间隔时间

#define  JUICING_TIMEOUT_VALUE                           15000//榨汁15s
#define  JUICE_INTERVAL_VALUE                            20//榨汁任务运行间隔
#define  ERR_CODE_MUTEX_TIMEOUT_VALUE                    10//获取错误码信号量超时时间
#define  OH_DOOR_INTERVAL_VALUE                          20//升降门任务运行间隔
#define  OH_DOOR_OC_DELAY_VALUE                          200//升降门在电机启动后开始检查过载的延时时间
#define  OH_DOOR_DETECT_TIMEOUT_VALUE                    5000//5秒钟后再次尝试关门
#define  OH_DOOR_TIMEOUT_VALUE                           16000//16秒操作取杯超时
#define  SERVO1_ENABLE_TIME_VALUE                        1000//舵机1工作使能时间
#define  SERVO1_TIMEOUT_VALUE                            2000//舵机1工作超时时间
#define  SERVO2_ENABLE_TIME_VALUE                        1000//舵机2工作使能时间
#define  SERVO2_TIMEOUT_VALUE                            2000//舵机2工作超时时间
#define  CUP_TIMEOUT_VALUE                               120000//2分钟等待果杯被拿走
#define  CUP_TAKE_AWAY_DELAY_VALUE                       5000 //在果杯被拿走后，等待一些时间再关门
#define  CUP_DETECT_INTERVAL_VALUE                       1000//1秒检测一次果杯是否被拿走
#define  TEMPERATURE_INTERVAL_VALUE                      5000//5秒检测一次温度

//RGB_LED
#define  JUICE_WHEEL_COLOR_CNT                           6//旋转时颜色数量
/*消息*/
#define  NULL_MSG                                        0
#define  RGB_LED_RAINBOW_MSG                             1
#define  RGB_LED_INDICATE_MSG                            2
#define  RGB_LED_JUICING_MSG                             3
#define  RGB_LED_MOTION_MSG                              4
#define  RGB_LED_STANDBY_MSG                             5
#define  RGB_LED_OK_MSG                                  6
#define  RGB_LED_ERROR_MSG                               7
#define  RGB_LED_CLOSE_MSG                               6
#define  OH_DOOR_CLOSE_MSG                               1
#define  OH_DOOR_OPEN_MSG                                2
#define  PRESSER_PRESS_MSG                               1
#define  PRESSER_UNPRESS_MSG                             2
#define  COMPRESSOR_OPEN_MSG                             1
#define  COMPRESSOR_CLOSE_MSG                            2
#define  SERVO1_ANGLE_OPEN_MSG                           115//时间=177
#define  SERVO1_ANGLE_CLOSE_MSG                          152//时间=207
#define  SERVO2_ANGLE_0_MSG                              0
#define  SERVO2_ANGLE_25_MSG                             25
#define  SERVO2_ANGLE_90_MSG                             90
#define  SERVO2_ANGLE_180_MSG                            180
#define  JUICE_START_MSG                                 1
#define  JUICE_STOP_MSG                                  2
#define  SYNC_START_MSG                                  1

         
/*信号*/
#define  OH_DOOR_REACH_TOP_POS_OK_SIGNAL                 (1<<0)
#define  OH_DOOR_REACH_BOT_POS_OK_SIGNAL                 (1<<1)
#define  OH_DOOR_REACH_TOP_POS_ERR_SIGNAL                (1<<2)
#define  OH_DOOR_REACH_BOT_POS_ERR_SIGNAL                (1<<3)
#define  SERVO1_REACH_POS_OK_SIGNAL                      (1<<4)
#define  SERVO2_REACH_POS_OK_SIGNAL                      (1<<5)
#define  MANIPULATOR_REACH_POS_OK_SIGNAL                 (1<<6)
#define  MANIPULATOR_REACH_POS_ERR_SIGNAL                (1<<7)
#define  PRESSER_REACH_TOP_POS_OK_SIGNAL                 (1<<8)
#define  PRESSER_REACH_TOP_POS_ERR_SIGNAL                (1<<9)
#define  PRESSER_REACH_BOT_POS_OK_SIGNAL                 (1<<10)
#define  PRESSER_REACH_BOT_POS_ERR_SIGNAL                (1<<11)
#define  JUICE_TIME_OK_SIGNAL                            (1<<12)
#define  SYNC_ALL_SIGNALS                                (0xFFFF)

/*进度码*/
#define  PROGRESS_ALL_COMPLETED                          0//全部流程结束
#define  PROGRESS_TRANSACTION_EXECUTING                  1//榨汁执行中
#define  PROGRESS_TRANSACTION_FAULT                      2//榨汁出错,交易失败
#define  PROGRESS_TRANSACTION_COMPLETED                  3//榨汁完成，交易成功,正在执行后续流程
#define  PROGRESS_FAULT_AFTER_TRANSACTION_COMPLETED      4//在榨汁和交易完成后，执行后续流程出错

/*错误码*/
#define  FAULT_CODE_24V_OC                               0xB0
#define  FAULT_CODE_OH_DOOR_UP_OC                        0xB1
#define  FAULT_CODE_OH_DOOR_DWN_OC                       0xB2
#define  FAULT_CODE_OH_DOOR_UP_TIMEOUT                   0xB3         
#define  FAULT_CODE_OH_DOOR_DWN_TIMEOUT                  0xB4

#define  FAULT_CODE_PRESSER_PRESS_OC                     0xB5
#define  FAULT_CODE_PRESSER_UNPRESS_OC                   0xB6
#define  FAULT_CODE_PRESSER_PRESS_TIMEOUT                0xB7
#define  FAULT_CODE_PRESSER_UNPRESS_TIMEOUT              0xB8

#define  FAULT_CODE_INTERNAL_ERR                         0xB9
#define  FAULT_CODE_COLUMN_STEP_MOTOR_FAULT              0xBA
#define  FAULT_CODE_MANIPULATOR_TIMEOUT                  0xBB

#define  FAULT_CODE_CUP_NOT_EXIST                        0xBC
#define  FAULT_CODE_CUP_NOT_TAKE_AWAY                    0xBD

#define  FAULT_TEMEPERATURE_SENSOR_ERR                   0xBE
#define  FAULT_TEMEPERATURE_OVER_HIGH_MAX                0xBF
#define  FAULT_TEMEPERATURE_UNDER_LOW_MIN                0xC0

//ADC任务
#define  ADC_CNT                                          5

#define  ADC_PRESSER_IDX                                  0//
#define  ADC_OH_DOOR_IDX                                  1
#define  ADC_24V_IDX                                      2//
#define  ADC_T_IDX                                        3
#define  ADC_BEMF_IDX                                     4

#define  OC_SCALE                                         10//运放比例
#define  RES_VALUE                                        5 //单位0.01欧姆
#define  ADC_24V_OC_THRESHOLD_mAMPERE                     6000//单位mA
#define  ADC_PRESSER_OC_THRESHOLD_mAMPERE                 1350//单位mA，堵转实测1350mA
#define  ADC_OH_DOOR_OC_THRESHOLD_mAMPERE                 4000//单位mA

#define  ADC_BEMF_DIV                                     8
#define  ADC_BEMF_THRESHOLD_mVOLTAGE                      1000 //单位mV  

/*温度任务*/
#define  T_WARNING_HIGH                                   30
#define  T_WARNING_LOW                                    -5


//机械手传感器状态
#define  SENSOR_STATE_NULL                                0xff
#define  SENSOR_STATE_HIGH                                1
#define  SENSOR_STATE_LOW                                 0

//限位点值
#define  NULL_POS                                         0
#define  TOP_POS                                          1
#define  MID_POS                                          2
#define  BOT_POS                                          3
//方向定义
#define  NULL_DIR                                         0xff
#define  POSITIVE_DIR                                     1
#define  NEGATIVE_DIR                                     2

//行方向位置点
#define  SENSOR_POS_IN_ROW_NULL                           0xff
#define  SENSOR_POS_IN_ROW_RST                            3

#define  SENSOR_POS_IN_ROW_1LOW                           4
#define  SENSOR_POS_IN_ROW_1                              6            
#define  SENSOR_POS_IN_ROW_2LOW                           8
#define  SENSOR_POS_IN_ROW_2                              10
#define  SENSOR_POS_IN_ROW_3LOW                           12
#define  SENSOR_POS_IN_ROW_3                              14
#define  SENSOR_POS_IN_ROW_CUP_SLOT                       16
#define  SENSOR_POS_IN_ROW_JUICE_PORT                     18
#define  SENSOR_POS_IN_ROW_STOP                           20

//列方向位置点
#define  SENSOR_POS_IN_COLUMN_NULL                        0
#define  SENSOR_POS_IN_COLUMN_RST                         13

#define  SENSOR_POS_IN_COLUMN_1                           4
#define  SENSOR_POS_IN_COLUMN_JUICE_PORT                  12
#define  SENSOR_POS_IN_COLUMN_2                           6
#define  SENSOR_POS_IN_COLUMN_3                           8
#define  SENSOR_POS_IN_COLUMN_4                           10
#define  SENSOR_POS_IN_COLUMN_5                           12
#define  SENSOR_POS_IN_COLUMN_STOP                        14

//对象状态机
typedef struct 
{
 uint8_t dir;
 uint8_t cur_pos;
 uint8_t tar_pos;
 uint8_t active;
 uint8_t sensor_state;
 uint8_t detect;
 uint16_t detect_timeout;
 uint32_t run_time;
}object_state_t;

//机械手状态机
typedef struct
{
object_state_t row;
object_state_t column;
uint8_t msg_send;
}manipulator_state_t;


#endif