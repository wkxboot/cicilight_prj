#ifndef  __SLIDEWAY_TASK_H__
#define  __SLIDEWAY_TASK_H__

/*单位脉冲*/
#define  VERTICAL_ENCODER_RESOLUTION                   800
#define  HORIZONTAL_ENCODER_RESOLUTION                 800

#define  VERTICAL_ENCODER_BASE_PULSES                  0x10000000
#define  HORIZONTAL_ENCODER_BASE_PULSES                0x10000000

#define  VERTICAL_RESET_PULSES                         10000/*垂直方向复位时估计需要的脉冲数*/
#define  HORIZONTAL_RESET_PULSES                       10000/*水平方向复位时估计需要的脉冲数*/
/*单位mm*/
#define  VERTICAL_MM_PER_CIRCLE                         60/*旋转一周的距离*/
#define  HORIZONTAL_MM_PER_CIRCLE                       60/*旋转一周的距离 */
#define  VERTICAL_ENCODER_PULSES_PER_MM                (VERTICAL_ENCODER_RESOLUTION/VERTICAL_MM_PER_CIRCLE)
#define  HORIZONTAL_ENCODER_PULSES_PER_MM              (HORIZONTAL_ENCODER_RESOLUTION/HORIZONTAL_MM_PER_CIRCLE)

#define  ROBOT_BASE_MM                                 200/*机械手基准高度*/
#define  VERTICAL_CUP_BASE_MM                          200/*第一层杯子基准高度*/
#define  VERTICAL_CUP_CLEARANCE_MM                     300/*垂直方向杯子间距*/
#define  VERTICAL_DOCKSITE_CLEARANCE_MM                150 /*停靠点和对应垂直坐标点间距*/
#define  VERTICAL_LIFT_UP_MM                           100/*垂直方向机械手提起距离*/
#define  VERTICAL_PUT_DWN_MM                           100/*垂直方向机械手下方距离*/

#define  VERTICAL_JUICING_MM                           1500
#define  HORIZONTAL_JUICING_MM                         1666666
#define  VERTICAL_SLOT_MM                              1400
#define  HORIZONTAL_SLOT_MM                            1666666
#define  VERTICAL_STANDBY_MM                           1000
#define  HORIZONTAL_STANDBY_MM                         1666666
#define  VERTICAL_RESET_MM                             0
#define  HORIZONTAL_RESET_MM                           1666666

#define  HORIZONTAL_CUP_CNT                             6                            


#define  VERTICAL_CNT                                   7
#define  HORIZONTAL_CNT                                 6

#define  VERTICAL_SERVO_ACCELERATION_PULSES_CNT         200 
#define  VERTICAL_SERVO_DECELERATION_PULSES_CNT         200 
#define  HORIZONTAL_SERVO_ACCELERATION_PULSES_CNT       200 
#define  HORIZONTAL_SERVO_DECELERATION_PULSES_CNT       200 

#define  VERTICAL_SERVO_TOLERANCE_PULSES                5
#define  HORIZONTAL_SERVO_TOLERANCE_PULSES              5

#define  VERTICAL_SERVO_PWR_STEP                        10/*以百分之10为步进值*/
#define  HORIZONTAL_SERVO_PWR_STEP                      10/*以百分之10为步进值*/
#define  VERTICAL_SERVO_NORMAL_PWR                      100
#define  VERTICAL_SERVO_SHOW_PWR                        50
#define  HORIZONTAL_SERVO_NORMAL_PWR                    100
#define  HORIZONTAL_SERVO_SHOW_PWR                      50





#define  VERTICAL_SERVO_UNIT_US                         1
#define  VERTICAL_SERVO_MAX_HZ                          20000/*20kHz*/
#define  VERTICAL_SERVO_MAX_VALUE                       ((1000000/VERTICAL_SERVO_MAX_HZ)/VERTICAL_SERVO_UNIT_US)

#define  HORIZONTAL_SERVO_UNIT_US                       10000
#define  HORIZONTAL_SERVO_MAX_HZ                        100/*100Hz*/
#define  HORIZONTAL_SERVO_MAX_VALUE                     ((1000000/VERTICAL_SERVO_MAX_HZ)/VERTICAL_SERVO_UNIT_US)
/**/
#define  IS_SERVO_POS_EQUIVALENT(ptr_servo,a,b)         ((a>=b?a-b:b-a)<=ptr_servo->encoder.tolerance)

/*滑台内部伺服ID*/
#define  VERTICAL_SERVO_ID                              1
#define  HORIZONTAL_SERVO_ID                            2

/*滑台消息*/
#define  SLIDEWAY_MSG_GOTO_RESET                        1
#define  SLIDEWAY_MSG_GOTO_CUP_TOP                      2
#define  SLIDEWAY_MSG_GOTO_CUP_BOT                      3
#define  SLIDEWAY_MSG_LIFT_UP_CUP                       4
#define  SLIDEWAY_MSG_PUT_INTO_SLOT                     5
#define  SLIDEWAY_MSG_GOTO_JUICING                      6
#define  SLIDEWAY_MSG_GOTO_SLOT                         7
#define  SLIDEWAY_MSG_SERVO_ARRIVE_RESET                8
#define  SLIDEWAY_MSG_SERVO_ARRIVE_NORMAL               9
#define  SLIDEWAY_MSG_SERVO_ERROR                       10
#define  SLIDEWAY_MSG_SERVO_RESTART                     11
#define  SLIDEWAY_MSG_GOTO_STANDBY                      12
#define  SLIDEWAY_MSG_STOP                              13

#endif