#ifndef  __SLIDEWAY_TASK_H__
#define  __SLIDEWAY_TASK_H__

#define  SLIDEWAY_ASSISTANT_TASK_INTERVAL          5
#define  HORIZONTAL_ENCODER_RESOLUTION             400
#define  PULSES_CNT_EQUIVALENT_TOLERENCE           5
#define  IS_SERVO_POS_EQUIVALENT(a,b)  ((a>=b?a-b:b-a)<=PULSES_CNT_EQUIVALENT_TOLERENCE)

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