#ifndef  __USER_TASKS_H__
#define  __USER_TAKSS_H__

#define  ROW_STEP_MOTOR_TIMEOUT_VALUE                   8000
#define  COLUMN_STEP_MOTOR_TIMEOUT_VALUE                8000

/*果汁机步进电机同步事件*/
#define  SYNC_ROW_STEP_MOTOR_ON_RST_POS_EVT             (1<<0)
#define  SYNC_ROW_STEP_MOTOR_ON_TAR_POS_EVT             (1<<1)
#define  SYNC_COLUMN_STEP_MOTOR_ON_RST_POS_EVT          (1<<2)
#define  SYNC_COLUMN_STEP_MOTOR_ON_TAR_POS_EVT          (1<<3)
#define  SYNC_START_EVT                                 (1<<4)

/*传感器检测事件*/
#define  SENSOR_CHECK_ROW_STEP_MOTOR_ON_RST_EVT        (1<<0)
#define  SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT        (1<<1)
#define  SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_RST_EVT     (1<<2)
#define  SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_TAR_EVT     (1<<3)

#define  SENSOR_CHECK_ALL_EVT                          (SENSOR_CHECK_ROW_STEP_MOTOR_ON_RST_EVT   |\
                                                       SENSOR_CHECK_ROW_STEP_MOTOR_ON_TAR_EVT    |\
                                                       SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_RST_EVT |\
                                                       SENSOR_CHECK_COLUMN_STEP_MOTOR_ON_TAR_EVT )







#endif