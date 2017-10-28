#ifndef  __IO_CHECK_TASK_H__
#define  __IO_CHECK_TASK_H__

#ifndef  IO_CHECK_TASK_ENABLE
#define  IO_CHECK_TASK_ENABLE                   1
#endif


#define  CMD_STR_MAX_LEN                        16

#define  CMD_IO_CHECK_ENABLE                    "en io"
#define  CMD_IO_CHECK_DISABLE                   "dis io"
#define  CMD_PWR_DWN_PRESSER                    "d presser"
#define  CMD_PWR_ON_POSITIVE_PRESSER            "p presser"
#define  CMD_PWR_ON_NEGATIVE_PRESSER            "n presser"
#define  CMD_PWR_DWN_OH_DOOR                    "d ohdoor"
#define  CMD_PWR_ON_POSITIVE_OH_DOOR            "p ohdoor"
#define  CMD_PWR_ON_NEGATIVE_OH_DOOR            "n ohdoor"
#define  CMD_PWR_ON_ENV_LAMP                    "o elamp"
#define  CMD_PWR_DWN_ENV_LAMP                   "c elamp"
#define  CMD_PWR_ON_COMPRESSOR                  "o compressor"
#define  CMD_PWR_DWN_COMPRESSOR                 "c compressor"
#define  CMD_PWR_ON_JUICING                     "o juice"
#define  CMD_PWR_DWN_JUICING                    "c juice"
#define  CMD_PWR_ON_POSITIVE_ROW_MOTOR          "p r"
#define  CMD_PWR_ON_NEGATIVE_ROW_MOTOR          "n r"
#define  CMD_PWR_DWN_ROW_MOTOR                  "d r"
#define  CMD_PWR_ON_POSITIVE_COLUMN_MOTOR       "p c"
#define  CMD_PWR_ON_NEGATIVE_COLUMN_MOTOR       "n c"
#define  CMD_PWR_DWN_COLUMN_MOTOR               "d c"

#define  CMD_SERVO1_CLOSE                       "s1 close"
#define  CMD_SERVO1_OPEN                        "s1 open"

#define  CMD_SERVO2_25                          "s2 25"
#define  CMD_SERVO2_90                          "s2 90"
#define  CMD_SERVO2_180                         "s2 180"

#define  CMD_MANIPULATOR_GO                     "m g"
#define  MANIPULATOR_GO_MSG                     (10<<8|8)

#define  CMD_GET_TEMPERATURE                    "get t"


#define  MSG_PWR_ON_POSITIVE_PRESSER            "按压果杯"
#define  MSG_PWR_ON_NEGATIVE_PRESSER            "释放果杯"
#define  MSG_PWR_ON_POSITIVE_OH_DOOR            "开门"
#define  MSG_PWR_ON_NEGATIVE_OH_DOOR            "关门"

#define  MSG_PWR_ON_COMPRESSOR                  "开压缩机"
#define  MSG_PWR_DWN_COMPRESSOR                 "关压缩机"
#define  MSG_PWR_ON_JUICING                     "开榨汁机"
#define  MSG_PWR_DWN_JUICING                    "关榨汁机"

#define  MSG_PWR_ON_WHEEL_RGB_LED               "开彩灯"
#define  MSG_PWR_ON_BLINK_RGB_LED               "开闪灯"
#define  MSG_PWR_ON_WHITE_RGB_LED               "开白灯"
#define  MSG_PWR_ON_YELLOW_RGB_LED              "开黄灯"
#define  MSG_PWR_DWN_RGB_LED                    "关灯"


void io_check_task(void const * argument);












#endif