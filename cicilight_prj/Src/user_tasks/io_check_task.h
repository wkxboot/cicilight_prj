#ifndef  __IO_CHECK_TASK_H__
#define  __IO_CHECK_TASK_H__

#ifndef  IO_CHECK_TASK_ENABLE
#define  IO_CHECK_TASK_ENABLE                   1
#endif


#define  CMD_STR_MAX_LEN                        16

/*RTT DEBUG 底层控制 begin*/
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
#define  CMD_GET_TEMPERATURE                    "get t"
#define  CMD_GET_DRV8711_STATUS                 "get 8711"
/*RTT DEBUG 底层控制 end*/

/*RTT DEBUG 高级功能 begin*/
#define  MSG_SERVO1_CLOSE                       "s1 close"
#define  MSG_SERVO1_OPEN                        "s1 open"

#define  MSG_SERVO2_25                          "s2 25"
#define  MSG_SERVO2_90                          "s2 90"
#define  MSG_SERVO2_180                         "s2 180"

#define  MSG_MANIPULATOR_RST                    "m rst"
#define  MANIPULATOR_RST_VALUE                  (SENSOR_POS_IN_ROW_RST<<8|SENSOR_POS_IN_COLUMN_RST)

#define  MSG_PRESS_CUP                          "压杯"
#define  MSG_UNPRESS_CUP                        "松杯"
#define  MSG_OPEN_OH_DOOR                       "开门"
#define  MSG_CLOSE_OH_DOOR                      "关门"

#define  MSG_TURN_ON_COMPRESSOR                 "开压缩机"
#define  MSG_TURN_OFF_COMPRESSOR                "关压缩机"
#define  MSG_TURN_ON_JUICING                    "开榨汁机"
#define  MSG_TURN_OFF_JUICING                   "关榨汁机"

#define  MSG_TURN_ON_STANDBY_RGB_LED            "开待机灯"
#define  MSG_TURN_ON_JUICING_RGB_LED            "开榨汁灯"
#define  MSG_TURN_ON_MOTION_RGB_LED             "开运动灯"
#define  MSG_TURN_ON_INDICATE_RGB_LED           "开提示灯"
#define  MSG_TURN_ON_OK_RGB_LED                 "开完成灯"
#define  MSG_TURN_ON_ERROR_RGB_LED              "开错误灯"
#define  MSG_TURN_ON_RAINBOW_RGB_LED            "开瀑布灯"
#define  MSG_TURN_OFF_RGB_LED                   "关灯"

/*RTT DEBUG 高级功能 end*/


void io_check_task(void const * argument);












#endif