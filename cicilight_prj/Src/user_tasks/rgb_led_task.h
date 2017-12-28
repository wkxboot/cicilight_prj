#ifndef  __RGB_LED_H__
#define  __RGB_LED_H__

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
#define  RGB_LED_CLOSE_MSG                               8


#define  RGB_LED_SECTION_DELAY_VALUE                     0   //每段灯亮起之间的等待时间
#define  RGB_LED_BLINK_DELAY_VALUE                       300 //全部灯闪烁的间隔时间
#define  RGB_LED_OVERWRITE_DELAY_VALUE                   30  //后一个灯亮起需要等待的时间
#define  RGB_LED_INTERVAL_VALUE                          100 //RGB LED空闲时运行间隔时间








#endif