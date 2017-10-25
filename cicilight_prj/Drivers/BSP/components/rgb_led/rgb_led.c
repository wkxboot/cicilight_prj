#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
   
#include "gpio.h"
#include "tim.h"
#include "rgb_led.h"
#include "app_log.h"
    
#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[rgb]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#endif

extern SPI_HandleTypeDef hspi1;

static uint8_t RGB_LED_BUFF[RGB_LED_NUM_MAX][24];

static uint32_t strip_color(uint8_t r, uint8_t g, uint8_t b);
static uint32_t strip_wheel(uint8_t WheelPos);
static void strip_set_pixel_color(uint8_t pos,uint32_t rgb,uint8_t brightness);
static void strip_show();

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
static uint32_t strip_color(uint8_t r, uint8_t g, uint8_t b) 
{
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// Input a value 0 to 83 to get a color value.
// The colours are a transition r - g - b - back to r.
static uint32_t strip_wheel(uint8_t WheelPos) 
{
  WheelPos = (RGB_LED_NUM_MAX-1) - WheelPos;
  
  if(WheelPos < (RGB_LED_NUM_MAX/3))
  {
    return strip_color(255 - WheelPos * (255/(RGB_LED_NUM_MAX/3)),0 , WheelPos * (255/(RGB_LED_NUM_MAX/3)));
  }
  if(WheelPos < (RGB_LED_NUM_MAX/3)*2)
  {
    WheelPos -= (RGB_LED_NUM_MAX/3);
    return strip_color(0, WheelPos * (255/(RGB_LED_NUM_MAX/3)), 255 - WheelPos * (255/(RGB_LED_NUM_MAX/3)));
  }
  WheelPos -= (RGB_LED_NUM_MAX/3)*2;
  return strip_color(WheelPos * (255/(RGB_LED_NUM_MAX/3)), 255 - WheelPos * (255/(RGB_LED_NUM_MAX/3)), 0);
}
/*
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
static uint32_t strip_wheel(uint8_t WheelPos) 
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip_color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip_color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip_color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
*/

//设置一个位置灯的颜色和亮度
static void strip_set_pixel_color(uint8_t pos,uint32_t rgb,uint8_t brightness)
{
  uint8_t r,g,b;
  uint32_t v_color;
  
  
  static uint8_t code0=0x30;
  static uint8_t code1=0x3c;
  
  r=(rgb>>16)& 0xff;
  g=(rgb>>8) & 0xff;
  b=rgb & 0xff;
   
  r=r*(brightness+1)>>8;
  g=g*(brightness+1)>>8;
  b=b*(brightness+1)>>8;
  
  v_color=g<<16|r<<8|b;
  for(uint8_t i=0;i<24;i++)
  {
  if(v_color&(1<<23))
  RGB_LED_BUFF[pos][i]=code1;
  else
  RGB_LED_BUFF[pos][i]=code0; 
  
  v_color<<=1;
  }
  
}
//显示
static void strip_show()
{
 HAL_SPI_Transmit_DMA(&hspi1,(uint8_t*)RGB_LED_BUFF,RGB_LED_NUM_MAX*24); 
}


//彩虹灯
 void rainbow(uint16_t wait_ms) 
{
  uint16_t i, j;

   for(j=0; j<RGB_LED_NUM_MAX; j++)
   {
    //APP_LOG_DEBUG("start time:%d\r\n",osKernelSysTick());
    for(i=0; i<RGB_LED_NUM_MAX; i++)
    {
      strip_set_pixel_color(i, strip_wheel((i+j)),255);
    }
    strip_show();
    //APP_LOG_DEBUG("end time:%d\r\n",osKernelSysTick());
    osDelay(wait_ms);
  }
}
//显示某一种颜色
void single_color(uint32_t rgb,uint8_t brightness)
{ 
  for(uint8_t pos=0;pos<RGB_LED_NUM_MAX;pos++)
  { 
  strip_set_pixel_color( pos,rgb,brightness);
  } 
  strip_show();
}

//瀑布覆盖颜色
void overwrite_color(uint32_t rgb,uint8_t pos,uint8_t brightness)
{ 
 strip_set_pixel_color(pos,rgb,brightness);
 strip_show();
}

/*

 void juicing_color(uint16_t wait_time)
{
 for(uint8_t i=0;i<JUICING_BLINK_CNT;i++)
 {
 single_color(JUICING_YELLOW_COLOR,255);//黄色rgb=255 255 0
 osDelay(JUICING_BLINK_DELAY_TIME);
 single_color(JUICING_BLACK_COLOR,255);//黑色
 osDelay(JUICING_BLINK_DELAY_TIME);
 }
 single_color(JUICING_BLACK_COLOR,255);//黑色
 for(uint8_t i=0;i<JUICING_REPEAT_CNT;i++)
 {
 overwrite_color(JUICING_WHITE_COLOR,RGB_LED_SECTION1_POS,RGB_LED_SECTION1_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_GREEN_COLOR,RGB_LED_SECTION2_POS,RGB_LED_SECTION2_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_BLUE_COLOR,RGB_LED_SECTION3_POS,RGB_LED_SECTION3_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 
 
 overwrite_color(JUICING_YELLOW_COLOR,RGB_LED_SECTION1_POS,RGB_LED_SECTION1_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_WHITE_COLOR,RGB_LED_SECTION2_POS,RGB_LED_SECTION2_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_GREEN_COLOR,RGB_LED_SECTION3_POS,RGB_LED_SECTION3_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 
 
 overwrite_color(JUICING_BLUE_COLOR,RGB_LED_SECTION1_POS,RGB_LED_SECTION1_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_YELLOW_COLOR,RGB_LED_SECTION2_POS,RGB_LED_SECTION2_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_WHITE_COLOR,RGB_LED_SECTION3_POS,RGB_LED_SECTION3_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 
 
 overwrite_color(JUICING_GREEN_COLOR,RGB_LED_SECTION1_POS,RGB_LED_SECTION1_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_BLUE_COLOR,RGB_LED_SECTION2_POS,RGB_LED_SECTION2_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
 overwrite_color(JUICING_YELLOW_COLOR,RGB_LED_SECTION3_POS,RGB_LED_SECTION3_CNT,wait_time,255);
 osDelay(JUICING_SECTION_DELAY_TIME);
}

}
*/