#ifndef  __RGB_LED_H__
#define  __RGB_LED_H__


#define  RGB_LED_NUM_MAX                  88

#define  RGB_LED_SECTION_CNT              30

#define  RGB_LED_WHITE_COLOR              (255<<16|255<<8|255)
#define  RGB_LED_RED_COLOR                (255<<16|0<<8|0)
#define  RGB_LED_GREEN_COLOR              (0<<16|255<<8|0)
#define  RGB_LED_BLUE_COLOR               (0<<16|0<<8|255)
#define  RGB_LED_YELLOW_COLOR             (255<<16|255<<8|0)
#define  RGB_LED_BLACK_COLOR              (0)



 void rainbow(uint16_t wait_ms);
 void single_color(uint32_t rgb,uint8_t brightness);
 void overwrite_color(uint32_t rgb,uint8_t pos,uint8_t brightness);



#endif