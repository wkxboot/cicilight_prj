#ifndef __MB_SLAVE_REG_H__
#define __MB_SLAVE_REG_H__


/* ----------------------- Defines ------------------------------------------*/
#define REG_HOLDING_START                   0x1001
#define REG_HOLDING_NREGS                   8
#define REG_INPUT_START                     0x1101
#define REG_INPUT_NREGS                     5


typedef  uint8_t (*write_handler_t)(uint8_t *ptr_buff);
typedef  uint8_t (*read_handler_t)(uint8_t *ptr_buff);

typedef enum
{
 REGINPUT_MODE,
 REGHOLDING_MODE,
}reg_mode_t;


typedef struct
{
 write_handler_t write_handle;
 read_handler_t  read_handle;
 uint16_t reg_start;
 uint16_t reg_cnt;
}reg_handler_t;

/********保持寄存器（读写寄存器）*****************/
#define  OPERATION_TYPE_REGHOLDING_ADDR      0x1001
#define  JUICE_POS_REGHOLDING_ADDR           0x1002
#define  JUICE_FAULT_CODE_REGHOLDING_ADDR    0x1003
#define  T_SETTING_REGHOLDING_ADDR           0x1004
#define  T_INCREASE_REGHOLDING_ADDR          0x1005
#define  T_DECREASE_REGHOLDING_ADDR          0x1006
#define  DEBUG_OH_DOOR_REGHOLDING_ADDR       0x1007
#define  DEBUG_RGB_LED_REGHOLDING_ADDR       0x1008

//------协议定义地址
#define  REG_HOLDING_PROTOCOL_CNT            4
#define  REG_INPUT_PROTOCOL_CNT              1

#define  PROTOCOL_START_JUICE_REG_ADDR       OPERATION_TYPE_REGHOLDING_ADDR
#define  PROTOCOL_START_JUICE_REG_CNT        2
#define  PROTOCOL_FAULT_CODE_REG_ADDR        JUICE_FAULT_CODE_REGHOLDING_ADDR
#define  PROTOCOL_FAULT_CODE_REG_CNT         1
#define  PROTOCOL_T_SETTING_REG_ADDR         T_SETTING_REGHOLDING_ADDR
#define  PROTOCOL_T_SETTING_REG_CNT          3
#define  PROTOCOL_DEBUG_REG_ADDR             DEBUG_OH_DOOR_REGHOLDING_ADDR
#define  PROTOCOL_DEBUG_REG_CNT              2

/********输入寄存器（只读寄存器）*****************/
#define  FIREMWARE_VERSION_REGINPUT_ADDR     0x1101
#define  DEVICE_ID_REGINPUT_ADDR             0x1103
#define  TEMPERATURE_REGINPUT_ADDR           0x1105

//------协议定义地址
#define  PROTOCOL_ALL_INFO_REG_ADDR          FIREMWARE_VERSION_REGINPUT_ADDR
#define  PROTOCOL_ALL_INFO_REG_CNT           5


/********保持寄存器的取值*************************/
#define  REG_VALUE_OPERATION_TYPE_SELL       0xFFFF
#define  REG_VALUE_OPERATION_TYPE_SHOW       0x0000 

#define  REG_VALUE_JUICE_ROW_MIN_POS         1
#define  REG_VALUE_JUICE_ROW_MAX_POS         3
#define  REG_VALUE_JUICE_COLUMN_MAX_POS      5
#define  REG_VALUE_JUICE_COLUMN_MIN_POS      1


#define  REG_VALUE_OH_DOOR_OPEN              0xFFFF
#define  REG_VALUE_OH_DOOR_CLOSE             0x0000
#define  REG_VALUE_RGB_LED_OPEN              0xFFFF
#define  REG_VALUE_RGB_LED_CLOSE             0x0000

#define  REG_VALUE_MIN_T_SETTING             4
#define  REG_VALUE_MAX_T_SETTING             10
#define  REG_VALUE_DEFAULT_T_SETTING         8
#define  REG_VALUE_MIN_T_INCREASE            1
#define  REG_VALUE_MAX_T_INCREASE            4
#define  REG_VALUE_DEFAULT_T_INCREASE        4
#define  REG_VALUE_MIN_T_DECREASE            1
#define  REG_VALUE_MAX_T_DECREASE            4
#define  REG_VALUE_DEFAULT_T_DECREASE        4
/********输入寄存器的取值*************************/
//当前版本 1.0.0.48
#define  REG_VALUE_FIRMWARE_VERSION_1        1
#define  REG_VALUE_FIRMWARE_VERSION_2        0
#define  REG_VALUE_FIRMWARE_VERSION_3        0
#define  REG_VALUE_FIRMWARE_VERSION_4        48

#define  REG_VALUE_DEVICE_ID_1               88
#define  REG_VALUE_DEVICE_ID_2               88
#define  REG_VALUE_DEVICE_ID_3               88
#define  REG_VALUE_DEVICE_ID_4               88


/**************************************************/

uint8_t get_reg_value(uint16_t reg_addr,uint16_t reg_cnt,uint16_t *ptr_buff,reg_mode_t reg_mode);
uint8_t set_reg_value(uint16_t reg_addr,uint16_t reg_cnt,uint16_t* ptr_buff,reg_mode_t reg_mode);

void mb_reg_init(void);
#endif