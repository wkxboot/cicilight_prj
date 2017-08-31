#ifndef __MB_SLAVE_REG_H__
#define __MB_SLAVE_REG_H__


/* ----------------------- Defines ------------------------------------------*/
#define REG_HOLDING_START           0x1000
#define REG_HOLDING_NREGS           3
#define REG_INPUT_START             0x1100
#define REG_INPUT_NREGS             1

typedef  void (*ptr_regholding_write_handler_t)(void);
extern ptr_regholding_write_handler_t ptr_msg_handler[REG_HOLDING_NREGS];

/********保持寄存器（读写寄存器）*****************/
#define  OPERATION_TYPE_REGHOLDING_ADDR      0x1000
#define  JUICE_POS_REGHOLDING_ADDR           0x1001
#define  JUICE_FAULT_CODE_REGHOLDING_ADDR    0x1002


/********输入寄存器（只读寄存器）*****************/
#define  FIREMWARE_VERSION_REGINPUT_ADDR     0x1100
#define  DEVICE_ID_REGINPUT_ADDR             0x1102
#define  TEMPERATURE_REGINPUT_ADDR           0x1104



/********保持寄存器的取值*************************/
#define  REG_VALUE_OPERATION_TYPE_GRAB       0xFFFF
#define  REG_VALUE_OPERATION_TYPE_MOTION     0x0000 
#define  REG_VALUE_JUICE_ROW_RESET_POS       0x0000
#define  REG_VALUE_JUICE_ROW_1_POS           0x0001
#define  REG_VALUE_JUICE_ROW_2_POS           0x0002
#define  REG_VALUE_JUICE_ROW_3_POS           0x0003
#define  REG_VALUE_JUICE_ROW_4_POS           0x0004
#define  REG_VALUE_JUICE_COLUMN_RESET_POS    0x0000
#define  REG_VALUE_JUICE_COLUMN_1_POS        0x0001
#define  REG_VALUE_JUICE_COLUMN_2_POS        0x0002
#define  REG_VALUE_JUICE_COLUMN_3_POS        0x0003
#define  REG_VALUE_JUICE_COLUMN_4_POS        0x0004

#define  REG_VALUE_JUICE_ROW_MAX_POS         REG_VALUE_JUICE_ROW_4_POS
#define  REG_VALUE_JUICE_COLUMN_MAX_POS      REG_VALUE_JUICE_COLUMN_4_POS
/********输入寄存器的取值*************************/

/**************************************************/
typedef enum
{
 REGINPUT_MODE,
 REGHOLDING_MODE,
}reg_mode_t;


uint32_t get_reg_value(uint16_t reg_addr,uint16_t reg_size,reg_mode_t reg_mode);
void set_reg_value(uint16_t reg_addr,uint16_t reg_size,uint32_t value,reg_mode_t reg_mode);
void set_rm_fault_code(uint32_t err_code_bit);
uint32_t get_rm_fault_code();

#endif