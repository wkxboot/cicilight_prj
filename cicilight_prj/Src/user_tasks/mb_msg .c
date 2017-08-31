#include "cmsis_os.h"
#include "app_log.h"

#include "mb_reg.h"
#include "mb_msg.h"


/*****************************************   地址      寄存器       ************/
static void msg_manipulator_opt_type_write(void);        //0x1000  写操作类型事件    
static void msg_manipulator_opt_pos_write(void);         //0x1001  写果汁杯位置事件
static void msg_fault_code_write(void);                  //0x1002  写错误码事件




ptr_regholding_write_handler_t ptr_msg_handler[REG_HOLDING_NREGS]=
{  
/*****************************************   地址         寄存器   ************/
msg_manipulator_opt_type_write,        //0x1000      写操作类型事件
msg_manipulator_opt_pos_write,         //0x1001      写果汁杯位置事件
msg_fault_code_write                   //0x1002      写错误码事件
};

extern QueueHandle_t juice_asyn_msg_queue_hdl;

void mb_msg_init()
{
  osMessageQDef(juice_asyn_msg_queue, 20, uint8_t);
  
  juice_asyn_msg_queue_hdl= osMessageCreate (osMessageQ(juice_asyn_msg_queue),NULL);
  if(juice_asyn_msg_queue_hdl)
  {
  APP_LOG_DEBUG("juice asyn msg queue init successed!\r\n"); 
  }
  else
  {
  APP_LOG_DEBUG("juice asyn msg queue init failed!\r\n");  
  }
  
}


static void msg_manipulator_opt_type_write(void)        //0x1000      写操作类型事件
{
  osStatus status; 
  uint16_t reg_value;
  reg_value= get_reg_value(OPERATION_TYPE_REGHOLDING_ADDR, 1,REGHOLDING_MODE);
  APP_LOG_DEBUG("write opt type reg value:%d\r\n",reg_value);

  status= osMessagePut(juice_asyn_msg_queue_hdl ,MSG_SET_OPT_TYPE,0);
  APP_LOG_DEBUG("send msg:set opt type!status:%d\r\n",status);

 (void)status;
}

static void msg_manipulator_opt_pos_write(void)    //0x1001      写果汁杯位置事件
{
   osStatus status;
   uint16_t reg_value;
   reg_value= get_reg_value(JUICE_POS_REGHOLDING_ADDR, 1,REGHOLDING_MODE);
   APP_LOG_DEBUG("write juice pos reg value:%d\r\n",reg_value);
   

   {
   reg_value= get_juice_fault_code();
   if(!reg_value)
   {
 
   status= osMessagePut (juice_asyn_msg_queue_hdl ,MSG_GO_TO_TAR_POS,0);
   APP_LOG_DEBUG("send msg:set tar pos!status:%d\r\n",status);
   }
   else
   {
    APP_LOG_DEBUG("juice in error, canot send msg to set opt type!\r\n");
   }
   }
  (void)status;
}

static void msg_fault_code_write(void)               //0x1002      写错误码事件
{
   osStatus status;
   uint32_t reg_value;
   reg_value= get_juice_fault_code();
   APP_LOG_DEBUG("write juice fault code reg value:%d\r\n",reg_value);
   
   status= osMessagePut (juice_asyn_msg_queue_hdl ,MSG_SET_FAULT_CODE,0);
   APP_LOG_DEBUG("send msg:set rw! status:%d\r\n",status);  
 
   (void)status;
}

/*
 switch(hi)
   {
   case REG_VALUE_JUICE_ROW_RESET_POS:
   msg_row=MSG_GO_TO_ROW_RESET_POS;
   break;
   case REG_VALUE_JUICE_ROW_1_POS:
   msg_row=MSG_GO_TO_ROW_1_POS;
   break;
   case REG_VALUE_JUICE_ROW_2_POS:
   msg_row=MSG_GO_TO_ROW_2_POS;
   break;
   case REG_VALUE_JUICE_ROW_3_POS:
   msg_row=MSG_GO_TO_ROW_3_POS;
   break;
   default:
   msg_row=MSG_GO_TO_ROW_RESET_POS;  
   }
  switch(lo)
   {
   case REG_VALUE_JUICE_COLUMN_RESET_POS:
   msg_column=MSG_GO_TO_COLUMN_RESET_POS;
   break;
   case REG_VALUE_JUICE_COLUMN_1_POS:
   msg_column=MSG_GO_TO_COLUMN_1_POS;
   break;
   case REG_VALUE_JUICE_COLUMN_2_POS:
   msg_column=MSG_GO_TO_COLUMN_2_POS;
   break;
   case REG_VALUE_JUICE_COLUMN_3_POS:
   msg_column=MSG_GO_TO_COLUMN_3_POS;
   break;
   case REG_VALUE_JUICE_COLUMN_4_POS:
   msg_column=MSG_GO_TO_COLUMN_4_POS;
   break;
   default:
   msg_column=MSG_GO_TO_COLUMN_RESET_POS;  
   }
*/