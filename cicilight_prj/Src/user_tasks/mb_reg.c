#include "cmsis_os.h"
#include "app_util.h"
#include "mb.h"
#include "mb_reg.h"
#include "juice_common.h"
#include "user_tasks.h"
#define APP_LOG_MODULE_NAME   "[MB_REG]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG 
#include "app_log.h"

/* ----------------------- Static variables ---------------------------------*/
//static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
//static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
/**************************************************************************/

static uint8_t write_opt_type_handle(uint8_t *ptr_buff);//写操作类型参数校验    
static uint8_t write_fault_code_handle(uint8_t *ptr_buffid);//写错误码参数校验   
static uint8_t read_fault_code_handle(uint8_t *ptr_buffid);//读错误码参数校验   
static uint8_t write_t_setting_handle(uint8_t *ptr_buff);
static uint8_t read_t_setting_handle(uint8_t *ptr_buff);
static uint8_t write_debug_cmd_handle(uint8_t *ptr_buff);
static uint8_t read_debug_cmd_handle(uint8_t *ptr_buff);
static uint8_t read_sys_info_handle(uint8_t *ptr_buff);//读取系统所有信息

reg_handler_t reg_holding[REG_HOLDING_PROTOCOL_CNT]={
{write_opt_type_handle,                    NULL,0x1001,2},
{write_fault_code_handle,read_fault_code_handle,0x1003,1},
{write_t_setting_handle,read_t_setting_handle,  0x1004,3},
{write_debug_cmd_handle,read_debug_cmd_handle,  0x1007,2},
};
reg_handler_t reg_input[REG_INPUT_PROTOCOL_CNT]={
{NULL,read_sys_info_handle,0x1101,5}
};

extern osMessageQId sync_msg_queue_hdl;
extern osMessageQId rgb_led_msg_queue_hdl;
extern osMessageQId running_led_msg_queue_hdl;
extern osMessageQId adc_msg_queue_hdl;
extern osMessageQId oh_door_msg_queue_hdl;
extern osMessageQId manipulator_msg_queue_hdl;
extern osMessageQId servo1_msg_queue_hdl;
extern osMessageQId servo2_msg_queue_hdl;
extern osMessageQId presser_msg_queue_hdl;
extern osMessageQId compressor_msg_queue_hdl;
extern osMessageQId juice_msg_queue_hdl;

extern osMutexId err_code_mutex_id;



void mb_reg_init(void)
{
 usRegHoldingBuf[3]=REG_VALUE_DEFAULT_T_SETTING;//默认设置的温度
 usRegHoldingBuf[4]=REG_VALUE_DEFAULT_T_INCREASE;//默认设置上浮值
 usRegHoldingBuf[5]=REG_VALUE_DEFAULT_T_DECREASE;//默认设置下浮值
 
 usRegInputBuf[0]=REG_VALUE_FIRMWARE_VERSION_1<<8|REG_VALUE_FIRMWARE_VERSION_2;
 usRegInputBuf[1]=REG_VALUE_FIRMWARE_VERSION_3<<8|REG_VALUE_FIRMWARE_VERSION_4;
 usRegInputBuf[2]=REG_VALUE_DEVICE_ID_1<<8|REG_VALUE_DEVICE_ID_2;
 usRegInputBuf[3]=REG_VALUE_DEVICE_ID_3<<8|REG_VALUE_DEVICE_ID_4;

 APP_LOG_INFO("++++++++++++++++++++++++++++++++++++++\r\n");
 APP_LOG_INFO("系统寄存器初始化结束！\r\n当前固件版本：Ver%d.%d.%d.%d ！\r\n",usRegInputBuf[0]>>8,usRegInputBuf[0]&0xff,usRegInputBuf[1]>>8,usRegInputBuf[1]&0xff);
 APP_LOG_INFO("当前设备ID：%d.%d.%d.%d ！\r\n",usRegInputBuf[2]>>8,usRegInputBuf[2]&0xff,usRegInputBuf[3]>>8,usRegInputBuf[3]&0xff);
 APP_LOG_INFO("++++++++++++++++++++++++++++++++++++++\r\n");
}

uint8_t get_reg_value(uint16_t reg_addr,uint16_t reg_cnt,uint16_t *ptr_buff,reg_mode_t reg_mode)
{
  if(ptr_buff == NULL)
    return JUICE_FALSE;
  
  if(reg_mode == REGHOLDING_MODE)
  {
  if(reg_addr < REG_HOLDING_START || reg_addr+reg_cnt > REG_HOLDING_START+REG_HOLDING_NREGS)
   return JUICE_FALSE;
  for(uint8_t i=0;i<reg_cnt;i++)
  {
    ptr_buff[i]=usRegHoldingBuf[reg_addr+i-REG_HOLDING_START];
  }
  }
  else if(reg_mode == REGINPUT_MODE)
  {
  if(reg_addr < REG_INPUT_START || reg_addr+reg_cnt > REG_INPUT_START+REG_INPUT_NREGS)
   return JUICE_FALSE;
  
  for(uint8_t i=0;i<reg_cnt;i++)
  {
    ptr_buff[i]=usRegInputBuf[reg_addr+i-REG_INPUT_START];
  }
  }
  else
  return JUICE_FALSE;
  
  return JUICE_TRUE;
}

uint8_t  set_reg_value(uint16_t reg_addr,uint16_t reg_cnt,uint16_t* ptr_buff,reg_mode_t reg_mode)
{
  if(ptr_buff == NULL)
    return JUICE_FALSE;
  
  if(reg_mode == REGHOLDING_MODE)
  {
  if(reg_addr < REG_HOLDING_START || reg_addr+reg_cnt > REG_HOLDING_START+REG_HOLDING_NREGS)
   return JUICE_FALSE;
  
  for(uint8_t i=0;i<reg_cnt;i++)
  {
    usRegHoldingBuf[reg_addr+i-REG_HOLDING_START]=ptr_buff[i];
  }
  }
  else if(reg_mode == REGINPUT_MODE)
  {
  if(reg_addr < REG_INPUT_START || reg_addr+reg_cnt > REG_INPUT_START+REG_INPUT_NREGS)
   return JUICE_FALSE;
  
  for(uint8_t i=0;i<reg_cnt;i++)
  {
    usRegInputBuf[reg_addr+i-REG_INPUT_START]=ptr_buff[i];
  }
  }
 return JUICE_TRUE;
}
/*
* 寄存器读写参数校验和消息发送
*/
static uint8_t write_opt_type_handle(uint8_t *ptr_buff)//榨汁指令处理
{
  uint8_t ret=JUICE_TRUE;
  uint16_t buff[2];
  if(ptr_buff==NULL)
  {
   ret=JUICE_FALSE;
  APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
  return ret;
  }
  buff[0]=*ptr_buff++<<8;
  buff[0]|=*ptr_buff++;
  buff[1]=*ptr_buff++<<8;
  buff[1]|=*ptr_buff++;

  if((buff[0]!=REG_VALUE_OPERATION_TYPE_SHOW       &&\
      buff[0]!=REG_VALUE_OPERATION_TYPE_SELL)      ||\
     (buff[1]>>8)<REG_VALUE_JUICE_ROW_MIN_POS      ||\
     (buff[1]>>8)>REG_VALUE_JUICE_ROW_MAX_POS      ||\
     (buff[1]&0xff)<REG_VALUE_JUICE_COLUMN_MIN_POS ||\
     (buff[1]&0xff)>REG_VALUE_JUICE_COLUMN_MAX_POS)
  {
  ret=JUICE_FALSE;
  APP_LOG_WARNING("写榨汁机目标位置值参数错误！操作类型：%d x：%d y：%d！\r\n",buff[0],buff[1]>>8,buff[1]&0xff);
  return ret;
  }
  else
  {
  APP_LOG_INFO("写榨汁机操作类型值参数正确！类型：%d x：%d y：%d\r\n",buff[0],buff[1]>>8,buff[1]&0xff);
  }
 if(set_reg_value(PROTOCOL_START_JUICE_REG_ADDR,PROTOCOL_START_JUICE_REG_CNT,buff,REGHOLDING_MODE)!=JUICE_TRUE)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("写榨汁操作类型失败！\r\n");
 return ret;
 }
 APP_LOG_INFO("写榨汁操作类型成功！\r\n");
 if(osMessagePut(sync_msg_queue_hdl,SYNC_START_MSG,0)!=osOK)//发送榨汁指令
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("向同步榨汁任务发送消息失败！\r\n");
 return ret;
 }
 APP_LOG_INFO("向同步榨汁任务发送消息成功！\r\n"); 
 
 return ret;
}


static uint8_t write_fault_code_handle(uint8_t *ptr_buff)
{
 uint8_t ret=JUICE_TRUE;
 uint16_t fault_code;
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 fault_code =*ptr_buff++<<8;
 fault_code|=*ptr_buff++;
 if(fault_code!=0)//目前只有清除全部错误，重置进度为空闲一个指令
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("写错误码值参数错误:%d！\r\n",fault_code);
 return ret;
 }
 APP_LOG_INFO("写错误码值参数正确:%d！\r\n",fault_code);
 
if(osMutexWait(err_code_mutex_id,ERR_CODE_MUTEX_TIMEOUT_VALUE)!=osOK)//获取设置错误码的信号量
{
ret=JUICE_FALSE;
APP_LOG_WARNING("获取错误码信号量失败！\r\n"); 
return ret; 
}
APP_LOG_WARNING("获取错误码信号量成功！\r\n"); 
if(set_reg_value(PROTOCOL_FAULT_CODE_REG_ADDR,PROTOCOL_FAULT_CODE_REG_CNT,&fault_code,REGHOLDING_MODE)!=JUICE_TRUE)
{
ret=JUICE_FALSE;
APP_LOG_WARNING("外部强制重置错误码失败！\r\n"); 
}
else
{
APP_LOG_INFO("外部强制重置错误码成功！\r\n"); 
}

osMutexRelease(err_code_mutex_id);
return ret;
}


static uint8_t read_fault_code_handle(uint8_t *ptr_buff)
{
 uint8_t ret=JUICE_TRUE;
 uint16_t fault_code;
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 APP_LOG_INFO("读错误码值参数正确！\r\n");
 if(get_reg_value(PROTOCOL_FAULT_CODE_REG_ADDR,PROTOCOL_FAULT_CODE_REG_CNT,&fault_code,REGHOLDING_MODE)!=JUICE_TRUE)
 {
  APP_LOG_WARNING("外部读取错误码值失败！\r\n");
  return ret;
 }
 *ptr_buff++=fault_code>>8;
 *ptr_buff++=fault_code&0xFF;
 APP_LOG_INFO("外部读取错误码成功！\r\n");

 return ret;
}
 

static uint8_t write_t_setting_handle(uint8_t *ptr_buff)//写温度设置
{
 uint8_t ret=JUICE_TRUE;
 uint16_t buff[3];
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 buff[0] =*ptr_buff++<<8;//t settting
 buff[0]|=*ptr_buff++;
 buff[1] =*ptr_buff++<<8;//t increase
 buff[1]|=*ptr_buff++;
 buff[2] =*ptr_buff++<<8;//t decrease
 buff[2]|=*ptr_buff++;
 
 if((buff[0]<REG_VALUE_MIN_T_SETTING)  ||\
    (buff[0]>REG_VALUE_MAX_T_SETTING)  ||\
    (buff[1]<REG_VALUE_MIN_T_INCREASE) ||\
    (buff[1]>REG_VALUE_MAX_T_INCREASE) ||\
    (buff[2]<REG_VALUE_MIN_T_DECREASE) ||\
    (buff[2]>REG_VALUE_MAX_T_DECREASE))
 {
  ret=JUICE_FALSE;
  APP_LOG_WARNING("写温度设定值参数错误！设置值：%d 上浮值：%d 下浮值：%d！\r\n",buff[0],buff[1],buff[2]);
  return ret;
 }

 APP_LOG_INFO("写温度设定值参数正确！设置值：%d 上浮值：%d 下浮值：%d！\r\n",buff[0],buff[1],buff[2]);
 if(set_reg_value(PROTOCOL_T_SETTING_REG_ADDR,PROTOCOL_T_SETTING_REG_CNT,buff,REGHOLDING_MODE)!=JUICE_TRUE)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("写温度设定值失败！\r\n");
 return ret;
 }
 APP_LOG_INFO("写温度设定值成功！\r\n");
 return ret;
}

static uint8_t read_t_setting_handle(uint8_t *ptr_buff)//读温度设置
{
 uint8_t ret=JUICE_TRUE;
 uint16_t buff[3];
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 APP_LOG_INFO("读温度控制值参数正确！\r\n"); 
 
 if(get_reg_value(PROTOCOL_T_SETTING_REG_ADDR,PROTOCOL_T_SETTING_REG_CNT,buff,REGHOLDING_MODE)!=JUICE_TRUE)
 {
 APP_LOG_WARNING("外部读取温度设置值失败！\r\n");
 return ret;
 }
 for(uint8_t i=0;i<3;i++)
 {
 *ptr_buff++=buff[i]>>8;
 *ptr_buff++=buff[i]&0xFF;
 }
 APP_LOG_INFO("外部读取温度设置值成功！\r\n");
 return ret;
}


static uint8_t write_debug_cmd_handle(uint8_t *ptr_buff)
{
 uint8_t ret=JUICE_TRUE;
 uint16_t buff[2];
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 //参数校验
 buff[0]=*ptr_buff++<<8;
 buff[0]|=*ptr_buff++; 
 if(buff[0]!=REG_VALUE_OH_DOOR_OPEN && buff[0]!=REG_VALUE_OH_DOOR_CLOSE)
 {
 ret=JUICE_FALSE;
 APP_LOG_INFO("写升降门参数错误！写的值：%d\r\n",buff[0]); 
 return ret;
 }
 APP_LOG_INFO("写升降门参数正确！写的值：%d\r\n",buff[0]); 
 
 buff[1]=*ptr_buff++<<8;
 buff[1]|=*ptr_buff++; 
 if(buff[1]!=REG_VALUE_RGB_LED_OPEN && buff[1]!=REG_VALUE_RGB_LED_CLOSE)
 {
 ret=JUICE_FALSE;
 APP_LOG_INFO("写RGB_LED参数错误！写的值：%d\r\n",buff[1]); 
 return ret;
 }
 APP_LOG_INFO("写RGB_LED参数正确！写的值：%d\r\n",buff[1]); 
 
 //把值写入寄存器
 if(set_reg_value(PROTOCOL_DEBUG_REG_ADDR,PROTOCOL_DEBUG_REG_CNT,buff,REGHOLDING_MODE)!=JUICE_TRUE)
 {
  ret=JUICE_FALSE;
  APP_LOG_WARNING("写调试命令失败！\r\n");
  return ret;
 }
  APP_LOG_WARNING("写调试命令成功！\r\n");
 
 if(buff[0]==REG_VALUE_OH_DOOR_OPEN)
 {
 if(osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_OPEN_MSG,0)!=osOK)
 {
 ret=JUICE_FALSE;
 APP_LOG_INFO("向升降门发送开门消息失败！\r\n"); 
 return ret;
 }
 APP_LOG_INFO("向升降门发送开门消息成功！\r\n"); 
 }
 else
 {
 if(osMessagePut(oh_door_msg_queue_hdl,OH_DOOR_CLOSE_MSG,0)!=osOK)
 {
 ret=JUICE_FALSE;
 APP_LOG_INFO("向升降门发送关门消息失败！\r\n"); 
 return ret;
 }
 APP_LOG_INFO("向升降门发送关门消息成功！\r\n");
 }
 
 if(buff[1]==REG_VALUE_RGB_LED_OPEN)
 {
  if(osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_STANDBY_MSG,0)!=osOK)
  {
  ret=JUICE_FALSE;
  APP_LOG_INFO("向RGB_LED任务发送开待机灯消息失败！\r\n"); 
  return ret;
  }
  APP_LOG_INFO("向RGB_LED任务发送开待机灯消息成功！\r\n"); 
 }
 else
 {
 if(osMessagePut(rgb_led_msg_queue_hdl,RGB_LED_CLOSE_MSG,0)!=osOK)
 {
  ret=JUICE_FALSE;
  APP_LOG_INFO("向RGB_LED任务发送关灯消息失败！\r\n"); 
  return ret;
 }
 APP_LOG_INFO("向RGB_LED任务发送关灯消息成功！\r\n");
 } 
 
 return ret;
}

static uint8_t read_debug_cmd_handle(uint8_t *ptr_buff)
{
 uint8_t ret=JUICE_TRUE;
 uint16_t buff[2];
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 APP_LOG_INFO("读单步调试命令参数正确！\r\n");
 
 if(get_reg_value(PROTOCOL_DEBUG_REG_ADDR,PROTOCOL_DEBUG_REG_CNT,buff,REGHOLDING_MODE)!=JUICE_TRUE)
 {
  ret=JUICE_FALSE;
  APP_LOG_WARNING("读取调试命令值失败！\r\n");
  return ret;
 }
 for(uint8_t i=0;i<2;i++)
 {
 *ptr_buff++=buff[i]>>8;
 *ptr_buff++=buff[i]&0xFF;
 }
 APP_LOG_INFO("读取调试命令值成功！\r\n");
 return ret;
}

//输入寄存器
static uint8_t read_sys_info_handle(uint8_t *ptr_buff)//读取系统所有信息
{
 uint8_t ret=JUICE_TRUE;
 uint16_t buff[5];
 if(ptr_buff==NULL)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("错误！寄存器指针为空！\r\n");
 return ret;
 }
 APP_LOG_INFO("读系统信息值参数正确！\r\n");
 if(get_reg_value(PROTOCOL_ALL_INFO_REG_ADDR,PROTOCOL_ALL_INFO_REG_CNT,buff,REGINPUT_MODE)!=JUICE_TRUE)
 {
 ret=JUICE_FALSE;
 APP_LOG_WARNING("外部读取系统信息失败！\r\n"); 
 return ret;
 }
 for(uint8_t i=0;i<5;i++)
 {
 *ptr_buff++=buff[i]>>8;
 *ptr_buff++=buff[i]&0xFF;
 }
 APP_LOG_INFO("外部读取系统信息成功！\r\n"); 
 return ret;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
     for(uint8_t i=0;i<REG_INPUT_PROTOCOL_CNT;i++)
     {
     if(reg_input[i].reg_start==usAddress && reg_input[i].reg_cnt  ==usNRegs)                 
      {
       if(reg_input[i].read_handle==NULL)
       {
       eStatus=MB_EILLSTATE;     
       return eStatus;
       }
       if(reg_input[i].read_handle((uint8_t*)pucRegBuffer)!=JUICE_TRUE)//发送任务消息
       eStatus=MB_EINVAL;
       
       return eStatus;
      }
     }
   eStatus= MB_EINVAL;
   }
   else
   {
   eStatus = MB_ENOREG;
   }
 
 return eStatus;  
}
/****************************************************************************/
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;  
    
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
               for(uint8_t i=0;i<REG_HOLDING_PROTOCOL_CNT;i++)
               {
               if(reg_holding[i].reg_start==usAddress && reg_holding[i].reg_cnt==usNRegs)
               {
               if(reg_holding[i].read_handle==NULL)
               {
               eStatus=MB_EILLSTATE;
               return eStatus;
               }
               if(reg_holding[i].read_handle((uint8_t*)pucRegBuffer)!=JUICE_TRUE)
               eStatus=MB_EINVAL;
               
               return eStatus;
               }
               }
               eStatus= MB_EINVAL;
               break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
             for(uint8_t i=0;i<REG_HOLDING_PROTOCOL_CNT;i++)
              {
              if(reg_holding[i].reg_start==usAddress && reg_holding[i].reg_cnt==usNRegs )
              {                
               if(reg_holding[i].write_handle==NULL)
               {
               eStatus=MB_EILLSTATE;
               return eStatus;
               }
               if(reg_holding[i].write_handle((uint8_t*)pucRegBuffer)!=JUICE_TRUE)
               eStatus=MB_EINVAL;
               
               return eStatus;             
              }
              }
             eStatus= MB_EINVAL;
             break;
            }
    }
    else
    {
    eStatus = MB_ENOREG;
    }
 return eStatus;
}


        
        
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

