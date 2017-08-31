#include "cmsis_os.h"
#include "app_util.h"
#include "mb.h"
#include "mb_reg.h"
#include "app_log.h"

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
/**************************************************************************/
static eMBErrorCode
eMBRegInputCB_Write( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );
/**
* @brief  获取寄存器的值
* @param pucRegBuffer 
* @param 
* @return 
* @details 
* @see 
*/

uint32_t get_reg_value(uint16_t reg_addr,uint16_t reg_size,reg_mode_t reg_mode)
{
  uint8_t ptr_reg[4]={0};
  uint32_t temp;
  if(reg_size==1)
  {
    if(reg_mode==REGINPUT_MODE)
    eMBRegInputCB(ptr_reg, reg_addr,1);
    if(reg_mode==REGHOLDING_MODE)
    eMBRegHoldingCB(ptr_reg, reg_addr,1,MB_REG_READ);  
    
    temp=ptr_reg[0]<<8|ptr_reg[1];
  }
 else if(reg_size==2)
  {
    if(reg_mode==REGINPUT_MODE)
    eMBRegInputCB(ptr_reg, reg_addr,2);
    if(reg_mode==REGHOLDING_MODE)
    eMBRegHoldingCB(ptr_reg, reg_addr,2,MB_REG_READ); 
    
    temp=ptr_reg[0]<<24|ptr_reg[1]<<16|ptr_reg[2]<<8|ptr_reg[3];
  }
  
 return temp;
}

/**
* @brief 设置寄存器数值 
* @param reg_addr 
* @param reg_size 
* @param value 
* @return none 
* @details --
* @see --
*/
void set_reg_value(uint16_t reg_addr,uint16_t reg_size,uint32_t value,reg_mode_t reg_mode)
{
 uint8_t ptr_reg[4];
 
 if(reg_size==1)
 {
 ptr_reg[0]=(value>>8)&0xFF;
 ptr_reg[1]=value&0xFF;
 if(reg_mode==REGINPUT_MODE)
 eMBRegInputCB_Write(ptr_reg, reg_addr,1 );
 if(reg_mode==REGHOLDING_MODE)
 eMBRegHoldingCB(ptr_reg, reg_addr,1,MB_REG_WRITE);  
 }
  if(reg_size==2)
 {
 ptr_reg[0]=(value>>24)&0xFF;
 ptr_reg[1]=(value>>16)&0xFF;
 ptr_reg[2]=(value>>8)&0xFF;
 ptr_reg[3]=value&0xFF;
 if(reg_mode==REGINPUT_MODE)
 eMBRegInputCB_Write(ptr_reg, reg_addr,2);
 if(reg_mode==REGHOLDING_MODE)
 eMBRegHoldingCB(ptr_reg, reg_addr,2,MB_REG_WRITE);  
 } 
}


/**
* @brief 设置错误码 
* @param -- 
* @return -- 
* @details --
* @see --
*/

void set_juice_fault_code(uint32_t err_code_bit)//每次写一位
{
  uint32_t err_code;
  err_code=get_reg_value( JUICE_FAULT_CODE_REGHOLDING_ADDR,1,REGHOLDING_MODE);
  if((err_code_bit <= 0x03FF) && (~err_code & err_code_bit))//判断是不是错误码（ 0-13 bit)
  {
  err_code|=err_code_bit;
  set_reg_value(JUICE_FAULT_CODE_REGHOLDING_ADDR,1,err_code,REGHOLDING_MODE); 
  }
}
/**
* @brief -- 
* @param -- 
* @return -- 
* @details --
* @see --
*/

uint32_t get_juice_fault_code()
{
  uint32_t err_code;
  err_code=get_reg_value( JUICE_FAULT_CODE_REGHOLDING_ADDR,1,REGHOLDING_MODE)&0x03FF;  
  return err_code ;
}





/*****************************************************************************/
static eMBErrorCode
eMBRegInputCB_Write( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
    
    int  iRegIndex;

    //APP_LOG_INFO("internal write input reg! ADDR:%d NUM:%d\r\n",usAddress,usNRegs);
    
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            usRegInputBuf[iRegIndex] = *pucRegBuffer++ << 8;
            usRegInputBuf[iRegIndex] |= *pucRegBuffer++;
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
} 
  
  
/**
* @brief -- 
* @param -- 
* @return -- 
* @details --
* @see --
*/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
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
    uint16_t prv_value;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                prv_value=usRegHoldingBuf[iRegIndex];
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                /******增加回调处理*****/
                if(prv_value!=usRegHoldingBuf[iRegIndex] && ptr_msg_handler[iRegIndex])//和原来的不相等同时处理函数不为空，就触发写事件
                ptr_msg_handler[iRegIndex]();
                
                iRegIndex++;
                usNRegs--;
            }
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

