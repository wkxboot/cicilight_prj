/*
 * FreeModbus Libary: ARM7/AT91SAM7X Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <assert.h>

/* ----------------------- Platform includes --------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal_uart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "cmsis_os.h"
#include "app_log.h"

#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[portserial]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG    
#endif
/* ----------------------- Defines ------------------------------------------*/
extern UART_HandleTypeDef huart2;

UART_HandleTypeDef * ptr_host_modbus_uart_handle =&huart2;


/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL bOkay = TRUE;
    APP_LOG_DEBUG("MB serial init£¡\r\n"); 
    return bOkay;
}

void
vMBPortSerialClose( void )
{
  APP_LOG_WARNING("MB serial close!\r\n"); 
}

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    
    if(xRxEnable)
    {
      /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(ptr_host_modbus_uart_handle, UART_IT_RXNE);
        /* Enable the UART Transmit data register empty Interrupt */
    __HAL_UART_DISABLE_IT(ptr_host_modbus_uart_handle, UART_IT_TXE);
    
    APP_LOG_WARNING("MB [enable RX ] [disable TX]!\r\n"); 
    }
    else
    {
         /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_DISABLE_IT(ptr_host_modbus_uart_handle, UART_IT_RXNE);
        /* Enable the UART Transmit data register empty Interrupt */
    __HAL_UART_ENABLE_IT(ptr_host_modbus_uart_handle, UART_IT_TXE);
    
       APP_LOG_WARNING("MB [enable TX ] [disable RX]!\r\n"); 
    }
}

void EW_MODBUS_USARTIRQHandler( void )
{
  uint32_t tmp_flag = 0, tmp_it_source = 0; 
  
  tmp_flag = __HAL_UART_GET_FLAG(ptr_host_modbus_uart_handle, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(ptr_host_modbus_uart_handle, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
     pxMBFrameCBByteReceived();
  }

  tmp_flag = __HAL_UART_GET_FLAG(ptr_host_modbus_uart_handle, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(ptr_host_modbus_uart_handle, UART_IT_TXE);
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    pxMBFrameCBTransmitterEmpty();
  }  
}


BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  ptr_host_modbus_uart_handle->Instance->DR = ucByte;
  return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
 *pucByte = (uint8_t)(ptr_host_modbus_uart_handle->Instance->DR & (uint8_t)0x00FF);
 return TRUE;
}

