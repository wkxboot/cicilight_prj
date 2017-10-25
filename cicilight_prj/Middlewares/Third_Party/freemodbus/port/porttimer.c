/* 
 * MODBUS Library: AT91SAM7X/FreeRTOS port
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: porttimer.c,v 1.1 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- FreeRTOS includes --------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "cmsis_os.h"
#define APP_LOG_MODULE_NAME   "[porttimer]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG 
#include "app_log.h"


extern osTimerId SLAVE_MB_timer_hdl;
/* ----------------------- Defines ------------------------------------------*/
#define TIMER_TIMEOUT_INVALID	( 65535U )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
APP_LOG_DEBUG("MB timer 初始化完毕!\r\n");
return TRUE;
}

void vMBPortTimerClose( void )
{
APP_LOG_DEBUG("MB timer 关闭!\r\n");
}

void vMBPortTimersEnable()
{
 /// Start or restart a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \param[in]     millisec      time delay value of the timer.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStart shall be consistent in every CMSIS-RTOS.
  osTimerStart( SLAVE_MB_timer_hdl, 3);
  APP_LOG_DEBUG("MB timer 启动!\r\n");
}

void vMBPortTimersDisable( )
{
/// Stop the timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStop shall be consistent in every CMSIS-RTOS.
  osTimerStop(SLAVE_MB_timer_hdl);
  APP_LOG_DEBUG("MB timer 停止!\r\n");
}


void SLAVE_MB_timer_expired_callback(void const * argument)
{
 APP_LOG_DEBUG("MB timer 时间到达!\r\n");
 pxMBPortCBTimerExpired();
}
