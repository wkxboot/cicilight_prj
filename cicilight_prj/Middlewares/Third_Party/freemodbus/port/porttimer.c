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
#include "app_log.h"

#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[porttimer]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG 
#endif


extern osTimerId SLAVE_MB_timer_hdl;
/* ----------------------- Defines ------------------------------------------*/
#define TIMER_TIMEOUT_INVALID	( 65535U )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
APP_LOG_DEBUG("MB timer init!\r\n");
return TRUE;
}

void vMBPortTimerClose( void )
{
APP_LOG_DEBUG("MB timer close!\r\n");
}

void vMBPortTimersEnable()
{
 /// Start or restart a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \param[in]     millisec      time delay value of the timer.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStart shall be consistent in every CMSIS-RTOS.
  osTimerStart (SLAVE_MB_timer_hdl, 3);
}

void vMBPortTimersDisable( )
{
/// Stop the timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStop shall be consistent in every CMSIS-RTOS.
  APP_LOG_DEBUG("MB timer stop!\r\n");
  osTimerStop (SLAVE_MB_timer_hdl);
 
}


void SLAVE_MB_timer_expired_callback(void const * argument)
{
 APP_LOG_DEBUG("MB timer expired!\r\n");
 pxMBPortCBTimerExpired();
}
