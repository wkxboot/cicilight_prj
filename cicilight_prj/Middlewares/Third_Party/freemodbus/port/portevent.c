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
 * File: $Id: portevent.c,v 1.1 2007/09/12 10:15:56 wolti Exp $
 */
/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "cmsis_os.h"
#include "app_log.h"

#if APP_LOG_ENABLED > 0    
#undef  APP_LOG_MODULE_NAME 
#undef  APP_LOG_MODULE_LEVEL
#define APP_LOG_MODULE_NAME   "[portevent]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG 
#endif


/* ----------------------- Defines ------------------------------------------*/
#define HDL_RESET( x ) do { \
    ( x )->xQueueHdl = 0; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    xQueueHandle    xQueueHdl;
} xEventInternalHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xEventInternalHandle arxEventHdls[1];

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL
xMBPortEventInit( void )
{
    BOOL            bOkay = FALSE;
    xQueueHandle    xQueueHdl;

    ENTER_CRITICAL_SECTION(  );
    xQueueHdl = xQueueCreate( 1, sizeof( eMBEventType ) );
    if( 0 != xQueueHdl )
    {
        arxEventHdls[0].xQueueHdl = xQueueHdl;
        bIsInitialized = TRUE;
        bOkay = TRUE;
    }
    EXIT_CRITICAL_SECTION(  );
    APP_LOG_DEBUG("event queue init!\r\n");
    return bOkay;
}

void
vMBPortEventClose(  )
{
    ENTER_CRITICAL_SECTION(  );
    if( bIsInitialized )
    {
        if( 0 != arxEventHdls[0].xQueueHdl )
        {
            vQueueDelete( arxEventHdls[0].xQueueHdl );
        }
        HDL_RESET( &arxEventHdls[0] );
    }
    EXIT_CRITICAL_SECTION(  );
    APP_LOG_DEBUG("event queen close!");
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    portBASE_TYPE   xEventSent = pdFALSE;

    ENTER_CRITICAL_SECTION(  );
    if( bIsInitialized )
    {
        if( bMBPIsWithinException(  ) )
        {
            xEventSent =
                xQueueSendFromISR( arxEventHdls[0].xQueueHdl, ( const void * )&eEvent, pdFALSE );
        }
        else
        {
            xEventSent = xQueueSend( arxEventHdls[0].xQueueHdl, ( const void * )&eEvent, pdFALSE );
        }
    }
    EXIT_CRITICAL_SECTION();
    APP_LOG_DEBUG("event POST id:%d!\r\n",(uint32_t)eEvent);
    return xEventSent == pdTRUE ? TRUE : FALSE;
}

BOOL
xMBPortEventGet( eMBEventType * peEvent )
{
    BOOL            bEventInQueue = FALSE;

   //ENTER_CRITICAL_SECTION(  );
    if( bIsInitialized )
    {
        if( pdTRUE == xQueueReceive( arxEventHdls[0].xQueueHdl, peEvent, osWaitForever ) )//任务在此等待事件到来 (任务切换)
        {
            bEventInQueue = TRUE;
        }
    }
    //EXIT_CRITICAL_SECTION(  );
    APP_LOG_DEBUG("event GET id:%d!\r\n",(uint32_t)*peEvent);
    return bEventInQueue;
}
